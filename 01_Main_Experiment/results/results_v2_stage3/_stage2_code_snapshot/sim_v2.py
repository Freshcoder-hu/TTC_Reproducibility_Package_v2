"""V2 orchestration: the frozen V1 loop plus the Stage-1 execution layer.

``run_scene_v2`` is a line-by-line descendant of ``simulation.run_scene``.  The
V1 loop (ordering of controller calls, view refreshes, obstacle updates,
collision accounting, log fields, RNG use, termination) is preserved exactly;
V2 inserts

* ``RobotFleetV2`` (turn rate as a state + joint caps + feasibility layer),
* ``advance_sheep_m0`` (C3 imposed on the velocity vector),
* continuous swept contact detection (read-only, extra metrics only),
* state-based limit-violation accounting.

With ``v2cfg = compat_v1cfg()`` and the archived V1 ``SimCfg``, every V2
insertion is a no-op and the run reproduces the archive bit-exactly
(acceptance test A).
"""
from __future__ import annotations

import time as _time

import numpy as np

import simulation
from obstacle import Obstacle
from path import Path  # noqa: F401  (re-exported for convenience)
from sheep import init_flock
from simulation import View, largest_cluster, _wrap
from robots import slot_positions

from v2.animal_m0 import advance_sheep_m0, sheep_velocity
from v2.animal_models import FAR, make_animal_model
from v2.contacts_v2 import ContactMonitor
from v2.feasibility_v2 import FeasibilityLayer
from v2.fleet_v2 import RobotFleetV2
from v2.limits_v2 import Counters, empty_stats


def _init_robots_ring_v2(view, cfg, s0, form="ring", R_fl=3.0):
    c = view.centroid if len(view.sheep_pos) else view.path.point_at(s0)
    tang = view.path.tangent_at(s0)
    slots, _ = slot_positions(c, R_fl, tang, cfg, form=form)
    fleet = RobotFleetV2(slots, cfg=cfg)
    fleet.head[:] = np.arctan2(tang[1], tang[0])
    return fleet


def _obs_arrays(obs, alive_only=False):
    if not obs:
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0, dtype=int)
    pos = np.array([o.pos for o in obs], dtype=float).reshape(-1, 2)
    vel = np.array([o.vel for o in obs], dtype=float).reshape(-1, 2)
    ids = np.arange(len(obs))
    if alive_only:
        m = np.array([o.is_alive() for o in obs], dtype=bool)
        return pos[m], vel[m], ids[m]
    return pos, vel, ids


def run_scene_v2(cfg, path, obstacles, controller, seed=0, T_max=180.0,
                 s0=8.0, trace=0.0, init=None, v2cfg=None,
                 contact=True, feasibility=True, projection="joint",
                 record_events=False, record_states=False,
                 record_animal=False, flock_init=None):
    """One V2 run.  Returns the V1 result dict plus a ``v2`` sub-dict.

    ``record_states`` (Stage 2) appends a *read-only* snapshot of the full
    plant state after every integration step.  It performs no arithmetic on
    the state, so switching it on cannot change the dynamics; it exists so
    bit-exactness claims can be checked on the whole trajectory instead of on
    a summary row.  ``record_animal`` does the same for the behaviour model's
    per-animal diagnostics (stimulus, surface gap, speed, heading).

    ``flock_init`` (Stage 2) overrides the initial flock construction, for the
    controlled behaviour test bed only (``v2.corridor_v2``).  The default
    ``None`` keeps ``sheep.init_flock`` exactly as the frozen protocol uses it.
    """
    from v2.config_v2 import V2Cfg
    v2cfg = v2cfg or V2Cfg()
    init = init or {}
    hard = v2cfg.hard_contacts()

    rng = np.random.default_rng(seed)
    flock = (init_flock(cfg.n_sheep, path.point_at(s0), radius=2.8, rng=rng)
             if flock_init is None else flock_init(seed, cfg, path, s0))
    view = View()
    view.path = path
    view.sheep = flock
    view.refresh(flock, RobotFleetV2(np.zeros((cfg.n_robots, 2))), cfg, obstacles)
    robots = _init_robots_ring_v2(view, cfg, s0, form=init.get("form", "ring"),
                                  R_fl=init.get("R_fl", 3.0))

    obs = [simulation.Obstacle(o.start.copy(), o.vel.copy(), o.radius, o.life,
                               o.t_spawn, o.a_max) for o in obstacles]

    n0 = cfg.n_sheep
    lost = 0
    collided = np.zeros(n0, dtype=bool)
    ctrl_ms_acc = 0.0
    ctrl_n = 0
    step = 0
    logs = []
    success_t = None
    timeout = False
    terminated_by = "time"
    t = 0.0
    dt = cfg.dt
    cp = cfg.ctrl_period
    cmd = np.zeros((cfg.n_robots, 2))
    prev_cmd = None
    tag = "init"

    tol = cfg.viol_tol
    robot_viol = sheep_viol = n_dt = 0
    dmin_ro = np.inf
    dmin_rr = np.inf
    rr_info = (np.inf, 0.0, "?", -1, -1)
    dmin_so = np.inf
    cvar_acc = 0.0
    cvar_n = 0
    out_normal = set()
    out_risk = set()
    seen_out = set()

    counters = Counters()
    mon = ContactMonitor(hard, tol=v2cfg.contact_tol) if contact else None
    trace_rows = [] if record_states else None
    animal = make_animal_model(v2cfg, cfg, seed, record=record_animal)
    feas = None
    if v2cfg.feas_enabled and feasibility:
        feas = FeasibilityLayer(v2cfg, cfg, counters, hard)
    C_diag = {}                     # (cls,i,j) -> dict  (post-hoc diagnosis)
    C_done = set()

    while t < T_max:
        view.t = t
        view.refresh(flock, robots, cfg, obs)

        ctrl_tick = (step % cp == 0)
        if ctrl_tick:
            t0 = _time.perf_counter()
            cmd, tag = controller.step(view, cfg)
            ctrl_ms_acc += (_time.perf_counter() - t0) * 1e3
            ctrl_n += 1
            if prev_cmd is not None:
                cvar_acc += float(np.linalg.norm(
                    cmd - prev_cmd, axis=1).mean())
                cvar_n += 1
            prev_cmd = cmd.copy()

        # ---- snapshots for the swept detector and the violation check ----
        shead0 = flock.head[flock.alive].copy()
        svel0 = sheep_velocity(flock)
        r_speed0 = robots.speed.copy()
        r_head0 = robots.head.copy()
        r_omega0 = robots.omega.copy()
        rob0 = robots.pos.copy()
        shp0 = flock.pos[flock.alive].copy()
        opos0, ovel0, oids0 = _obs_arrays(obs)
        alive_start = np.array([o.is_alive() for o in obs], dtype=bool) \
            if obs else np.zeros(0, dtype=bool)

        robots.step(cmd, cfg, v2cfg,
                    obs_pos=view.obs_pos if view.obs_active_n else None,
                    obs_vel=view.obs_vel if view.obs_active_n else None,
                    sheep_pos=shp0, sheep_vel=svel0,
                    t=t, ctrl_tick=ctrl_tick, feas=feas, counters=counters,
                    obs_ids=oids0[alive_start] if obs else None)
        # ---- Stage-2 hook: the behaviour model ------------------------
        #   ``animal.step`` replaces the *desired-velocity* function only; the
        #   (V2) box projection inside it is the Stage-1 one.  For
        #   animal_model="M0" it is a verbatim call of the Stage-1
        #   ``advance_sheep_m0`` with the same arguments, which is what keeps
        #   M0 bit-exact.
        if animal.senses and obs:
            op = np.where(alive_start[:, None], opos0, FAR)
            orr = np.array([o.radius for o in obs], dtype=float)
        else:
            op, orr = None, None
        animal.step(flock, cfg, v2cfg, robots.pos, path, counters,
                    projection=projection, obs_pos=op, obs_r=orr, t=t)

        if trace_rows is not None:
            trace_rows.append((
                float(t), int(step),
                flock.alive.copy(), flock.pos.copy(), flock.head.copy(),
                flock.speed.copy(), robots.pos.copy(), robots.head.copy(),
                robots.speed.copy(), robots.omega.copy(),
                (_obs_arrays(obs)[0].copy()),
            ))

        # ---- V1 constraint check (kept verbatim) -------------------------
        if np.any(robots.speed > cfg.v_max * (1 + tol)):
            robot_viol += 1
        dth = _wrap(robots.head - r_head0)
        if np.any(np.abs(dth) / dt > cfg.omega_max * (1 + tol)):
            robot_viol += 1
        spd = flock.speed[flock.alive]
        if spd.size:
            if np.any(spd > cfg.v_flee * (1 + tol)) or \
                    np.any(spd < cfg.v_base * (1 - tol)):
                sheep_viol += 1
            dth_s = _wrap(flock.head[flock.alive] - shead0)
            if np.any(np.abs(dth_s) / dt > cfg.turn_max * (1 + tol)):
                sheep_viol += 1
        n_dt += 1

        # ---- V2 state-based limit accounting -----------------------------
        if v2cfg.enforce_r4:
            dw = np.abs(robots.omega - r_omega0)
            bad = dw > v2cfg.a_ang * dt * (1 + v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_R4", int(bad.sum()))
        if v2cfg.enforce_lat:
            lat = robots.speed * np.abs(robots.omega)
            bad = lat > v2cfg.a_lat * (1 + v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_LAT", int(bad.sum()))
        dvr = np.abs(robots.speed - r_speed0)
        bad = dvr > cfg.a_lin * dt * (1 + v2cfg.viol_tol)
        if bad.any():
            counters.hit("viol_R3", int(bad.sum()))
        bad = robots.speed > cfg.v_max * (1 + v2cfg.viol_tol)
        if bad.any():
            counters.hit("viol_R1", int(bad.sum()))
        bad = np.abs(_wrap(robots.head - r_head0)) / dt \
            > cfg.omega_max * (1 + v2cfg.viol_tol)
        if bad.any():
            counters.hit("viol_R2", int(bad.sum()))
        if v2cfg.enforce_c3 and svel0.size:
            A = v2cfg.a_sheep * dt
            dv = np.linalg.norm(sheep_velocity(flock) - svel0, axis=1)
            bad = dv > A * (1 + v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_C3", int(bad.sum()))
            # The model's own admissible C1 band (Stage 2): [s_lo, s_hi] of the
            # behaviour model.  For M0 these are exactly V2Cfg.s_min/s_max, so
            # the Stage-1 arithmetic and counter values are unchanged.
            bad = spd < animal.s_lo * (1 - v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_C1_low", int(bad.sum()))
            bad = spd > animal.s_hi * (1 + v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_C1_high", int(bad.sum()))
            bad = np.abs(_wrap(flock.head[flock.alive] - shead0)) / dt \
                > cfg.turn_max * (1 + v2cfg.viol_tol)
            if bad.any():
                counters.hit("viol_C2", int(bad.sum()))

        # ---- V1 robot-safety distance statistics (kept verbatim) --------
        if cfg.n_robots > 1:
            for i in range(cfg.n_robots):
                for j in range(i + 1, cfg.n_robots):
                    dd = float(np.linalg.norm(robots.pos[i] - robots.pos[j]))
                    if dd < dmin_rr:
                        dmin_rr = dd
                        rr_info = (dd, t, tag, i, j)
        for o in obs:
            if o.is_alive():
                if len(robots.pos):
                    dmin_ro = min(dmin_ro, float(np.linalg.norm(
                        robots.pos - o.pos, axis=1).min()))
                if flock.alive.any():
                    dd = np.linalg.norm(flock.pos[flock.alive] - o.pos,
                                        axis=1).min()
                    dmin_so = min(dmin_so, float(dd))

        for o in obs:
            o.update(t + dt)

        # ---- V2 continuous contact detection ----------------------------
        if mon is not None:
            alive_end = np.array([o.is_alive() for o in obs], dtype=bool) \
                if obs else np.zeros(0, dtype=bool)
            use = alive_start | alive_end
            opos1, _, oids1 = _obs_arrays(obs)
            mon.update(
                t, dt,
                rob=(rob0, robots.pos.copy()),
                sheep=(shp0, flock.pos[flock.alive].copy()),
                obs=(opos0[use], opos1[use]),
                obs_ids=oids0[use],
                rob_head0=r_head0, rob_v=r_speed0, rob_omega=r_omega0,
                n_sub=v2cfg.contact_substeps,
                arc=(v2cfg.contact_mode == "arc"))
            # post-hoc C diagnosis on newly started contact episodes
            if v2cfg.c_diag_enabled and feas is not None:
                for key, t_ev in mon.new_events:
                    if key[0] not in ("rr", "ro", "rs"):
                        continue        # animal-animal contacts are not C events
                    if key in C_done:
                        continue
                    C_done.add(key)
                    feas.classify_contact(float(t_ev), key[0], key[1], key[2],
                                          None)

        # ---- V1 collision detection (sheep-obstacle), kept verbatim -----
        view.refresh(flock, robots, cfg, obs)
        if view.obs_active_n and flock.alive.any():
            for i in np.where(flock.alive)[0]:
                p = flock.pos[i]
                hit = any(np.linalg.norm(p - o.pos) < cfg.r_sheep + o.radius
                          for o in view.obstacles_active)
                if hit and not collided[i]:
                    collided[i] = True
                    lost += 1

        # ---- logs (kept verbatim) ---------------------------------------
        if ctrl_tick:
            n_alive = int(flock.alive.sum())
            pos = view.sheep_pos_global
            ttc_min = np.inf
            if n_alive:
                inside = path.inside_frac(pos)
                rho = float(inside * n_alive / n0)
                s_c, lat_c, _ = path.closest(pos.mean(axis=0))
                cl = largest_cluster(pos, cfg.cohesion_r)
                cflock = float(cl.sum()) / n_alive
                risk_active = bool(view.obs_active_n) and \
                    np.isfinite(ttc_min) and (ttc_min < cfg.risk_ref_ttc)
                for k in np.where(~cl)[0]:
                    if k not in seen_out:
                        seen_out.add(int(k))
                        if risk_active:
                            out_risk.add(int(k))
                        else:
                            out_normal.add(int(k))
                dmin_s = np.inf
                for o in view.obstacles_active:
                    dmin_s = min(dmin_s, float(np.linalg.norm(
                        pos - o.pos, axis=1).min()))
                if np.isfinite(dmin_s):
                    dmin_so = min(dmin_so, dmin_s)
                from ttc import ttc_matrix
                T, _, _ = ttc_matrix(pos, view.sheep_vel_global,
                                     view.obs_pos, view.obs_vel,
                                     cfg.r_sheep + cfg.r_obs, horizon=12.0)
                ttc_min = float(np.min(T)) if np.isfinite(T).any() else np.inf
            else:
                rho, s_c, lat_c, cflock, dmin_s = 0.0, 0.0, 0.0, 0.0, np.inf
                ttc_min = np.inf
            cov_dir = float(view.mask_direct.mean()) if len(
                view.mask_direct) else 0.0
            cov_sha = float(view.mask_shared.mean()) if len(
                view.mask_shared) else 0.0
            K = len(obs)
            ttc_each = np.full(K, np.inf)
            if K and n_alive:
                from ttc import ttc_matrix
                p_all = np.array([o.pos if o.is_alive() else o.start
                                  for o in obs], dtype=float)
                v_all = np.array([o.vel for o in obs], dtype=float)
                Tall, _, _ = ttc_matrix(pos, view.sheep_vel_global, p_all,
                                        v_all, cfg.r_sheep + cfg.r_obs,
                                        horizon=12.0)
                for k, o in enumerate(obs):
                    col = Tall[:, k]
                    ttc_each[k] = float(np.min(col)) \
                        if (o.is_alive() and np.isfinite(col).any()) else np.inf
            c_goal = view.path.point_at(cfg.goal_frac * view.path.total)
            if n_alive:
                cnt = pos.mean(axis=0)
                d_goal = float(np.linalg.norm(cnt - c_goal))
                d_range = float(np.max(np.linalg.norm(pos - cnt, axis=1)))
                vbar = view.sheep_vel_global.mean(axis=0)
                gdir = c_goal - cnt
                ng, nv = float(np.linalg.norm(gdir)), float(np.linalg.norm(vbar))
                head_err = (float(np.degrees(np.arccos(np.clip(
                    float(vbar @ gdir) / max(ng * nv, 1e-9), -1.0, 1.0))))
                    if (ng > 1e-9 and nv > 1e-9) else np.nan)
                nguyen_ok = bool(d_goal <= 2.0 and d_range <= 12.0)
            else:
                d_goal = d_range = np.inf
                head_err = np.nan
                nguyen_ok = False
            logs.append(dict(t=t, rho=rho, lat=lat_c, ttc=ttc_min,
                             ttc_each=ttc_each.tolist(),
                             dmin=(dmin_s if np.isfinite(dmin_s) else np.inf),
                             cflock=cflock, cov_direct=cov_dir,
                             cov_shared=cov_sha, lost=lost, tag=tag,
                             d_goal=d_goal, d_range=d_range,
                             head_err=head_err, nguyen_ok=nguyen_ok))

        # ---- termination (kept verbatim) --------------------------------
        if flock.alive.any():
            s_c, _, _ = path.closest(flock.pos[flock.alive].mean(axis=0))
            if s_c >= cfg.goal_frac * path.total:
                success_t = t
                terminated_by = "goal"
                break
        if cfg.terminate_on_collision and lost > 0:
            terminated_by = "collision"
            break
        if trace > 0 and step % max(1, int(trace / dt)) == 0:
            c = flock.pos[flock.alive].mean(axis=0) if flock.alive.any() \
                else np.zeros(2)
            s_c, lat_c, _ = path.closest(c)
            print(f"t={t:6.1f} s={s_c:6.2f} lat={lat_c:6.2f} "
                  f"rho={logs[-1]['rho']:.2f} lost={lost} tag={tag}",
                  flush=True)
        t += dt
        step += 1
    else:
        timeout = True

    ctrl_ms = ctrl_ms_acc / max(ctrl_n, 1)
    res = dict(
        cfg=cfg, path=path, logs=logs, lost=lost, n0=n0,
        success=success_t is not None, success_t=success_t,
        timeout=bool(timeout), terminated_by=terminated_by,
        ctrl_ms=ctrl_ms, controller=type(controller).__name__,
        controller_obj=controller,
        violation_rate=(robot_viol + sheep_viol) /
        max(n_dt * (cfg.n_robots + cfg.n_sheep), 1),
        dmin_robot_obs=dmin_ro, dmin_robot_robot=dmin_rr,
        rr_min_t=rr_info[1], rr_min_tag=rr_info[2],
        rr_min_pair=(rr_info[3], rr_info[4]),
        dmin_sheep_obs=dmin_so,
        control_var=cvar_acc / max(cvar_n, 1),
        out_normal=len(out_normal), out_risk=len(out_risk),
        role_switches=getattr(controller, "role_switch_count", np.nan),
    )

    v2 = dict(config_id=v2cfg.config_id, projection=projection,
              animal_model=animal.name)
    v2.update(counters.to_dict())
    v2.update(animal.summary())

    # ---- Stage-2 controller layer (speed policy / predictor) summary ----
    from v2.controller_v2 import OursRolloutV2, unpack
    core, pol, shadow = unpack(controller)
    if pol is not None:
        v2.update(pol.policy.summary())
    if isinstance(core, OursRolloutV2):
        v2.update(core.summary())
    if shadow is not None:
        v2.update(shadow.summary())
        res["shadow_log"] = shadow.log
    if mon is not None:
        v2.update(mon.summary())
    if feas is not None:
        v2.update(feas.summary())
        if record_events:
            v2["events"] = feas.events[:500]
            v2["contact_C"] = feas.contact_C[:200]
        v2["brake_events_sample"] = feas.brake_events[:50]
    res["v2"] = v2
    res["v2cfg"] = v2cfg
    res["animal_model_obj"] = animal
    if trace_rows is not None:
        res["trace_states"] = trace_rows
    if record_animal:
        res["animal_log"] = animal.log
    return res
