"""Controlled single-obstacle corridor: the Stage-2 behaviour test bed.

One obstacle crosses a straight 12 m-wide corridor; **there are no robots**
(``n_robots = 0``), so what is measured is the animal model alone -- no herding
controller, no robot repulsion, no robot sharing of obstacle tracks.  That is
exactly the acceptance-test-B requirement ("a controlled case -- one obstacle
crossing the corridor, no robots").

Geometry (metres, path along +x, arc length s = x):

    corridor      straight, s in [0, 70], half-width W = 6.0
    flock         36 animals on a 6x6 grid (pitch 1.55 x 1.10 m) at s0 = 16
    obstacle      r_o = 1.2, start (20, -8), velocity (0, +1.0)

The obstacle reaches y = 0 (the corridor centre) at t = 8 s and its surface
crosses the corridor between t ~ 1.5 s and t ~ 14.5 s, i.e. while the flock is
advancing through x = 16-25 m.  The encounter therefore happens inside the run
whatever the animal speed (1.0-2.0 m/s), so the timing does not have to be
re-tuned per model -- which matters when M2 changes the speed.

Everything is deterministic given ``seed`` (the flock initial condition is the
only stochastic input).
"""
from __future__ import annotations

import numpy as np

import corrections

corrections.install(environment=True, contact=True)

from config import default_cfg                       # noqa: E402
from obstacle import Obstacle                        # noqa: E402
from path import straight_path                       # noqa: E402
from sheep import SheepState                         # noqa: E402

from v2.metrics_v2 import summarize_v2               # noqa: E402
from v2.sim_v2 import run_scene_v2                   # noqa: E402

T_MAX = 22.0


class NullController:
    """No robots: the command is empty and nothing is decided here."""

    name = "NullController"
    switch_count = 0
    role_switch_count = 0

    def step(self, view, cfg):
        return np.zeros((len(view.robot_pos), 2)), "normal"


def grid_flock(seed, cfg, path, s0, nx=6, ny=6, dx=1.55, dy=1.10,
               jitter=0.10, head_jitter=0.10, v0=1.0):
    """Coherent formation start for the no-robot corridor.

    ``sheep.init_flock`` packs 36 animals into a 2.8 m disc.  With no robots
    present, soft repulsion then dominates the Couzin branch and the group
    diffuses in a random direction instead of walking the corridor -- measured:
    the centroid drifted to (5, -3) after 10 s and the flock never met the
    obstacle.  A 6x6 grid (1.55 m x 1.10 m pitch, both above ``r_rep`` = 1.1 m)
    advances along the path as a coherent group, which is what a controlled
    encounter needs.  This is a test-bed initial condition only: every protocol
    run keeps ``init_flock`` and the frozen V1/V2 numbers are untouched.
    The seeded jitter provides independent-but-comparable realisations.
    """
    rng = np.random.default_rng([int(seed), 7])
    gx, gy = np.meshgrid(np.arange(nx) - (nx - 1) / 2.0,
                         np.arange(ny) - (ny - 1) / 2.0)
    pos = np.stack([gx.ravel() * dx, gy.ravel() * dy], axis=1)
    pos += rng.uniform(-jitter, jitter, pos.shape)
    pos[:, 0] += path.point_at(s0)[0]
    pos[:, 1] += path.point_at(s0)[1]
    head = rng.uniform(-head_jitter, head_jitter, len(pos))
    return SheepState(pos, head, np.full(len(pos), float(v0)))


def corridor_scene(seed=1000, r_obs=1.2, x_cross=20.0, y0=-8.0, vy=1.0,
                   s0=16.0, T_max=T_MAX, n_sheep=36, halfwidth=6.0,
                   length=70.0):
    cfg = default_cfg(n_sheep=n_sheep, n_robots=0)
    cfg.r_obs = float(r_obs)
    path = straight_path(0.0, float(length), halfwidth=float(halfwidth))
    obs = [Obstacle([float(x_cross), float(y0)], [0.0, float(vy)],
                    radius=float(r_obs), life=float(T_max) + 10.0,
                    t_spawn=0.0, a_max=0.0)]
    return dict(name="C1_corridor_crossing", path=path, obstacles=obs,
                cfg=cfg, T_max=float(T_max), s0=float(s0))


def run_corridor(seed, v2cfg, record_animal=True, scene=None):
    """One corridor run.  Returns (metrics dict, res)."""
    scn = scene or corridor_scene(seed=seed)
    cfg = scn["cfg"]
    res = run_scene_v2(cfg, scn["path"], scn["obstacles"], NullController(),
                       seed=seed, T_max=scn["T_max"], s0=scn["s0"],
                       v2cfg=v2cfg, contact=True, feasibility=False,
                       record_states=True, record_animal=record_animal,
                       flock_init=grid_flock)
    m = summarize_v2(res, v2cfg)
    m.update(corridor_metrics(res, v2cfg, cfg))
    return m, res


# ---------------------------------------------------------------------------
# behaviour metrics computed from the recorded trajectory
# ---------------------------------------------------------------------------
def corridor_metrics(res, v2cfg, cfg, lat_tol=0.0):
    tr = res["trace_states"]
    T = len(tr)
    spos = np.array([r[3] for r in tr])          # (T,N,2)
    shead = np.array([r[4] for r in tr])         # (T,N)
    sspd = np.array([r[5] for r in tr])          # (T,N)
    opos = np.array([r[10] for r in tr])         # (T,K,2)
    tarr = np.array([r[0] for r in tr])
    dt = cfg.dt
    n = spos.shape[1]
    r_sum = cfg.r_sheep + cfg.r_obs

    # ---- surface gap per animal per tick (min over obstacles) ----
    d = np.linalg.norm(spos[:, :, None, :] - opos[:, None, :, :], axis=3)
    gap = d - r_sum                                  # (T,N,K)
    gap_min = gap.min(axis=2)                        # (T,N)
    per_animal_min = gap_min.min(axis=0)             # (N,)
    contacted = per_animal_min <= 0.0

    # sampled contact onsets (cross-check for the swept detector)
    inside = gap_min <= 0.0
    onsets = np.zeros_like(inside)
    onsets[0] = inside[0]
    onsets[1:] = inside[1:] & ~inside[:-1]
    n_episodes_sampled = int(onsets.sum())

    # ---- heading rate and speed distributions ----
    dth = np.angle(np.exp(1j * np.diff(shead, axis=0)))
    omega = np.abs(dth) / dt                          # (T-1,N)
    spd = sspd
    # ---- oscillation: sign flips of the heading increment ----
    sgn = np.sign(dth)
    flips = np.zeros_like(sgn)
    flips[1:] = (sgn[1:] * sgn[:-1]) < 0
    dur = max(T * dt, 1e-9)
    flip_rate = flips.sum(axis=0) / dur                # per animal per second
    # ---- path efficiency: net displacement / travelled distance ----
    step_len = np.linalg.norm(np.diff(spos, axis=0), axis=2)
    travelled = step_len.sum(axis=0)
    net = np.linalg.norm(spos[-1] - spos[0], axis=1)
    eff = net / np.maximum(travelled, 1e-9)
    # group extent (a blow-up would show here, dispersal alone would not)
    ctr = spos.mean(axis=1)
    rad95 = np.percentile(np.linalg.norm(spos - ctr[:, None, :], axis=2), 95,
                          axis=1)
    finite = bool(np.isfinite(spos).all() and np.isfinite(sspd).all()
                  and np.isfinite(shead).all())
    n_far = int(np.sum(np.linalg.norm(spos[-1] - spos[0].mean(axis=0),
                                      axis=1) > 40.0))

    # ---- corridor keeping (true boundary and inner control boundary) ----
    half = scn_hw(res)
    s_c = spos[:, :, 0]                               # straight path, s = x
    inside_frac = float(np.mean(np.abs(spos[:, :, 1]) <= half))
    max_abs_lat = float(np.max(np.abs(spos[:, :, 1])))

    out = dict(
        n_ticks=T,
        # contact
        animal_contact_fraction=float(contacted.mean()),
        n_animals_contacted=int(contacted.sum()),
        n_contact_episodes_sampled=n_episodes_sampled,
        mean_min_clearance=float(per_animal_min.mean()),
        worst_min_clearance=float(per_animal_min.min()),
        p10_min_clearance=float(np.percentile(per_animal_min, 10)),
        # response
        onset_latency=onset_latency(res, cfg),
        # speed
        speed_mean=float(spd.mean()), speed_p95=float(np.percentile(spd, 95)),
        speed_max=float(spd.max()),
        speed_min=float(spd.min()),
        frac_stopped=float(np.mean(spd < 0.05)),
        frac_speed_off_band=float(np.mean(
            (np.abs(spd - 1.0) > 1e-6) & (np.abs(spd - 1.5) > 1e-6))),
        # heading
        omega_mean=float(omega.mean()), omega_p95=float(np.percentile(omega, 95)),
        omega_max=float(omega.max()),
        frac_omega_at_cap=float(np.mean(omega >= 0.999 * cfg.turn_max)),
        # oscillation / explosion
        flip_rate_mean=float(flip_rate.mean()),
        flip_rate_p95=float(np.percentile(flip_rate, 95)),
        path_efficiency_mean=float(eff.mean()),
        # corridor
        inside_frac=inside_frac,
        max_abs_lateral=max_abs_lat,
        radius95_max=float(np.max(rad95)),
        finite_state=finite,
        n_exploded=n_far,
    )
    for k, v in res["v2"].items():
        if k.startswith(("stim_", "sep_guard", "body_priority", "n_sensed",
                         "gain_", "delay_", "obs_delay", "n_animal_steps")):
            out.setdefault(k, v)
    out["config_id"] = v2cfg.config_id
    out["animal_model"] = v2cfg.animal_model
    out["m1_sense_range"] = v2cfg.m1_sense_range
    out["m1_gain"] = v2cfg.m1_gain
    out["m1_decay"] = v2cfg.m1_decay
    out["m1_obs_delay"] = v2cfg.m1_obs_delay
    out["m2_gain_sigma"] = v2cfg.m2_gain_sigma
    out["m2_react_delay"] = v2cfg.m2_react_delay
    out["m2_slow_gain"] = v2cfg.m2_slow_gain
    out["m_v_max"] = v2cfg.m_v_max
    return out


def scn_hw(res):
    """Corridor half-width as the path actually reports it."""
    return float(res["path"].halfwidth_at(0.0))


def onset_latency(res, cfg, frac=0.2, per_animal=False):
    """Per-animal avoidance-onset latency [s] (mean, or the full array).

    Two fixed, model-independent instants:

    ``t_enter``  first tick at which the obstacle's **surface gap** is inside
                 the animal's sensing range -- measured on the *geometric*
                 (zero-delay) gap, so it is the same instant whatever the
                 model's delays are;
    ``t_react``  first tick at which the *received* stimulus reaches
                 ``frac * k_M1`` -- an **absolute** level, so the measure does
                 not depend on how close the animal eventually gets.

    The latency is ``t_react - t_enter``; it contains the observation delay,
    the M2 reaction delay and the ramp of the distance weight.  Returns NaN
    for animals that never reach the threshold (M0: all of them).
    """
    log = res.get("animal_log") or []
    if not log:
        return np.zeros(0) if per_animal else float("nan")
    v2cfg = res["v2cfg"]
    # frac = 0 means "first tick with a nonzero obstacle term", i.e. the first
    # response of any size; a tiny floor keeps the comparison well defined.
    thr = max(float(frac) * float(v2cfg.m1_gain), 1e-12)
    gaps = np.array([e["gap"] for e in log])        # (T,N)
    stim = np.array([e["stim"] for e in log])       # (T,N)
    tarr = np.array([e["t"] for e in log])
    R = v2cfg.m1_sense_range
    n = gaps.shape[1]
    lat = np.full(n, np.nan)
    for i in range(n):
        gi, si = gaps[:, i], stim[:, i]
        enter = np.where(gi < R)[0]
        if not len(enter):
            continue
        react = np.where((si >= thr)
                         & (np.arange(len(si)) >= enter[0]))[0]
        if not len(react):
            continue
        lat[i] = tarr[react[0]] - tarr[enter[0]]
    if per_animal:
        return lat
    return float(np.nanmean(lat)) if np.isfinite(lat).any() else float("nan")
