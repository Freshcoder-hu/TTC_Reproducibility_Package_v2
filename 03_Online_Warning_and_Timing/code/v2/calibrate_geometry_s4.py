"""Verify the Stage-4 scene geometry (Table-14 family) before the pilot.

For every Stage-4 scene this script measures, on the **no-intervention
counterfactual** (B1, passive, the obstacles replayed kinematically through the
recorded world — the Stage-3 ``observability_v2`` method):

1. **initial range** — distance from the nearest robot to the obstacle at its
   spawn tick.  The design intent is that this is >= 2R (i.e. the obstacle is
   initially *outside* the 70 m detection range); the V1 geometry failed this
   (53.1 m at release), which is why 40/70/100 m were bit-identical there.
2. **where the crossing happens** — the flock centroid's arc length at the
   counterfactual contact, which must be strictly inside the active route
   (``s0 < s_c < goal_frac * L``), not behind the start or past the goal.
3. **first-detection time, corridor-entry time, contact time and the observable
   warning time** ``T_obs = tau_hit - tau_det`` (exact quality, zero delay).
4. **TTC at first detection** — the analytic TTC of the nearest animal to the
   obstacle on the tick the fleet can first see it.  This is the "intended
   initial TTC" column of the scenario table.
5. **hazard calibration** — the *unmitigated* contact fraction (B1, obstacles
   present, no avoidance logic) and the *mitigated* result (full V2 method), so
   the "graze" offset can be shown to be a real hazard that is nevertheless
   avoidable.  A scene where nobody collides is uninformative; a scene where
   nobody can avoid is saturated.

    python -X utf8 -m v2.calibrate_geometry_s4 --jobs 4

Writes ``results_v2_stage4/geometry/geometry_calibration.{csv,json}``.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from run_experiments import ALGS  # noqa: E402
from ttc import ttc_matrix  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import run_cell  # noqa: E402
from v2.scenarios_v2 import (GOAL_FRAC, R_OBS, R_SENSE, S0, W_V2,  # noqa: E402
                             cfg_for, resolve_scene)
from v2.sim_v2 import run_scene_v2  # noqa: E402

OUT = ROOT / "results_v2_stage4" / "geometry"
TAG = "s4_cal"
DT = 0.05


# ---------------------------------------------------------------------------
def reference_run(seed, scn, v2cfg, T_max=None):
    """Passive B1 run with the obstacles REMOVED, full state recorded."""
    from v2.runner_s4 import scale_cfg
    v2cfg = scale_cfg(scn, v2cfg)          # fixed-width S4: team size is a scene
    cfg = cfg_for(scn, v2cfg)              # property, so mirror it first
    res = run_scene_v2(cfg, scn["path"], [], ALGS["B1"](), seed=seed,
                       T_max=float(T_max or scn["T_max"]), s0=scn["s0"],
                       v2cfg=v2cfg, contact=False, record_states=True)
    tr = res["trace_states"]
    t = np.array([r[0] for r in tr], dtype=float)
    rpos = np.array([r[6] for r in tr], dtype=float)
    spos = np.array([r[3] for r in tr], dtype=float)
    shead = np.array([r[4] for r in tr], dtype=float)
    sspd = np.array([r[5] for r in tr], dtype=float)
    # the V1 log records one row per *control* tick (0.1 s); the reference here
    # is per integration tick, so the centroid for the arc-length checks is
    # recomputed directly from the animal positions
    s_c = spos.mean(axis=1)
    return dict(t=t, rpos=rpos, spos=spos, shead=shead, sspd=sspd,
                centroid=s_c, path_total=float(res["path"].total))


def measure_obstacle(ref, ob, r_sheep=0.35, halfwidth=W_V2, R=R_SENSE,
                     delay=0.0):
    """Times of interest for one obstacle of the scene (kinematic replay)."""
    t = ref["t"]
    start = np.asarray(ob.start, dtype=float)
    vel = np.asarray(ob.vel, dtype=float)
    t_spawn = float(ob.t_spawn)
    life = float(ob.life) if ob.life is not None else np.inf
    dt = t[1] - t[0] if len(t) > 1 else DT
    age = t - t_spawn
    on = (age >= -1e-9) & (age < life)
    opos = np.full((len(t), 2), np.nan)
    opos[on] = start[None, :] + vel[None, :] * age[on, None]
    drob = np.linalg.norm(opos[:, None, :] - ref["rpos"], axis=2).min(axis=1)
    dso = np.linalg.norm(opos[:, None, :] - ref["spos"], axis=2).min(axis=1)
    lat = np.abs(opos[:, 1])

    def first(mask, after=-np.inf):
        idx = np.where(mask & (t >= after - 1e-9))[0]
        return (float(t[idx[0]]), int(idx[0])) if len(idx) else (np.nan, -1)

    tau_det, i_det = first(np.isfinite(drob) & (drob <= R))
    tau_corr, _ = first(np.isfinite(lat) & (lat <= halfwidth + R_OBS))
    tau_hit, i_hit = first(np.isfinite(dso) & (dso <= R_OBS + r_sheep))
    # first tick at which the obstacle exists (age >= 0): argmin(|t-t_spawn|)
    # can land one integration tick *before* the spawn, where the obstacle does
    # not exist yet and its distance is undefined
    i_spawn = int(np.argmax(on)) if on.any() else 0
    init_range = float(drob[i_spawn]) if np.isfinite(drob[i_spawn]) else np.nan
    # TTC at first detection (analytic, truth geometry of that tick)
    ttc_det = np.nan
    if i_det >= 0:
        sp = ref["spos"][i_det]
        sv = (ref["sspd"][i_det][:, None]
              * np.stack([np.cos(ref["shead"][i_det]),
                          np.sin(ref["shead"][i_det])], axis=1))
        T, _, _ = ttc_matrix(sp, sv, opos[i_det][None, :], vel[None, :],
                             R_OBS + r_sheep, horizon=60.0)
        ttc_det = float(np.min(T)) if np.isfinite(T).any() else np.inf
    s_hit = float(ref["centroid"][i_hit, 0]) if i_hit >= 0 else np.nan
    return dict(
        t_spawn=t_spawn, init_range=init_range,
        outside_sense=bool(np.isfinite(init_range) and init_range > R),
        tau_det=tau_det, tau_corr=tau_corr, tau_hit=tau_hit,
        T_obs=(tau_hit - tau_det - delay) if np.isfinite(tau_hit) and
        np.isfinite(tau_det) else np.nan,
        ttc_at_detect=ttc_det,
        s_centroid_at_detect=(float(ref["centroid"][i_det, 0])
                              if i_det >= 0 else np.nan),
        s_centroid_at_hit=s_hit,
        crossing_inside_route=bool(np.isfinite(s_hit) and S0 < s_hit
                                   < GOAL_FRAC * float(ref["path_total"])),
        start_x=float(start[0]), start_y=float(start[1]))


# ---------------------------------------------------------------------------
def scene_calibration(sid, seeds=(1000, 1001, 1002), v2cfg=None):
    v2cfg = v2cfg or stage3_v2cfg("M1")
    scn = resolve_scene(sid)
    rows = []
    L = float(scn["path"].total)
    for seed in seeds:
        ref = reference_run(seed, scn, v2cfg)
        for k, ob in enumerate(scn["obstacles"], start=1):
            m = measure_obstacle(ref, ob)
            m.update(scenario=sid, seed=seed, obstacle=k,
                     path_len=L, goal_s=GOAL_FRAC * L,
                     crossing_inside_route=bool(
                         np.isfinite(m["s_centroid_at_hit"])
                         and S0 < m["s_centroid_at_hit"] < GOAL_FRAC * L))
            rows.append(m)
    return rows


def hazard(sid, seed=1000, algs=("B1", "Ours_rollout"), v2cfg=None):
    """Unmitigated (B1) and mitigated (full method) outcome of the scene."""
    v2cfg = v2cfg or stage3_v2cfg("M1")
    out = []
    for alg in algs:
        row, res = run_cell(sid, alg, seed, v2cfg=v2cfg, return_res=True)
        out.append(dict(scenario=sid, seed=seed, alg=alg,
                        collision_rate=row.get("collision_rate"),
                        min_dist=row.get("min_dist"),
                        min_clearance_so=res["v2"].get(
                            "min_surface_clearance_so"),
                        n_contact_so=res["v2"].get("n_contact_events_so"),
                        rho=row.get("rho_avg"), success=row.get("success"),
                        ended=res["terminated_by"],
                        first_detect_1=res["v2"].get("sense_first_detect_1"),
                        t_pre_on=row.get("t_pre_on"),
                        t_divert_on=row.get("t_divert_on")))
    return out


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--seeds", nargs="+", type=int, default=[1000, 1001, 1002])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--skip-hazard", action="store_true")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    from v2.scenarios_v2 import SCENARIOS_V2
    sids = args.scenarios or list(SCENARIOS_V2)
    v2cfg = stage3_v2cfg("M1")

    # --- 1. reference replay per (scene, seed) -------------------------
    tasks = [(sid, seed) for sid in sids for seed in args.seeds]
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            chunks = list(ex.map(_cal_entry, tasks))
    else:
        chunks = [_cal_entry(t) for t in tasks]
    rows = [r for c in chunks for r in c]
    _write_csv(OUT / "geometry_calibration.csv", rows)
    print(f"{'scenario':9s}{'obs':>4s}{'seed':>6s}{'v':>5s}{'T_enc':>7s}"
          f"{'t_spawn':>8s}{'init_rng':>9s}{'outside':>8s}{'tau_det':>8s}"
          f"{'tau_hit':>8s}{'T_obs':>7s}{'ttc_det':>8s}{'s_hit':>7s}"
          f"{'inside':>7s}")
    for r in rows:
        print(f"{r['scenario']:9s}{r['obstacle']:>4d}{r['seed']:>6d}"
              f"{r['speed']:>5.1f}{r['t_enc']:>7.1f}{r['t_spawn']:>8.2f}"
              f"{r['init_range']:>9.1f}{str(r['outside_sense']):>8s}"
              f"{r['tau_det']:>8.2f}{r['tau_hit']:>8.2f}"
              f"{r['T_obs']:>7.2f}{r['ttc_at_detect']:>8.2f}"
              f"{r['s_centroid_at_hit']:>7.1f}"
              f"{str(r['crossing_inside_route']):>7s}")

    # --- 2. hazard calibration (unmitigated vs mitigated) --------------
    haz = []
    if not args.skip_hazard:
        ht = [(sid, 1000) for sid in sids]
        if args.jobs > 1:
            from concurrent.futures import ProcessPoolExecutor
            with ProcessPoolExecutor(max_workers=args.jobs) as ex:
                hc = list(ex.map(_haz_entry, ht))
        else:
            hc = [_haz_entry(t) for t in ht]
        haz = [r for c in hc for r in c]
        _write_csv(OUT / "geometry_hazard.csv", haz)
        print(f"\n{'scenario':9s}{'alg':14s}{'C':>7s}{'min_dist':>9s}"
              f"{'clear_so':>9s}{'n_so':>5s}{'rho':>7s}{'ended':>10s}")
        for r in haz:
            print(f"{r['scenario']:9s}{r['alg']:14s}"
                  f"{(r['collision_rate'] or 0):>7.3f}"
                  f"{(r['min_dist'] if r['min_dist'] == r['min_dist'] else np.nan):>9.2f}"
                  f"{(r['min_clearance_so'] or np.nan):>9.2f}"
                  f"{int(r['n_contact_so'] or 0):>5d}"
                  f"{(r['rho'] or 0):>7.3f}{str(r['ended']):>10s}")
    (OUT / "geometry_calibration.json").write_text(json.dumps(
        dict(replay=rows, hazard=haz, seeds=args.seeds,
             R_sense=R_SENSE, halfwidth=W_V2, s0=S0,
             goal_s=GOAL_FRAC * 140.0), indent=2, default=float),
        encoding="utf-8")
    print(f"\nwrote {OUT / 'geometry_calibration.csv'}")


def _cal_entry(task):
    sid, seed = task
    v2cfg = stage3_v2cfg("M1")
    scn = resolve_scene(sid)
    ref = reference_run(seed, scn, v2cfg)
    L = float(scn["path"].total)
    out = []
    for k, (ob, rec) in enumerate(zip(scn["obstacles"],
                                      scn.get("design") or []), start=1):
        m = measure_obstacle(ref, ob)
        m.update(scenario=sid, seed=seed, obstacle=k, path_len=L,
                 goal_s=GOAL_FRAC * L, speed=rec["speed"], t_enc=rec["t_enc"],
                 lateral=rec["lateral"], angle_deg=rec["angle_deg"],
                 start_x=rec["start"][0], start_y=rec["start"][1],
                 travel=rec["travel"], rule_ok=rec["rule_ok"],
                 crossing_inside_route=bool(
                     np.isfinite(m["s_centroid_at_hit"])
                     and S0 < m["s_centroid_at_hit"] < GOAL_FRAC * L))
        out.append(m)
    return out


def _haz_entry(task):
    sid, seed = task
    return hazard(sid, seed=seed)


def _write_csv(path, rows):
    if not rows:
        return
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


if __name__ == "__main__":
    main()
