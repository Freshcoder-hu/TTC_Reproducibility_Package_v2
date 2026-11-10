"""Stage 3, step 1: measure the **actually observable** warning time.

The TTC tuple must not be fixed before we know how much warning the sensing
interface can physically deliver.  This module measures, per encounter
geometry and per obstacle approach speed,

    tau_det    first tick at which any robot is within ``R`` of the obstacle
    tau_avail  tau_det + delivery delay (after i.i.d. packet loss)
    tau_corr   first tick at which the obstacle disc touches the corridor
    tau_hit    first tick at which the obstacle disc touches the *reference*
               flock (no-intervention counterfactual)
    T_obs      tau_hit - tau_avail          <- the observable warning time
    T_corr     tau_corr - tau_avail         <- the warning before the corridor

METHOD (and why it is a legitimate counterfactual)
--------------------------------------------------
The robot/flock kinematics come from a **real plant run** of the scenario with
a *passive* controller (B1, pure path shepherding) and **no obstacles**.  That
trajectory is the no-intervention counterfactual the manuscript asks for
(§4.8 last paragraph: "a no-intervention counterfactual replay or an offline
reference rollout can define forecasted contact").  Using a real run keeps the
robot geometry honest (the fleet really does advance at ~1 m/s and the robots
really do sit 2-4 m from the flock), while keeping the measurement independent
of any avoidance the obstacle would have provoked.

The obstacle is then replayed kinematically (constant velocity, the plant's own
obstacle model) through that recorded world.  Detection, corridor entry and
contact are evaluated offline, so the whole range x delay x loss sweep is free
once the reference kinematics are cached.

Fixed encounter construction (documented so it cannot drift):

* encounter point ``E = (35, offset)`` on the S2 straight 70 m path;
* encounter time ``T_enc = 27 s``, which is where the passive flock centroid
  reaches x = 35 m (measured: 34.4-35.6 m over the 20 calibration seeds);
* obstacle released at ``E - u * speed * T_enc``, i.e. it passes ``E`` at
  ``T_enc`` for every speed, so **speed is the only variable** and the initial
  range grows with the speed (3 m/s -> 81 m, 4 -> 108 m, 6 -> 162 m for a
  perpendicular crossing).  This is the honest way to ask "is a 16 s warning
  achievable": a fast obstacle simply starts further away.

Usage:
    python -X utf8 -m v2.observability_v2            # capture + table
    python -X utf8 -m v2.observability_v2 --recapture
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from run_experiments import ALGS  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402

from v2.config_v2 import (SENSE_DELAY_LEVELS, SENSE_ERR_LEVELS,  # noqa: E402
                          SENSE_LOSS_LEVELS, SENSE_RANGE_LEVELS,
                          sim_overrides, stage3_v2cfg)
from v2.sim_v2 import run_scene_v2  # noqa: E402

OUT = ROOT / "results_v2_stage3"
OBS = OUT / "observability"
CAL_SEEDS = list(range(1000, 1020))

E_X = 35.0
T_ENC = 27.0
SPEEDS = (3.0, 4.0, 6.0)

# geometry -> unit approach direction (the obstacle passes E moving along u)
_OBL = np.array([-0.8, -0.45])
_OBL = _OBL / np.linalg.norm(_OBL)
GEOMETRIES = {
    "head_on": dict(u=np.array([-1.0, 0.0]), offset=0.0,
                    label="head-on, path centreline"),
    "cross_0": dict(u=np.array([0.0, -1.0]), offset=0.0,
                    label="perpendicular crossing, centre"),
    "cross_3": dict(u=np.array([0.0, -1.0]), offset=3.6,
                    label="perpendicular crossing, 3.6 m offset (graze)"),
    "cross_6": dict(u=np.array([0.0, -1.0]), offset=6.0,
                    label="perpendicular crossing, 6.0 m offset (corridor edge)"),
    "oblique_3": dict(u=_OBL, offset=3.6,
                      label="oblique (S2 direction), 3.6 m offset"),
}

LOSS_TAG = 20260501          # RNG stream for the packet-loss replay


# ---------------------------------------------------------------------------
def encounter(geom, speed):
    """Obstacle (start, vel) that passes ``E`` at ``T_enc`` along ``u``."""
    g = GEOMETRIES[geom]
    E = np.array([E_X, float(g["offset"])])
    vel = g["u"] * float(speed)
    return E - vel * T_ENC, vel


# ---------------------------------------------------------------------------
def capture_reference(seed, sid="S2", T_max=60.0, v2cfg=None, out=None):
    """Real plant run, passive controller, **no obstacles** (counterfactual)."""
    v2cfg = v2cfg or stage3_v2cfg("M1")
    scn = SCENARIOS[sid]()
    cfg = replace(scn["cfg"], **sim_overrides(v2cfg))
    v2cfg.assert_consistent(cfg)
    ctrl = ALGS["B1"]()
    res = run_scene_v2(cfg, scn["path"], [], ctrl, seed=seed, T_max=T_max,
                       s0=scn["s0"], v2cfg=v2cfg, contact=False,
                       record_states=True)
    tr = res["trace_states"]
    t = np.array([r[0] for r in tr], dtype=float)
    rpos = np.array([r[6] for r in tr], dtype=float)
    spos = np.array([r[3] for r in tr], dtype=float)
    return dict(t=t, rpos=rpos, spos=spos, sid=sid, seed=seed,
                config_id=v2cfg.config_id)


def load_reference(seed, recapture=False):
    OBS.mkdir(parents=True, exist_ok=True)
    p = OBS / f"ref_{seed}.npz"
    if p.exists() and not recapture:
        z = np.load(p)
        return dict(t=z["t"], rpos=z["rpos"], spos=z["spos"], seed=seed)
    ref = capture_reference(seed)
    np.savez_compressed(p, t=ref["t"], rpos=ref["rpos"], spos=ref["spos"])
    return ref


# ---------------------------------------------------------------------------
def _first(t, mask, t_min):
    """First time in ``mask`` at/after ``t_min`` (``nan`` if never)."""
    idx = np.where(mask & (t >= t_min - 1e-9))[0]
    return float(t[idx[0]]) if len(idx) else np.nan


def measure(ref, geom, speed, R=70.0, delay=0.0, loss=0.0, seed=None,
            r_obs=1.2, r_sheep=0.35, halfwidth=6.0, path=None, dt=0.05):
    """One (geometry, speed, interface) cell -> all the times of interest."""
    t = ref["t"]
    start, vel = encounter(geom, speed)
    o = start[None, :] + vel[None, :] * t[:, None]              # (n,2)
    drob = np.linalg.norm(o[:, None, :] - ref["rpos"], axis=2).min(axis=1)
    dso = np.linalg.norm(o[:, None, :] - ref["spos"], axis=2).min(axis=1)
    det = drob <= R
    tau_det = _first(t, det, dt)
    # ---- availability: first *delivered* detection frame + delay ----
    tau_avail = np.nan
    if np.isfinite(tau_det):
        if loss > 0:
            gid = list(GEOMETRIES).index(geom)      # deterministic, not hash()
            rng = np.random.default_rng([int(seed or 0), LOSS_TAG,
                                         int(round(speed * 100)), gid])
            frames = np.where(det)[0]
            hit = None
            for i in frames:
                # 12 robots each dropping independently: one survivor suffices
                if rng.random(12).min() >= loss:
                    hit = float(t[i])
                    break
            tau_avail = (hit if hit is not None else tau_det) + delay
        else:
            tau_avail = tau_det + delay
    # ---- corridor entry: obstacle disc touches |lat| <= W ----
    lat = np.abs(o[:, 1]) if path is None else np.abs(path.closest(o)[1])
    tau_corr = _first(t, lat <= halfwidth + r_obs, dt)
    # ---- counterfactual contact with the reference flock ----
    tau_hit = _first(t, dso <= r_obs + r_sheep, dt)
    out = dict(geometry=geom, speed=float(speed), R=float(R), delay=float(delay),
               loss=float(loss), seed=seed,
               tau_det=tau_det, tau_avail=tau_avail, tau_corr=tau_corr,
               tau_hit=tau_hit,
               T_obs=(tau_hit - tau_avail) if (np.isfinite(tau_hit)
                                               and np.isfinite(tau_avail))
               else np.nan,
               T_corr=(tau_corr - tau_avail) if (np.isfinite(tau_corr)
                                                 and np.isfinite(tau_avail))
               else np.nan,
               start_x=float(start[0]), start_y=float(start[1]),
               start_range=float(np.linalg.norm(start - ref["rpos"][0].mean(
                   axis=0))),
               already_in_corridor=bool(lat[0] <= halfwidth + r_obs))
    return out


# ---------------------------------------------------------------------------
def build_table(refs, R=70.0, delay=0.0, loss=0.0, speeds=SPEEDS,
                geoms=None):
    rows = []
    for geom in (geoms or list(GEOMETRIES)):
        for v in speeds:
            for ref in refs:
                rows.append(measure(ref, geom, v, R=R, delay=delay, loss=loss,
                                    seed=ref["seed"]))
    return rows


def aggregate(rows, key=("geometry", "speed")):
    """Mean/percentile of T_obs over seeds, per (geometry, speed)."""
    cells = {}
    for r in rows:
        k = tuple(r[k] for k in key)
        cells.setdefault(k, []).append(r)
    out = []
    for k, rs in cells.items():
        tob = np.array([r["T_obs"] for r in rs], dtype=float)
        tob = tob[np.isfinite(tob)]
        det = np.array([r["tau_det"] for r in rs], dtype=float)
        cor = np.array([r["tau_corr"] for r in rs], dtype=float)
        hit = np.array([r["tau_hit"] for r in rs], dtype=float)
        out.append(dict(
            **{key[i]: k[i] for i in range(len(key))},
            n=len(rs),
            n_never_detected=int(np.sum(~np.isfinite(det))),
            tau_det_mean=float(np.nanmean(det)) if np.isfinite(det).any() else np.nan,
            tau_corr_mean=float(np.nanmean(cor)) if np.isfinite(cor).any() else np.nan,
            tau_hit_mean=float(np.nanmean(hit)) if np.isfinite(hit).any() else np.nan,
            T_obs_mean=float(tob.mean()) if tob.size else np.nan,
            T_obs_p10=float(np.percentile(tob, 10)) if tob.size else np.nan,
            T_obs_p50=float(np.percentile(tob, 50)) if tob.size else np.nan,
            T_obs_min=float(tob.min()) if tob.size else np.nan,
            T_obs_max=float(tob.max()) if tob.size else np.nan,
            negative=int(np.sum(tob < 0)) if tob.size else 0))
    out.sort(key=lambda r: (r.get("speed", 0), r.get("geometry", "")))
    return out


def write_csv(path, rows):
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


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", nargs="*", type=int, default=CAL_SEEDS)
    ap.add_argument("--recapture", action="store_true")
    args = ap.parse_args()
    OBS.mkdir(parents=True, exist_ok=True)
    refs = [load_reference(s, recapture=args.recapture) for s in args.seeds]
    # -- where is the flock at T_enc?  (documents the encounter construction) --
    x_enc = [float(r["spos"][int(np.argmin(np.abs(r["t"] - T_ENC)))]
                   [:, 0].mean()) for r in refs]
    print(f"reference kinematics: {len(refs)} seeds, "
          f"{len(refs[0]['t'])} ticks, T_max={refs[0]['t'][-1]:.1f} s")
    print(f"flock centroid x at T_enc={T_ENC:g} s: "
          f"{np.nanmean(x_enc):.2f} +- {np.nanstd(x_enc):.2f} m "
          f"(min {np.nanmin(x_enc):.2f}, max {np.nanmax(x_enc):.2f}); "
          f"encounter point x = {E_X:g} m")

    # ---- primary table: R=70, exact quality ----
    rows = build_table(refs, R=70.0)
    write_csv(OBS / "observable_cells_R70.csv", rows)
    agg = aggregate(rows)
    write_csv(OBS / "observable_table_R70.csv", agg)
    print("\n=== observable warning time, R = 70 m, exact/zero-delay ----")
    hdr = ("geometry", "v", "tau_det", "tau_corr", "tau_hit", "T_obs mean",
           "p10", "min", "neg")
    print(("{:<10}{:>5}{:>9}{:>9}{:>9}{:>11}{:>8}{:>8}{:>5}").format(*hdr))
    for r in agg:
        print("{:<10}{:>5.0f}{:>9.2f}{:>9.2f}{:>9.2f}{:>11.2f}{:>8.2f}"
              "{:>8.2f}{:>5d}".format(
                  r["geometry"], r["speed"], r["tau_det_mean"],
                  r["tau_corr_mean"], r["tau_hit_mean"], r["T_obs_mean"],
                  r["T_obs_p10"], r["T_obs_min"], r["negative"]))

    # ---- primary distribution over the design envelope (3 and 4 m/s) ----
    env = [r["T_obs"] for r in rows
           if r["speed"] in (3.0, 4.0) and np.isfinite(r["T_obs"])]
    stress = [r["T_obs"] for r in rows
              if r["speed"] == 6.0 and np.isfinite(r["T_obs"])]
    allv = [r["T_obs"] for r in rows if np.isfinite(r["T_obs"])]
    def pct(a, q):
        return float(np.percentile(a, q)) if a else float("nan")
    summary = dict(
        R=70.0, n_cells=len(rows), n_seeds=len(refs),
        design_envelope=dict(speeds=[3.0, 4.0], n=len(env),
                             mean=float(np.mean(env)) if env else None,
                             p05=pct(env, 5), p10=pct(env, 10),
                             p25=pct(env, 25), p50=pct(env, 50),
                             min=float(np.min(env)) if env else None),
        stress_6ms=dict(n=len(stress),
                        mean=float(np.mean(stress)) if stress else None,
                        p10=pct(stress, 10), p50=pct(stress, 50),
                        min=float(np.min(stress)) if stress else None),
        all_speeds=dict(n=len(allv), p10=pct(allv, 10), p50=pct(allv, 50),
                        min=float(np.min(allv)) if allv else None))
    print("\n  design envelope (3-4 m/s, all geometries): "
          f"p10={summary['design_envelope']['p10']:.2f} s, "
          f"p50={summary['design_envelope']['p50']:.2f} s, "
          f"min={summary['design_envelope']['min']:.2f} s")
    print(f"  6 m/s stress: p10={summary['stress_6ms']['p10']:.2f} s, "
          f"min={summary['stress_6ms']['min']:.2f} s")
    print(f"  all speeds: p10={summary['all_speeds']['p10']:.2f} s")

    # ---- range / delay / loss sweep (offline replay) ----
    sweep = []
    for R in SENSE_RANGE_LEVELS:
        for d in SENSE_DELAY_LEVELS:
            for lo in SENSE_LOSS_LEVELS:
                rr = build_table(refs, R=R, delay=d, loss=lo)
                a = aggregate(rr)
                tob = [x["T_obs"] for x in rr if np.isfinite(x["T_obs"])]
                env2 = [x["T_obs"] for x in rr
                        if x["speed"] in (3.0, 4.0) and np.isfinite(x["T_obs"])]
                st2 = [x["T_obs"] for x in rr
                       if x["speed"] == 6.0 and np.isfinite(x["T_obs"])]
                sweep.append(dict(
                    R=R, delay=d, loss=lo, n=len(tob),
                    n_never=int(np.sum(~np.isfinite(
                        [x["tau_det"] for x in rr]))),
                    T_obs_p10=pct(env2, 10), T_obs_p50=pct(env2, 50),
                    T_obs_min=float(np.min(env2)) if env2 else np.nan,
                    T_obs_p10_all=pct(tob, 10),
                    T_obs_p50_all=pct(tob, 50),
                    T_obs_min_all=float(np.min(tob)) if tob else np.nan,
                    stress_p10=pct(st2, 10), stress_min=(
                        float(np.min(st2)) if st2 else np.nan)))
                write_csv(OBS / f"observable_cells_R{int(R)}_d{d}_l{lo}.csv", rr)
    write_csv(OBS / "observable_sweep.csv", sweep)
    print("\n=== sensitivity: R x delay x loss (T_obs p10 over 3-4 m/s) ===")
    print(f"{'R':>6}{'delay':>7}{'loss':>7}{'p10':>8}{'p50':>8}"
          f"{'min':>8}{'6m/s p10':>10}")
    for s in sweep:
        print(f"{s['R']:>6.0f}{s['delay']:>7.2f}{s['loss']:>7.2f}"
              f"{s['T_obs_p10']:>8.2f}{s['T_obs_p50']:>8.2f}"
              f"{s['T_obs_min']:>8.2f}{s['stress_p10']:>10.2f}")

    (OUT / "observability_summary.json").write_text(
        json.dumps(dict(summary=summary, sweep=sweep,
                        geometries={k: dict(offset=v["offset"],
                                            u=v["u"].tolist(),
                                            label=v["label"])
                                    for k, v in GEOMETRIES.items()},
                        speeds=list(SPEEDS), E_X=E_X, T_ENC=T_ENC,
                        seeds=list(args.seeds),
                        flock_x_at_Tenc=dict(mean=float(np.nanmean(x_enc)),
                                             sd=float(np.nanstd(x_enc)),
                                             min=float(np.nanmin(x_enc)),
                                             max=float(np.nanmax(x_enc)))),
                       indent=2), encoding="utf-8")
    print(f"\nwrote {OBS}")


if __name__ == "__main__":
    main()
