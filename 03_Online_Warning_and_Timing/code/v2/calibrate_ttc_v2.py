"""Stage 3, step 2: derive, check and **freeze** the TTC tuple.

The tuple is not chosen, it is mapped from the measured observable-time
distribution (``v2/observability_v2.py``) with four pre-registered rules:

R1  T_PRE    = floor_0.5( p10(T_obs | design envelope) - GUARD_PRE )
      design envelope = obstacle speeds 3 and 4 m/s, all five encounter
      geometries, R = 70 m, exact/zero-delay quality (the manuscript's initial
      configuration; 6 m/s is the *stress* level and is reported separately).
      GUARD_PRE = 1.0 s = ten control periods = one full PRE/DIVERT decision
      cycle, so the warning band starts strictly inside what the interface can
      deliver on its worst decile.
R2  T_DIVERT = floor_1.0( T_PRE - B_DIV ),
      B_DIV = t_react + t_lat + GUARD_DIV,
      t_react = m1_obs_delay + one control period  (the animal's observation
                delay plus one decision period),
      t_lat   = 2*sqrt(divert_shift / a_lat)       (bang-bang lateral shift:
                accelerate, then decelerate, at the author's a_lat),
      GUARD_DIV = 1.0 s.
      Requirement: T_DIVERT must be <= min(T_obs) over *all* cells, otherwise
      DIVERT can be entered after the obstacle has already arrived.
R3  T_REC    = T_PRE + max( h, ceil_0.5(median RECOVER duration) ).  The
      clearance threshold must sit at least one hysteresis band above the
      warning band (otherwise the PRE exit gate would overlap the RECOVER exit
      gate on the return path) and at least as long as the recovery itself.
      The RECOVER duration is *measured* in the pilot runs (RECOVER entry to
      the first tick that satisfies the restoration condition rho >= recover_rho
      and |lat| <= recover_lat, else the whole episode).
R4  h        = 3.0 s, the manuscript's V2 value, required to satisfy
      T_DIVERT + h < T_PRE (otherwise the DIVERT exit gate would sit above the
      PRE entry gate and the band would be empty).

The script also answers the acceptance question "what fails, and what range
would it need": for every (geometry, speed) it solves for the smallest
robot->obstacle range at which the cell's 10th percentile observable time
reaches 12 s (the frozen T_PRE) and 16 s (the manuscript's proposal).

    python -X utf8 -m v2.calibrate_ttc_v2
"""
from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from scenarios import SCENARIOS  # noqa: E402

from v2.config_v2 import STAGE2_TTC, STAGE3_TTC, stage3_v2cfg  # noqa: E402
from v2.observability_v2 import (GEOMETRIES, SPEEDS, aggregate,  # noqa: E402
                                 build_table, load_reference, measure)
from v2.runner_v2 import run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage3"
OBS = OUT / "observability"
CAL_SEEDS = list(range(1000, 1020))

GUARD_PRE = 1.0
GUARD_DIV = 1.0


def floor_to(x, grid):
    return math.floor(float(x) / grid + 1e-9) * grid


def ceil_to(x, grid):
    return math.ceil(float(x) / grid - 1e-9) * grid


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
# measured RECOVER restoration time
# ---------------------------------------------------------------------------
def recover_restoration_times(sid, seed, v2cfg):
    """RECOVER entry -> first tick satisfying the restoration condition."""
    scn = SCENARIOS[sid]()
    row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=v2cfg,
                        return_res=True)
    logs = res["logs"]
    tags = [l["tag"] for l in logs]
    out = []
    i = 0
    n = len(logs)
    while i < n:
        if tags[i] != "recover":
            i += 1
            continue
        j = i
        while j < n and tags[j] == "recover":
            j += 1
        t0 = float(logs[i]["t"])
        rest = np.nan
        for k in range(i, j):
            lg = logs[k]
            if lg["rho"] >= 0.88 and abs(float(lg["lat"])) <= 3.0:
                rest = float(lg["t"]) - t0
                break
        out.append(dict(scenario=sid, seed=seed, t_enter=t0,
                        dur_recover=float(logs[j - 1]["t"]) - t0 + 0.1,
                        restore=rest, ended=tags[j] if j < n else "end"))
        i = j
    return out, row


# ---------------------------------------------------------------------------
# required range for a target warning time
# ---------------------------------------------------------------------------
def cell_p10(refs, geom, speed, R, pct=10):
    vals = []
    for ref in refs:
        r = measure(ref, geom, speed, R=R)
        if np.isfinite(r["T_obs"]):
            vals.append(r["T_obs"])
    return float(np.percentile(vals, pct)) if vals else np.nan


def required_range(refs, geom, speed, target, lo=20.0, hi=200.0):
    """Smallest R on a 2 m grid with cell p10(T_obs) >= target (nan if none)."""
    if cell_p10(refs, geom, speed, hi) < target:
        return np.nan
    a, b = lo, hi
    while b - a > 2.0:
        mid = 0.5 * (a + b)
        if cell_p10(refs, geom, speed, mid) >= target:
            b = mid
        else:
            a = mid
    return float(b)


# ---------------------------------------------------------------------------
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    refs = [load_reference(s) for s in CAL_SEEDS]
    rows70 = build_table(refs, R=70.0)
    agg70 = aggregate(rows70)
    env = [r["T_obs"] for r in rows70
           if r["speed"] in (3.0, 4.0) and np.isfinite(r["T_obs"])]
    stress = [r["T_obs"] for r in rows70
              if r["speed"] == 6.0 and np.isfinite(r["T_obs"])]
    allv = [r["T_obs"] for r in rows70 if np.isfinite(r["T_obs"])]
    p10_env = float(np.percentile(env, 10))
    min_all = float(np.min(allv))
    min_stress = float(np.min(stress))

    print("=" * 78)
    print("Stage-3 TTC calibration -- derivation transcript")
    print("=" * 78)
    print(f"observable-time cells: {len(rows70)} "
          f"({len(refs)} seeds x {len(GEOMETRIES)} geometries x {len(SPEEDS)} "
          f"speeds), R = 70 m, exact quality")
    print(f"  design envelope (3-4 m/s): p10 = {p10_env:.2f} s, "
          f"min = {min(env):.2f} s")
    print(f"  6 m/s stress:              p10 = "
          f"{float(np.percentile(stress, 10)):.2f} s, min = {min_stress:.2f} s")

    # ---- R1 ----------------------------------------------------------
    t_pre = floor_to(p10_env - GUARD_PRE, 0.5)
    print(f"\nR1  T_PRE  = floor_0.5(p10 {p10_env:.2f} - guard {GUARD_PRE:.1f}) "
          f"= {t_pre:.1f} s")
    print(f"    margin over p10: {p10_env - t_pre:+.2f} s; "
          f"over the worst seed in the design envelope: "
          f"{min(env) - t_pre:+.2f} s; over the worst seed at 6 m/s: "
          f"{min_stress - t_pre:+.2f} s")
    print(f"    the manuscript's 16 s margin would be {p10_env - 16.0:+.2f} s "
          f"-> UNACHIEVABLE at R = 70 m")

    # ---- R2 ----------------------------------------------------------
    v2 = stage3_v2cfg("M1")
    scn = SCENARIOS["S2"]()
    cfg = scn["cfg"]
    t_react = float(v2.m1_obs_delay) + cfg.ctrl_period * cfg.dt
    t_lat = 2.0 * math.sqrt(3.0 / float(v2.a_lat))
    b_div = t_react + t_lat + GUARD_DIV
    t_div = floor_to(t_pre - b_div, 1.0)
    print(f"\nR2  t_react = m1_obs_delay {v2.m1_obs_delay:.2f} + one control "
          f"period {cfg.ctrl_period * cfg.dt:.2f} = {t_react:.2f} s")
    print(f"    t_lat   = 2*sqrt(divert_shift 3.0 / a_lat {v2.a_lat:.1f}) = "
          f"{t_lat:.3f} s")
    print(f"    B_DIV   = {t_react:.2f} + {t_lat:.3f} + guard {GUARD_DIV:.1f} "
          f"= {b_div:.2f} s")
    print(f"    T_DIVERT= floor_1.0({t_pre:.1f} - {b_div:.2f}) = {t_div:.1f} s")
    print(f"    check: T_DIVERT <= min T_obs over all cells ({min_all:.2f}) -> "
          f"{'PASS' if t_div <= min_all else 'FAIL'} "
          f"(margin {min_all - t_div:+.2f} s; {min_stress - t_div:+.2f} s at "
          f"6 m/s)")

    # ---- R3: measured RECOVER restoration ----------------------------
    rec_rows = []
    for sid in ("S2", "S3"):
        for seed in (1000, 1001):
            rr, row = recover_restoration_times(
                sid, seed, stage3_v2cfg("M1", **STAGE2_TTC))
            rec_rows.extend(rr)
            print(f"    RECOVER episodes {sid}/{seed}: "
                  f"{[(round(x['t_enter'], 1), round(x['dur_recover'], 1), None if not np.isfinite(x['restore']) else round(x['restore'], 1)) for x in rr]}")
    rest = [x["restore"] for x in rec_rows if np.isfinite(x["restore"])]
    rec_dur = [x["dur_recover"] for x in rec_rows]
    med_rest = float(np.median(rest)) if rest else float("nan")
    med_dur = float(np.median(rec_dur)) if rec_dur else 0.0
    h = 3.0
    ex = max(h, ceil_to(max(med_dur, 0.5), 0.5))
    t_rec = ceil_to(t_pre + ex, 0.5)
    print(f"\nR3  RECOVER episodes: {len(rec_rows)}; median duration "
          f"{med_dur:.2f} s; median restoration {med_rest:.2f} s "
          f"({len(rest)} episodes reached the restoration condition)")
    print(f"    T_REC = T_PRE + max(h, ceil_0.5(median duration)) = "
          f"{t_pre:.1f} + {ex:.1f} = {t_rec:.1f} s")

    # ---- R4 ----------------------------------------------------------
    print(f"\nR4  h = {h:.1f} s; T_DIVERT + h = {t_div + h:.1f} < "
          f"T_PRE = {t_pre:.1f} -> "
          f"{'PASS' if t_div + h < t_pre else 'FAIL'}")

    frozen = dict(ttc_pre=float(t_pre), ttc_divert=float(t_div),
                  ttc_rec=float(t_rec), ttc_hyst=float(h))
    print(f"\nFROZEN TUPLE: (T_PRE, T_DIVERT, T_REC, h) = "
          f"({t_pre:.1f}, {t_div:.1f}, {t_rec:.1f}, {h:.1f}) s")
    print(f"  shipped STAGE3_TTC in config_v2 = {STAGE3_TTC}")
    ok = all(abs(frozen[k] - float(STAGE3_TTC[k])) < 1e-9 for k in frozen)
    print(f"  config_v2.STAGE3_TTC agrees: {ok}")

    # ---- gates that must hold ---------------------------------------
    gates = []
    gates.append(("T_PRE <= p10(design envelope)", t_pre <= p10_env,
                  f"{t_pre:.1f} <= {p10_env:.2f}"))
    gates.append(("T_DIVERT <= min(T_obs) over all cells", t_div <= min_all,
                  f"{t_div:.1f} <= {min_all:.2f}"))
    gates.append(("T_PRE > T_DIVERT > 0", 0 < t_div < t_pre, "ordering"))
    gates.append(("T_REC > T_PRE", t_rec > t_pre, "ordering"))
    gates.append(("T_DIVERT + h < T_PRE", t_div + h < t_pre,
                  f"{t_div + h:.1f} < {t_pre:.1f}"))
    gates.append(("frozen == config_v2.STAGE3_TTC", ok, "config check"))
    print("\n  gates:")
    for name, good, detail in gates:
        print(f"    [{'PASS' if good else 'FAIL'}] {name}  ({detail})")

    # ---- mapping table (per cell) ------------------------------------
    mapping = []
    for r in agg70:
        tob = [x["T_obs"] for x in rows70
               if x["geometry"] == r["geometry"] and x["speed"] == r["speed"]
               and np.isfinite(x["T_obs"])]
        tob = sorted(tob)
        mapping.append(dict(
            geometry=r["geometry"], speed=r["speed"], n=len(tob),
            tau_det_mean=r["tau_det_mean"], tau_corr_mean=r["tau_corr_mean"],
            tau_hit_mean=r["tau_hit_mean"],
            T_obs_p10=r["T_obs_p10"], T_obs_mean=r["T_obs_mean"],
            T_obs_min=r["T_obs_min"], T_obs_max=r["T_obs_max"],
            # the largest T_PRE this cell alone would allow (its 10th pct)
            T_PRE_implied=r["T_obs_p10"],
            passes_frozen_T_PRE=bool(r["T_obs_p10"] >= t_pre),
            passes_frozen_T_DIVERT=bool(r["T_obs_min"] >= t_div),
            passes_proposed_16=bool(r["T_obs_p10"] >= 16.0),
            passes_proposed_8=bool(r["T_obs_p10"] >= 8.0),
            in_design_envelope=bool(r["speed"] in (3.0, 4.0))))
    write_csv(OUT / "ttc_mapping.csv", mapping)
    print("\n  per-cell mapping (T_PRE_implied = cell p10 of T_obs):")
    print(f"    {'geometry':<11}{'v':>5}{'T_obs p10':>11}{'T_PRE impl':>11}"
          f"{'>=12':>6}{'>=16':>6}")
    for m in mapping:
        print(f"    {m['geometry']:<11}{m['speed']:>5.0f}"
              f"{m['T_obs_p10']:>11.2f}{m['T_PRE_implied']:>11.2f}"
              f"{str(m['passes_frozen_T_PRE']):>6}"
              f"{str(m['passes_proposed_16']):>6}")

    # ---- required range: what would 16 s (and 12 s) need? ------------
    req = []
    for geom in GEOMETRIES:
        for v in SPEEDS:
            r12 = required_range(refs, geom, v, t_pre)
            r16 = required_range(refs, geom, v, 16.0)
            req.append(dict(geometry=geom, speed=v,
                            R_required_T_PRE_frozen=r12,
                            R_required_T_PRE_16=r16,
                            R_required_T_DIVERT=required_range(
                                refs, geom, v, t_div)))
    write_csv(OUT / "required_range.csv", req)
    env16 = [r["R_required_T_PRE_16"] for r in req
             if r["speed"] in (3.0, 4.0) and np.isfinite(r["R_required_T_PRE_16"])]
    st16 = [r["R_required_T_PRE_16"] for r in req
            if r["speed"] == 6.0 and np.isfinite(r["R_required_T_PRE_16"])]
    env12 = [r["R_required_T_PRE_frozen"] for r in req
             if r["speed"] in (3.0, 4.0)
             and np.isfinite(r["R_required_T_PRE_frozen"])]
    div_r = [r["R_required_T_DIVERT"] for r in req
             if np.isfinite(r["R_required_T_DIVERT"])]
    print("\n  required robot->obstacle range (smallest R with cell p10 >= "
          "target):")
    print(f"    {'geometry':<11}{'v':>5}{'R for T_DIVERT':>15}"
          f"{'R for T_PRE=12':>16}{'R for T_PRE=16':>16}")
    for r in req:
        print(f"    {r['geometry']:<11}{r['speed']:>5.0f}"
              f"{r['R_required_T_DIVERT']:>15.1f}"
              f"{r['R_required_T_PRE_frozen']:>16.1f}"
              f"{r['R_required_T_PRE_16']:>16.1f}")
    print(f"\n    16 s needs <= {max(env16):.1f} m at 3-4 m/s and "
          f"{max(st16):.1f} m at 6 m/s  (max over geometries)")
    print(f"    12 s needs <= {max(env12):.1f} m at 3-4 m/s")
    print(f"    T_DIVERT = {t_div:.1f} s needs <= {max(div_r):.1f} m "
          f"(i.e. 70 m is enough everywhere)")

    summary = dict(
        frozen=frozen, stage2_tuple=STAGE2_TTC,
        rules=dict(GUARD_PRE=GUARD_PRE, GUARD_DIV=GUARD_DIV,
                   t_react=t_react, t_lat=t_lat, B_DIV=b_div,
                   floor_pre=0.5, floor_div=1.0,
                   recover_restore_median=med_rest,
                   recover_duration_median=float(np.median(rec_dur))),
        measured=dict(p10_design_envelope=p10_env,
                      min_design_envelope=float(min(env)),
                      min_all_cells=min_all, min_stress_6ms=min_stress,
                      p10_stress_6ms=float(np.percentile(stress, 10)),
                      p50_design_envelope=float(np.percentile(env, 50)),
                      n_cells=len(rows70)),
        margins=dict(T_PRE_vs_p10=p10_env - t_pre,
                     T_PRE_vs_min_all=min_all - t_pre,
                     T_PRE_vs_16=16.0 - t_pre,
                     T_DIVERT_vs_min_all=min_all - t_div,
                     T_DIVERT_vs_min_stress=min_stress - t_div,
                     T_PRE16_margin_at_70=p10_env - 16.0),
        required_range=dict(env_16=max(env16), stress_16=max(st16),
                            env_12=max(env12), divert=max(div_r),
                            rows=req),
        mapping=mapping,
        gates=[dict(name=n, ok=bool(o), detail=d) for n, o, d in gates],
    )
    (OUT / "ttc_calibration.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    write_csv(OUT / "recover_episodes.csv", rec_rows)
    print(f"\nwrote {OUT / 'ttc_calibration.json'}")
    return 0 if all(o for _, o, _ in gates) else 1


if __name__ == "__main__":
    sys.exit(main())
