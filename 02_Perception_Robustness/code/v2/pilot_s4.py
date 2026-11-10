"""Stage-4 pilot screening (manuscript Table 15 / §4.8).

    python -X utf8 -m v2.pilot_s4 --jobs 4

4 methods x 3 case types x 3 calibration seeds = 36 runs.

methods
    B1              path shepherding only (no risk logic)
    B2              distance-threshold reactive (V1 calibrated threshold)
    Ours_event      the V2 event controller **without** the rollout planner
    Ours_rollout    the full V2 method
cases
    no_obstacle          V2S1  (140 m, 16 m, no obstacle)
    single_obstacle      V2S2  (140 m, 16 m, one 4 m/s oblique graze)
    sequential_obstacle  V2S3  (220 m, 16 m, three releases)
seeds
    1000, 1001, 1002  (calibration set only; 2000-2029 is untouched)

This is **not** the formal comparison: n = 3, no significance is claimed, and
the output is a screening table that decides whether the Stage-4 geometry is
worth the formal 8-method matrix (Stage 5).

Censoring: a run that hits the horizon without reaching the goal is
**right-censored**, not dropped.  ``censored`` is 1 for those runs, they stay in
every denominator, and their completion time is reported as NaN while the
censored count is displayed next to each cell.

Outputs (``results_v2_stage4/pilot/``): ``pilot_runs.csv``,
``pilot_summary.csv``, ``pilot_summary.json``, ``pilot_table.md``.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import registry_alg, run_cell  # noqa: E402
from v2.scenarios_v2 import PILOT_CASES, W_V2  # noqa: E402

OUT = ROOT / "results_v2_stage4" / "pilot"
METHODS = ["B1", "B2", "Ours_event", "Ours_rollout"]
SEEDS = [1000, 1001, 1002]

RUN_COLS = [
    "case", "scenario", "method", "seed", "config_id",
    "contact_fraction", "any_contact", "n_contact_events_so",
    "min_center_dist", "min_clearance_so", "dmin_robot_robot",
    "min_clearance_ro", "min_clearance_rs",
    "rho_mean", "rho_min", "max_dev", "max_dev_norm",
    "success", "completion", "exec_time", "censored", "ended", "sim_time_s",
    "coverage_at_end", "t_pre_on", "t_divert_on", "first_detect",
    "recovery", "recovery_success", "preempt_ok",
    "violation_C3", "violation_R4", "violation_lat",
    "violation_C1_low", "violation_C1_high", "violation_C2",
    "n_rr_contacts", "n_ro_contacts",
]


def _f(v, default=np.nan):
    try:
        x = float(v)
        return x if np.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def one_run(case, sid, method, seed, v2cfg):
    row, res = run_cell(sid, method, seed, v2cfg=v2cfg, return_res=True)
    v2 = res["v2"]
    last = res["logs"][-1] if res["logs"] else {}
    success = bool(row.get("success", False))
    return dict(
        case=case, scenario=sid, method=method, seed=seed,
        config_id=row.get("config_id"),
        contact_fraction=_f(row.get("collision_rate"), 0.0),
        any_contact=bool(_f(row.get("collision_rate"), 0.0) > 0.0),
        n_contact_events_so=_f(v2.get("n_contact_events_so"), 0.0),
        min_center_dist=_f(row.get("min_dist")),
        min_clearance_so=_f(v2.get("min_surface_clearance_so")),
        dmin_robot_robot=_f(row.get("dmin_robot_robot")),
        min_clearance_ro=_f(v2.get("min_surface_clearance_ro")),
        min_clearance_rs=_f(v2.get("min_surface_clearance_rs")),
        rho_mean=_f(row.get("rho_avg")), rho_min=_f(row.get("rho_min")),
        max_dev=_f(row.get("max_dev")),
        max_dev_norm=_f(row.get("max_dev")) / W_V2,
        success=success, completion=bool(row.get("completion", False)),
        exec_time=_f(row.get("exec_time")),
        censored=0 if success else 1,
        ended=res["terminated_by"],
        sim_time_s=_f(last.get("t")),
        coverage_at_end=_f(last.get("rho")),
        t_pre_on=_f(row.get("t_pre_on")), t_divert_on=_f(row.get("t_divert_on")),
        first_detect=_f(v2.get("sense_first_detect_1")),
        recovery=_f(row.get("recovery")),
        recovery_success=bool(row.get("recovery_success", False)),
        preempt_ok=bool(row.get("preempt_ok", False)),
        violation_C3=_f(row.get("violation_C3"), 0.0),
        violation_R4=_f(row.get("violation_R4"), 0.0),
        violation_lat=_f(row.get("violation_lat"), 0.0),
        violation_C1_low=_f(row.get("violation_C1_low"), 0.0),
        violation_C1_high=_f(row.get("violation_C1_high"), 0.0),
        violation_C2=_f(row.get("violation_C2"), 0.0),
        n_rr_contacts=_f(v2.get("n_contact_events_rr"), 0.0),
        n_ro_contacts=_f(v2.get("n_contact_events_ro"), 0.0),
        first_detect_2=_f(v2.get("sense_first_detect_2")),
        first_detect_3=_f(v2.get("sense_first_detect_3")),
        event_order_ok=bool(row.get("event_order_ok", False)),
        recover_start_1=_f(row.get("recover_start_1")),
        dur_recover=_f(row.get("dur_recover")),
        n_ctrl_ticks=_f(row.get("n_ctrl_ticks")),
    )


def _mean(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and np.isfinite(x)]
    return float(np.mean(xs)) if xs else np.nan


def _summ(rows):
    return dict(
        contact_fraction=_mean([r["contact_fraction"] for r in rows]),
        any_contact_rate=_mean([1.0 if r["any_contact"] else 0.0
                                for r in rows]),
        n_contact_events_so=_mean([r["n_contact_events_so"] for r in rows]),
        min_center_dist=_mean([r["min_center_dist"] for r in rows]),
        min_clearance_so=_mean([r["min_clearance_so"] for r in rows]),
        dmin_robot_robot=_mean([r["dmin_robot_robot"] for r in rows]),
        rho_mean=_mean([r["rho_mean"] for r in rows]),
        rho_min=_mean([r["rho_min"] for r in rows]),
        max_dev=_mean([r["max_dev"] for r in rows]),
        max_dev_norm=_mean([r["max_dev_norm"] for r in rows]),
        completion_rate=_mean([1.0 if r["success"] else 0.0 for r in rows]),
        n_censored=int(sum(r["censored"] for r in rows)),
        n_runs=len(rows),
        exec_time_mean=_mean([r["exec_time"] for r in rows]),
        coverage_at_end=_mean([r["coverage_at_end"] for r in rows]),
        t_pre_on=_mean([r["t_pre_on"] for r in rows]),
        t_divert_on=_mean([r["t_divert_on"] for r in rows]),
        first_detect=_mean([r["first_detect"] for r in rows]),
        recovery=_mean([r["recovery"] for r in rows]),
        n_rr_contacts=_mean([r["n_rr_contacts"] for r in rows]),
        n_ro_contacts=_mean([r["n_ro_contacts"] for r in rows]),
        sim_time_s=_mean([r["sim_time_s"] for r in rows]),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--seeds", nargs="+", type=int, default=SEEDS)
    ap.add_argument("--methods", nargs="+", default=METHODS)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    v2cfg = stage3_v2cfg("M1")
    for m in args.methods:
        registry_alg(m)

    tasks = [(case, sid, m, seed)
             for case, sid in PILOT_CASES.items()
             for m in args.methods for seed in args.seeds]
    t0 = time.time()
    print(f"Stage-4 pilot: {len(tasks)} runs "
          f"({len(PILOT_CASES)} cases x {len(args.methods)} methods x "
          f"{len(args.seeds)} seeds), jobs={args.jobs}, "
          f"config_id={v2cfg.config_id}")
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            rows = list(ex.map(_task, tasks))
    else:
        rows = [_task(t) for t in tasks]
    wall = time.time() - t0

    keys = RUN_COLS + ["first_detect_2", "first_detect_3", "event_order_ok",
                       "recover_start_1", "dur_recover", "n_ctrl_ticks"]
    with open(out / "pilot_runs.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---- aggregate per (case, method) ---------------------------------
    table = []
    for case in PILOT_CASES:
        for m in args.methods:
            sub = [r for r in rows if r["case"] == case and r["method"] == m]
            row = dict(case=case, scenario=PILOT_CASES[case], method=m,
                       n=len(sub))
            row.update(_summ(sub))
            table.append(row)
    keys2 = list(table[0])
    with open(out / "pilot_summary.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys2, extrasaction="ignore")
        w.writeheader()
        for r in table:
            w.writerow(r)

    hdr = (f"{'case':22s}{'method':14s}{'C':>7s}{'anyC':>6s}{'nSO':>5s}"
           f"{'minDist':>8s}{'clr_so':>8s}{'rho':>7s}{'dev/W':>7s}"
           f"{'compl':>7s}{'cens':>6s}{'rhoEnd':>7s}{'PRE':>7s}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in table:
        print(f"{r['case']:22s}{r['method']:14s}"
              f"{r['contact_fraction']:>7.3f}{r['any_contact_rate']:>6.2f}"
              f"{r['n_contact_events_so']:>5.1f}"
              f"{r['min_center_dist']:>8.2f}{r['min_clearance_so']:>8.2f}"
              f"{r['rho_mean']:>7.3f}{r['max_dev_norm']:>7.3f}"
              f"{r['completion_rate']:>7.2f}"
              f"{r['n_censored']:>4d}/{r['n_runs']:<2d}"
              f"{r['coverage_at_end']:>7.3f}{r['t_pre_on']:>7.1f}")

    # ---- paired contrasts (same seed, same case) ----------------------
    print("\npaired contrasts vs Ours_rollout (negative dC = Ours has fewer "
          "contacts):")
    contrasts = []
    for case in PILOT_CASES:
        ours = {r["seed"]: r for r in rows
                if r["case"] == case and r["method"] == "Ours_rollout"}
        for m in args.methods:
            if m == "Ours_rollout":
                continue
            base = {r["seed"]: r for r in rows
                    if r["case"] == case and r["method"] == m}
            seeds = sorted(set(ours) & set(base))
            dC = [ours[s]["contact_fraction"] - base[s]["contact_fraction"]
                  for s in seeds]
            dmin = [ours[s]["min_center_dist"] - base[s]["min_center_dist"]
                    for s in seeds]
            contrasts.append(dict(case=case, baseline=m, n_pairs=len(seeds),
                                  dC_mean=float(np.mean(dC)) if dC else np.nan,
                                  dC_wins=int(sum(1 for x in dC if x < 0)),
                                  dC_ties=int(sum(1 for x in dC if x == 0)),
                                  dmin_mean=float(np.nanmean(dmin))
                                  if dmin else np.nan))
            print(f"  {case:22s} Ours - {m:14s} dC = "
                  f"{contrasts[-1]['dC_mean']:+.3f} "
                  f"(better on {contrasts[-1]['dC_wins']}/{len(seeds)} seeds, "
                  f"ties {contrasts[-1]['dC_ties']}), "
                  f"d(min centre dist) = {contrasts[-1]['dmin_mean']:+.2f} m")

    (out / "pilot_summary.json").write_text(json.dumps(dict(
        config_id=v2cfg.config_id, methods=args.methods,
        cases={k: v for k, v in PILOT_CASES.items()},
        seeds=list(args.seeds), wall_s=wall, table=table,
        contrasts=contrasts, n_runs=len(rows)), indent=2, default=float),
        encoding="utf-8")

    # ---- markdown table for the report --------------------------------
    md = ["| case | method | C | any-contact | min centre dist [m] | "
          "min clearance [m] | rho | max_dev/W | completion | censored |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in table:
        md.append(f"| {r['case']} | {r['method']} | "
                  f"{r['contact_fraction']:.3f} | {r['any_contact_rate']:.2f} "
                  f"| {r['min_center_dist']:.2f} | {r['min_clearance_so']:.2f} "
                  f"| {r['rho_mean']:.3f} | {r['max_dev_norm']:.3f} | "
                  f"{r['completion_rate']:.2f} | "
                  f"{r['n_censored']}/{r['n_runs']} |")
    (out / "pilot_table.md").write_text("\n".join(md) + "\n",
                                        encoding="utf-8")
    print(f"\nwrote {out}  ({len(rows)} runs, {wall:.1f} s)")


def _task(t):
    case, sid, method, seed = t
    return one_run(case, sid, method, seed, stage3_v2cfg("M1"))


if __name__ == "__main__":
    main()
