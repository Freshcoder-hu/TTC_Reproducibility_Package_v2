"""V1 vs V2 comparison matrix for the Stage-1 report.

Three arms on identical scenarios/seeds, paired:
  V1       -- the archived plant (compat configuration, bit-exact to the archive)
  V2a      -- V2 as specified by the author (d_rr_safe = 1.2 m)
  V2b      -- V2 with the evidence-based robot-robot margin (d_rr_safe = 2.0 m)

    python -m v2.compare_v1_v2 --jobs 4
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace

import corrections

corrections.install(environment=True, contact=True)

from v2.config_v2 import compat_v1cfg, default_v2cfg   # noqa: E402
from v2.runner_v2 import run_cell                     # noqa: E402

ARMS = {
    "V1": dict(compat=True),
    "V1solid": dict(compat=True, r_robot=0.45),
    "V2a": dict(v2cfg=default_v2cfg(d_rr_safe=1.2)),
    "V2b": dict(v2cfg=default_v2cfg()),
}

COLS = ["success", "collision_rate", "min_dist", "dmin_robot_obs",
        "dmin_robot_robot", "rho_avg", "rho_min", "max_dev", "cflock_min",
        "min_surface_clearance_rr", "min_surface_clearance_ro",
        "min_surface_clearance_rs", "min_surface_clearance_so",
        "n_contact_events_rr", "n_contact_events_ro", "n_contact_events_rs",
        "n_contact_events_so", "first_contact_class", "first_contact_time",
        "n_infeasible", "n_infeasible_S", "n_infeasible_P", "n_contact_C",
        "n_brake_events", "violation_C3", "violation_R4", "violation_lat",
        "ttc_min", "recovery_success"]


def _task(t):
    sid, arm, seed = t
    kw = dict(ARMS[arm])
    row = run_cell(sid, "Ours_rollout", seed, **kw)
    row["arm"] = arm
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=["S2", "S3"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[200, 201, 202])
    ap.add_argument("--arms", nargs="+", default=list(ARMS))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="results_v2_stage1/compare.csv")
    args = ap.parse_args()

    tasks = [(s, a, sd) for s in args.scenarios for a in args.arms
             for sd in args.seeds]
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            rows = list(ex.map(_task, tasks))
    else:
        rows = [_task(t) for t in tasks]

    import os
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "arm", "seed"] + COLS)
        for r in rows:
            w.writerow([r["scenario"], r["arm"], r["seed"]]
                       + [r.get(c) for c in COLS])
    print(f"wrote {args.out} ({len(rows)} rows)")

    def mean(vals):
        v = [x for x in vals if isinstance(x, (int, float)) and x == x]
        return sum(v) / len(v) if v else float("nan")

    print(f"\n{'scen':5s} {'arm':4s} " + " ".join(f"{c[:10]:>11s}"
                                                  for c in COLS[:9]))
    for s in args.scenarios:
        for a in args.arms:
            rr = [r for r in rows if r["scenario"] == s and r["arm"] == a]
            print(f"{s:5s} {a:4s} " + " ".join(
                f"{mean([r.get(c) for r in rr]):11.3f}" for c in COLS[:9]))
    print(f"\n{'scen':5s} {'arm':4s} " + " ".join(f"{c[:10]:>11s}"
                                                  for c in COLS[9:18]))
    for s in args.scenarios:
        for a in args.arms:
            rr = [r for r in rows if r["scenario"] == s and r["arm"] == a]
            vals = []
            for c in COLS[9:18]:
                v = [r.get(c) for r in rr]
                if isinstance(v[0], str):
                    vals.append(max(set(v), key=v.count))
                else:
                    vals.append(f"{mean(v):.3f}")
            print(f"{s:5s} {a:4s} " + " ".join(f"{str(x):>11s}" for x in vals))
    print(f"\n{'scen':5s} {'arm':4s} " + " ".join(f"{c[:10]:>11s}"
                                                  for c in COLS[18:]))
    for s in args.scenarios:
        for a in args.arms:
            rr = [r for r in rows if r["scenario"] == s and r["arm"] == a]
            print(f"{s:5s} {a:4s} " + " ".join(
                f"{mean([r.get(c) for r in rr]):11.3f}" for c in COLS[18:]))


if __name__ == "__main__":
    main()
