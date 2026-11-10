"""Diagnostic sweep over V2 design variants (not part of the reported results).

Used to decide, with evidence, the two things the Stage-1 report must justify:
the soft robot-robot target and what the braking supervisor is allowed to act on.
Each variant is one `run_cell` on the same scenario/seed.

    python -m v2.diag_v2 --scenarios S3 --seeds 200 201
"""
from __future__ import annotations

import argparse
from dataclasses import replace

import corrections

corrections.install(environment=True, contact=True)

from v2.config_v2 import default_v2cfg           # noqa: E402
from v2.runner_v2 import run_cell               # noqa: E402

VARIANTS = {
    # explicit overrides, so the table in the report is reproducible regardless
    # of what the shipped default happens to be
    "shipped": {},
    "drr1.2": dict(d_rr_safe=1.2),
    "drr1.5": dict(d_rr_safe=1.5),
    "drr2.0": dict(d_rr_safe=2.0),
    "no_brake": dict(brake_enabled=False),
    "brake_ro_only": dict(brake_classes=("ro",)),
    "brake_rr_only": dict(brake_classes=("rr",)),
    "no_lat": dict(enforce_lat=False),
    "no_r4": dict(enforce_r4=False),
}

COLS = ["success", "collision_rate", "min_dist", "dmin_robot_obs",
        "dmin_robot_robot", "rho_avg", "max_dev",
        "min_surface_clearance_rr", "min_surface_clearance_ro",
        "min_surface_clearance_rs",
        "n_contact_events_rr", "n_contact_events_ro", "n_contact_events_rs",
        "n_infeasible", "n_infeasible_S", "n_infeasible_P", "n_contact_C",
        "n_brake_events", "violation_C3", "violation_R4", "violation_lat"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=["S3"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[200, 201])
    ap.add_argument("--alg", default="Ours_rollout")
    ap.add_argument("--variants", nargs="+", default=list(VARIANTS))
    args = ap.parse_args()

    base = default_v2cfg()
    for sid in args.scenarios:
        print(f"\n===== {sid} / {args.alg} =====")
        rows = {}
        for name in args.variants:
            kw = VARIANTS[name]
            v2cfg = replace(base, **kw) if kw else base
            acc = {c: [] for c in COLS}
            for seed in args.seeds:
                row = run_cell(sid, args.alg, seed, v2cfg=v2cfg)
                for c in COLS:
                    acc[c].append(row.get(c))
            rows[name] = acc
        hdr = f"{'variant':18s}" + "".join(f"{c[:11]:>12s}" for c in COLS)
        print(hdr)
        for name, acc in rows.items():
            def f(vals):
                nums = [v for v in vals if isinstance(v, (int, float))
                        and v == v]
                if not nums:
                    b = vals[0]
                    return str(b)[:11] if b is not None else "?"
                return f"{sum(nums) / len(nums):.3g}"
            print(f"{name:18s}" + "".join(f"{f(acc[c]):>12s}" for c in COLS))


if __name__ == "__main__":
    main()
