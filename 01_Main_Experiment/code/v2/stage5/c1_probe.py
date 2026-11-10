"""Item 3: the C1 audit -- why ``C1 violations = 0`` is vacuous, and a stress
case in which C1-low does bind.

    python -X utf8 -m v2.stage5.c1_probe --jobs 6

Stage 4 reported ``bind_C1_low_sheep = bind_C1_high_sheep = 0`` in every run and
therefore that "C1 violations are 0".  A bound that never binds cannot be
violated, so the zero carries no information.  There are **two independent
reasons** on this configuration, and this script establishes both:

1. **The band itself is degenerate for the shipped animal model.**  ``M1`` uses
   ``s_lo = m_s_min = 0.0`` (the author decision that animals may slow to a
   standstill), so the C1 *lower* bound it must respect is 0.  ``limits_v2`` has
   no C1-upper counter at all: the upper bound ``s_hi = m_v_max = 2.0`` is
   enforced through the C3 speed clip.

2. **Even with a non-zero lower bound the geometry forbids binding.**  The
   projection can only flag ``c1_low`` when the C2 turn clip lets the animal ask
   for a turn at which the reachable speed set cannot contain ``s_lo``:

       |dth| >= dth_acc = asin(A / s_old),   A = a_sheep * dt = 0.20 m/s
       and   s_lo > sqrt(s_old^2 - A^2)

   With ``|dth| <= omega_s * dt = 0.15 rad`` and ``s_old <= s_hi = 1.5``:
   ``asin(0.20/1.34) = 0.150``, so the first condition needs ``s_old > 1.34``,
   and there ``sqrt(s_old^2 - 0.04) >= 1.325 > s_lo`` for every admissible
   ``s_lo <= 1.0``.  The two conditions cannot hold together: the C2 turn cap
   (0.15 rad/tick) is *smaller* than the turn at which the C1 lower bound first
   becomes unreachable (>= 0.169 rad at ``s_old = 1.34``).

Part 1 evaluates the real ``project_sheep_increment`` over the reachable box and
confirms reason 2 numerically (0 binding cells with the shipped constants).
Part 2 runs a **documented stress configuration** (``m_s_min = 1.0`` so the V1
C1 band [1.0, 1.5] is re-imposed, ``a_sheep = 1.0`` m/s^2 so A = 0.05 m/s) in
the simulator: ``bind_C1_low_sheep > 0`` **and** ``violation_C1_low = 0``.  That
is the evidence that the machinery works when the band is non-degenerate -- and
that its silence in the formal matrix is structural, not a pass.

Outputs: ``results_v2_stage5/c1_audit/C1_grid.csv``, ``C1_stress_runs.csv``,
``C1_audit.json``.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.limits_v2 import Counters, project_sheep_increment  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5.runs import run_one  # noqa: E402

DT = 0.05
EPS_V = 1e-6
S_HI = 1.5


def grid_scan(a_s, omega_s, s_lo, s_hi, s_old_max=None, step_s=0.005):
    """Evaluate the real projection over the **reachable** (speed, turn) box.

    ``s_old`` is swept only over ``[s_lo, s_hi]``: the projection is never asked
    to handle a speed outside the model's own admissible band (``M1``:
    ``[0, m_v_max] = [0, 2.0]``; ``M0``: ``[s_min, s_max] = [1.0, 1.5]``), and
    evaluating it outside that box produces spurious ``c1_low`` flags that say
    nothing about the simulator.
    """
    rows = []
    n_bind = 0
    dth_max = omega_s * DT
    hi = float(s_old_max if s_old_max is not None else s_hi)
    for s_old in np.arange(float(s_lo), hi + 1e-12, step_s):
        for dth in np.arange(-3.0 * dth_max, 3.0 * dth_max + 1e-12,
                             max(0.005, 3.0 * dth_max / 31.0)):
            c = Counters()
            project_sheep_increment(float(s_old), float(dth), float(s_old),
                                    dt=DT, a_s=float(a_s),
                                    omega_s=float(omega_s), s_lo=float(s_lo),
                                    s_hi=float(s_hi), eps_v=EPS_V,
                                    counters=c)
            bind = int(c.get("bind_C1_low_sheep") > 0)
            n_bind += bind
            if bind:
                rows.append(dict(a_s=a_s, omega_s=omega_s, s_lo=s_lo,
                                 s_hi=s_hi, s_old=float(s_old),
                                 dth=float(dth), bind=1))
    return rows, n_bind


def _task(t):
    scene, seed, cfg = t
    row = run_one(scene, "Ours_rollout", seed, v2cfg=cfg)
    keep = dict(scene=scene, seed=seed, config_id=row.get("config_id"),
                kind=("stress" if cfg.m_s_min > 0 else "shipped"),
                m_s_min=cfg.m_s_min, a_sheep=cfg.a_sheep,
                bind_C1_low=row.get("bind_C1_low_sheep"),
                bind_C1_high=row.get("bind_C1_high_sheep"),
                bind_C3=row.get("bind_C3_sheep"),
                opp_sheep_steps=row.get("opp_sheep_steps"),
                viol_C1_low=row.get("violation_C1_low"),
                viol_C1_high=row.get("violation_C1_high"),
                viol_C2=row.get("violation_C2"),
                viol_C3=row.get("violation_C3"),
                C=row.get("collision_rate"),
                clr_so=row.get("min_surface_clearance_so"),
                rho=row.get("rho_avg"), arrival=row.get("arrival"),
                ended=row.get("terminated_by"))
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=["V2S2", "V2S3"])
    ap.add_argument("--seeds", nargs="+", type=int,
                    default=common.CALIB_SEEDS[:3])
    ap.add_argument("--jobs", type=int, default=6)
    args = ap.parse_args()
    common.ensure_dirs()
    out = common.C1_DIR

    # ---------------- part 1: the structural scan ----------------------
    grids = [
        dict(name="shipped_M1", a_s=4.0, omega_s=3.0, s_lo=0.0, s_hi=2.0),
        dict(name="shipped_M0_band", a_s=4.0, omega_s=3.0, s_lo=1.0, s_hi=1.5),
        dict(name="turn_cap_relaxed", a_s=4.0, omega_s=8.0, s_lo=1.0, s_hi=1.5),
        dict(name="stress_a1_msmin1", a_s=1.0, omega_s=3.0, s_lo=1.0, s_hi=2.0),
        dict(name="a2_msmin1", a_s=2.0, omega_s=3.0, s_lo=1.0, s_hi=2.0),
    ]
    all_rows, summary = [], []
    for g in grids:
        rows, n = grid_scan(g["a_s"], g["omega_s"], g["s_lo"], g["s_hi"])
        for r in rows:
            r["grid"] = g["name"]
            all_rows.append(r)
        summary.append(dict(grid=g["name"], a_s=g["a_s"], omega_s=g["omega_s"],
                            s_lo=g["s_lo"], s_hi=g["s_hi"], n_bind=n,
                            dth_cap=round(g["omega_s"] * DT, 4),
                            s_old_min=(min((r["s_old"] for r in rows),
                                           default=float("nan"))),
                            s_old_max=(max((r["s_old"] for r in rows),
                                           default=float("nan")))))
        print(f"grid {g['name']:20s} a_s={g['a_s']:<4} omega={g['omega_s']:<4} "
              f"s_lo={g['s_lo']:<4} s_hi={g['s_hi']:<4} -> {n:5d} binding cells",
              flush=True)
    with open(out / "C1_grid.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["grid", "a_s", "omega_s", "s_lo",
                                          "s_hi", "s_old", "dth", "bind"],
                           extrasaction="ignore")
        w.writeheader()
        w.writerows(all_rows)

    # ---------------- part 2: the documented stress case ---------------
    shipped = stage3_v2cfg("M1")
    stress = shipped.with_(m_s_min=1.0, a_sheep=1.0)
    tasks = [(s, sd, shipped) for s in args.scenes for sd in args.seeds] + \
            [(s, sd, stress) for s in args.scenes for sd in args.seeds]
    print(f"\nC1 stress: {len(tasks)} runs "
          f"(shipped {shipped.config_id} vs stress {stress.config_id})",
          flush=True)
    if args.jobs > 1:
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            runs = list(ex.map(_task, tasks, chunksize=1))
    else:
        runs = [_task(t) for t in tasks]
    with open(out / "C1_stress_runs.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(runs)
    for kind in ("shipped", "stress"):
        sub = [r for r in runs if r["kind"] == kind]
        print(f"  {kind:8s}: bind_C1_low={sum(int(r['bind_C1_low'] or 0) for r in sub):6d} "
              f"viol_C1_low={sum(int(r['viol_C1_low'] or 0) for r in sub)} "
              f"C={np.mean([float(r['C']) for r in sub]):.4f} "
              f"rho={np.mean([float(r['rho']) for r in sub]):.3f}")
    common.write_json(out / "C1_audit.json", dict(
        shipped_config_id=shipped.config_id,
        stress_config_id=stress.config_id,
        stress_overrides=dict(m_s_min=1.0, a_sheep=1.0),
        grid_summary=summary, runs=runs,
        shipped_c1_low_bind=int(sum(int(r["bind_C1_low"] or 0)
                                    for r in runs if r["kind"] == "shipped")),
        stress_c1_low_bind=int(sum(int(r["bind_C1_low"] or 0)
                                   for r in runs if r["kind"] == "stress")),
        stress_c1_low_viol=int(sum(int(r["viol_C1_low"] or 0)
                                   for r in runs if r["kind"] == "stress"))))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
