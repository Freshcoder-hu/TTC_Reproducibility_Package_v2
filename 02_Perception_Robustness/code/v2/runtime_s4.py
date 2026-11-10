"""Measured runtime of the Stage-4 geometry, and the Stage-5 projection.

    python -X utf8 -m v2.runtime_s4

Times one run of every Stage-4 scene (seed 1000, full V2 method and, for the
cheapest baseline, B2) with a single worker, measures the wall clock per run and
per control tick, and projects the formal matrices:

    screening matrix   4 methods x 3 cases  x 30 seeds
    formal matrix      8 methods x 10 scenes x 30 seeds   (manuscript Table 15,
                       stage 6; evaluation seeds 2000-2029)
    ablation matrix    300 runs (the V1 ablation size, retained in V2)

The projection is deliberately arithmetic (runs x measured seconds / workers):
no scaling law is assumed, because the cost is dominated by the number of
integration ticks, which the geometry fixes.

Writes ``results_v2_stage4/runtime.json``.
"""
from __future__ import annotations

import json
import statistics
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
from v2.scenarios_v2 import SCENARIOS_V2, resolve_scene  # noqa: E402

OUT = ROOT / "results_v2_stage4"

# the eight methods of the V1 comparison, retained for V2 (Table 15, stage 6)
FORMAL_METHODS = ["Ours_rollout", "B1", "B2", "B3", "VH-Path", "VH-Danger",
                  "B-Outmost", "B-PCBF"]


def time_cell(sid, alg, seed=1000, v2cfg=None):
    v2cfg = v2cfg or stage3_v2cfg("M1")
    registry_alg(alg)
    t0 = time.perf_counter()
    row, res = run_cell(sid, alg, seed, v2cfg=v2cfg, return_res=True)
    dt = time.perf_counter() - t0
    n_ctrl = max(len(res["logs"]), 1)
    n_tick = max(len(res["trace_states"]) if res.get("trace_states") else
                 int(round(res["logs"][-1]["t"] / 0.05)) if res["logs"] else 1, 1)
    return dict(scenario=sid, alg=alg, seed=seed, wall_s=dt,
                n_ctrl_ticks=n_ctrl, sim_s=float(res["logs"][-1]["t"])
                if res["logs"] else 0.0,
                ms_per_ctrl_tick=1e3 * dt / n_ctrl,
                ms_per_sim_second=1e3 * dt / max(
                    float(res["logs"][-1]["t"]) if res["logs"] else 1.0, 1e-9),
                ended=res["terminated_by"])


def project(per_run_s, n_runs, workers=(1, 4, 8, 16)):
    return {f"workers_{w}": n_runs * per_run_s / w for w in workers}


def main():
    rows = []
    for sid in SCENARIOS_V2:
        rows.append(time_cell(sid, "Ours_rollout"))
    rows.append(time_cell("V2S2", "B2"))
    rows.append(time_cell("V2S3", "B2"))
    for r in rows:
        print(f"{r['scenario']:9s}{r['alg']:14s}wall={r['wall_s']:7.2f} s  "
              f"ctrl_ticks={r['n_ctrl_ticks']:5d}  "
              f"sim={r['sim_s']:6.1f} s  "
              f"{r['ms_per_ctrl_tick']:6.1f} ms/tick  "
              f"{r['ms_per_sim_second']:7.1f} ms/sim-s  {r['ended']}")
    full = [r for r in rows if r["alg"] == "Ours_rollout"]
    mean_full = float(np.mean([r["wall_s"] for r in full]))
    mean_base = float(np.mean([r["wall_s"] for r in rows
                               if r["alg"] == "B2"]))
    # method-cost split: the baselines do not plan, so they are cheaper; the
    # projection uses the measured mean of a planning method and of a
    # non-planning one (4 planning + 4 non-planning out of the 8)
    planned_s = mean_full
    base_s = mean_base
    eight_cost = 4 * planned_s + 4 * base_s
    proj = dict(
        measured=rows,
        mean_wall_s_planning_method=planned_s,
        mean_wall_s_baseline_method=base_s,
        per_seed_all_10_scenes_s=eight_cost,
        matrices=dict(
            screening_4x3x30=dict(
                n_runs=4 * 3 * 30,
                s=30 * (3 * planned_s + 3 * base_s),
                projection=project((3 * planned_s + 3 * base_s) / 3.0,
                                   4 * 3 * 30)),
            formal_8x10x30=dict(
                n_runs=8 * 10 * 30,
                s=30 * eight_cost,
                projection=project(eight_cost, 8 * 10 * 30)),
            ablation_300=dict(n_runs=300, s=300 * planned_s,
                              projection=project(planned_s, 300))))
    print(f"\nmean wall per run: planning method {planned_s:.2f} s, "
          f"baseline {base_s:.2f} s")
    for name, m in proj["matrices"].items():
        p = m["projection"]
        print(f"{name:18s} {m['n_runs']:5d} runs  single-thread "
              f"{m['s'] / 3600:6.2f} h   x4 {p['workers_4'] / 3600:5.2f} h   "
              f"x8 {p['workers_8'] / 3600:5.2f} h   x16 "
              f"{p['workers_16'] / 3600:5.2f} h")
    (OUT / "runtime.json").write_text(json.dumps(proj, indent=2),
                                      encoding="utf-8")
    print(f"\nwrote {OUT / 'runtime.json'}")


if __name__ == "__main__":
    main()
