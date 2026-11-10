"""Measured parallel efficiency of the Stage-4 runner (the jobs=4 / jobs=12 input).

    python -X utf8 -m v2.parallel_efficiency_s4

Submits the same cell (V2S2 / Ours_rollout / seed 1000) eight times through
``ProcessPoolExecutor`` with ``max_workers = 1, 4, 12`` and reports the wall
clock, the implied speed-up and the efficiency.  This is the only *measured*
parallel datapoint for the Stage-5 projection besides the stored pilot
(36 runs at ``jobs=8`` in 196.4 s).

Why it matters: the Stage-4 per-run cost is ~13 ms per control tick on this
laptop, i.e. the plant dominates and the run is cache-heavy; eight workers on a
6P+4E CPU therefore do not give an 8x speed-up (the pilot already hinted at
this).  The projection in ``v2/project_runtime_s4.py`` uses the efficiency
measured here instead of assuming perfect packing.

Writes ``results_v2_stage4/parallel_efficiency.csv`` / ``.json``.
"""
from __future__ import annotations

import csv
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

OUT = ROOT / "results_v2_stage4"
CELL = ("V2S2", "Ours_rollout", 1000)
N = 8


def _one(_):
    import corrections as _c

    _c.install(environment=True, contact=True)
    from v2.config_v2 import stage3_v2cfg
    from v2.runner_s4 import run_cell

    run_cell(*CELL, v2cfg=stage3_v2cfg("M1"))
    return 0


def main():
    rows = []
    for jobs in (1, 4, 12):
        t0 = time.perf_counter()
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            list(ex.map(_one, range(N)))
        wall = time.perf_counter() - t0
        rows.append(dict(cell="/".join(map(str, CELL)), n_runs=N, jobs=jobs,
                         wall_s=wall, s_per_run=wall / N))
        print(f"jobs={jobs:2d}: {N} runs in {wall:7.2f} s -> "
              f"{wall / N:6.2f} s/run", flush=True)
    base = rows[0]["wall_s"]
    for r in rows:
        r["speedup"] = base / r["wall_s"]
        r["efficiency"] = r["speedup"] / r["jobs"]
        print(f"jobs={r['jobs']:2d}: speed-up {r['speedup']:.2f}x, "
              f"efficiency {100 * r['efficiency']:.0f} %")
    with open(OUT / "parallel_efficiency.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    (OUT / "parallel_efficiency.json").write_text(json.dumps(rows, indent=2),
                                                  encoding="utf-8")
    print(f"\nwrote {OUT / 'parallel_efficiency.csv'}")


if __name__ == "__main__":
    main()
