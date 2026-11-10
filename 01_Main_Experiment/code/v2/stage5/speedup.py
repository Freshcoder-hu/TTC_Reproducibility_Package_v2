"""Measured jobs=1 / 4 / 12 speed-up for the Stage-5 projection.

    python -X utf8 -m v2.stage5.speedup

Eight identical cells (V2S2 / Ours_rollout / seed 2000) through
``ProcessPoolExecutor`` at 1, 4 and 12 workers.  This is the measured input of
the Stage-5 projection (``tables.timing_table``) and the source of the clean,
*uncontended* per-cycle cost: the ``ctrl_ms`` of the ``jobs = 1`` arm is measured
with no other work on the machine, unlike the 12-way concurrent matrix batches.

Writes ``results_v2_stage5/checks/speedup.csv`` / ``.json``.
"""
from __future__ import annotations

import csv
import json
import os
import sys
import time
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
from v2.stage5 import common  # noqa: E402
from v2.stage5.runs import run_one  # noqa: E402

CELL = ("V2S2", "Ours_rollout", 2000)
N = 8


def _one(_):
    row = run_one(*CELL, v2cfg=stage3_v2cfg("M1"))
    return float(row.get("ctrl_ms") or np.nan), float(row.get("wall_s") or 0.0)


def main():
    common.ensure_dirs()
    rows = []
    for jobs in (1, 4, 12):
        t0 = time.perf_counter()
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            res = list(ex.map(_one, range(N)))
        wall = time.perf_counter() - t0
        ctrl = [r[0] for r in res if np.isfinite(r[0])]
        rows.append(dict(cell="/".join(map(str, CELL)), n_runs=N, jobs=jobs,
                         wall_s=wall, s_per_run=wall / N,
                         ctrl_ms_mean=float(np.mean(ctrl)) if ctrl else np.nan,
                         ctrl_ms_sd=(float(np.std(ctrl)) if ctrl else np.nan)))
        print(f"jobs={jobs:2d}: {N} runs in {wall:7.2f} s -> {wall / N:5.2f} "
              f"s/run, ctrl_ms={np.mean(ctrl):.3f}", flush=True)
    base = rows[0]["wall_s"]
    for r in rows:
        r["speedup"] = base / r["wall_s"]
        r["efficiency"] = r["speedup"] / r["jobs"]
        print(f"jobs={r['jobs']:2d}: speed-up {r['speedup']:.2f}x, "
              f"efficiency {100 * r['efficiency']:.0f} %", flush=True)
    out = common.CHECK_DIR
    with open(out / "speedup.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    (out / "speedup.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"\nwrote {out / 'speedup.csv'}")


if __name__ == "__main__":
    main()
