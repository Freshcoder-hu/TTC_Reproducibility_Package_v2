"""The formal Stage-5 matrix: 8 methods x 10 scenarios x seeds 2000-2029.

    python -X utf8 -m v2.stage5.matrix --jobs 12            # all eight methods
    python -X utf8 -m v2.stage5.matrix --methods B1 B2 --limit 2

Checkpointing is **per method**: ``results_v2_stage5/matrix/<method>.csv`` is
written atomically when that method's 300 cells are done, and a re-run skips a
method whose file already has the full, complete set of 300 unique cells.  A
crash therefore loses at most the batch in flight.

A cell that raises is written as a row with NaN metrics, ``run_error`` set and
``censored = 1``; it stays in every denominator (acceptance B counts it, the
census reports it).  Nothing is dropped.

Timing: each batch's wall clock is measured and stored in
``matrix/_batches.json``; ``wall_s`` per cell is stored in the CSV.  BLAS is
pinned to one thread by ``corrections.install(environment=True)`` and by the
environment the driver sets before the pool starts.
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import registry_alg  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5.runs import HEADER, S5_KEYS, run_one  # noqa: E402

METHOD_ORDER = common.METHODS


# ---------------------------------------------------------------------------
def _write_rows(path, rows, header=HEADER):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r.get(k, np.nan) for k in header])
    tmp.replace(path)


def _complete(path, n_expected):
    """True iff ``path`` already holds ``n_expected`` unique (scene, seed) cells."""
    if not path.exists():
        return False
    try:
        with open(path, encoding="utf-8-sig") as f:
            rows = list(csv.DictReader(f))
    except (OSError, csv.Error):
        return False
    cells = {(r.get("scenario"), r.get("seed")) for r in rows}
    return len(cells) >= n_expected


def _task(t):
    return run_one(*t["args"], **t["kw"])


def run_method(method, scenes, seeds, jobs, force=False, limit=None):
    """One checkpointed batch (one method x all scenes x all seeds)."""
    path = common.MATRIX_DIR / f"{method}.csv"
    v2cfg = stage3_v2cfg("M1")
    scenes = list(scenes)
    seeds = list(seeds)
    if limit:
        scenes, seeds = scenes[:limit], seeds[:limit]
    n_expected = len(scenes) * len(seeds)
    if not force and _complete(path, n_expected):
        print(f"[{method:14s}] cached ({n_expected} cells) -> {path.name}",
              flush=True)
        return dict(method=method, cached=True, n=len(scenes) * len(seeds),
                    wall_s=0.0)
    registry_alg(method)
    tasks = [dict(args=(sid, method, sd), kw=dict(v2cfg=v2cfg))
             for sid in scenes for sd in seeds]
    t0 = time.perf_counter()
    done = 0
    rows = []
    print(f"[{method:14s}] {len(tasks)} cells, jobs={jobs}", flush=True)
    if jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            for row in ex.map(_task, tasks, chunksize=1):
                rows.append(row)
                done += 1
                if done % 25 == 0 or done == len(tasks):
                    el = time.perf_counter() - t0
                    print(f"[{method:14s}] {done:3d}/{len(tasks)}  "
                          f"{el:7.1f} s  ({el / done:5.2f} s/cell)", flush=True)
    else:
        for t in tasks:
            rows.append(_task(t))
            done += 1
            if done % 25 == 0 or done == len(tasks):
                el = time.perf_counter() - t0
                print(f"[{method:14s}] {done:3d}/{len(tasks)}  {el:7.1f} s",
                      flush=True)
    wall = time.perf_counter() - t0
    order = {s: i for i, s in enumerate(scenes)}
    rows.sort(key=lambda r: (order.get(r.get("scenario"), 99),
                             int(r.get("seed") or 0)))
    _write_rows(path, rows)
    n_err = sum(1 for r in rows if str(r.get("run_error") or "").strip())
    n_cens = sum(1 for r in rows if int(r.get("censored") or 0))
    batch = dict(method=method, cached=False, n=len(rows), wall_s=wall,
                 s_per_cell=wall / max(len(rows), 1), jobs=jobs,
                 n_error=n_err, n_censored=n_cens,
                 per_cell_wall_mean=float(np.nanmean(
                     [float(r.get("wall_s") or np.nan) for r in rows]))
                 if rows else np.nan,
                 per_cell_wall_sd=float(np.nanstd(
                     [float(r.get("wall_s") or np.nan) for r in rows]))
                 if rows else np.nan)
    print(f"[{method:14s}] DONE {len(rows)} rows in {wall:.1f} s "
          f"({wall / max(len(rows), 1):.2f} s/cell), errors={n_err}, "
          f"censored={n_cens} -> {path.name}", flush=True)
    return batch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--methods", nargs="+", default=METHOD_ORDER)
    ap.add_argument("--scenes", nargs="+", default=common.SCENARIOS)
    ap.add_argument("--seeds", nargs="+", type=int, default=common.SEEDS)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--limit", type=int, default=None,
                    help="smoke mode: first N scenes x first N seeds")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--tag", default="matrix")
    args = ap.parse_args()
    common.ensure_dirs()
    v2cfg = stage3_v2cfg("M1")
    print(f"Stage-5 matrix  config_id={v2cfg.config_id}  "
          f"methods={len(args.methods)} scenes={len(args.scenes)} "
          f"seeds={args.seeds[0]}..{args.seeds[-1]} jobs={args.jobs}",
          flush=True)
    t0 = time.time()
    batches = []
    for m in args.methods:
        batches.append(run_method(m, args.scenes, args.seeds, args.jobs,
                                  force=args.force, limit=args.limit))
    total = time.time() - t0
    manifest = dict(config_id=v2cfg.config_id, methods=args.methods,
                    scenes=args.scenes, seeds=args.seeds, jobs=args.jobs,
                    limit=args.limit, total_wall_s=total, batches=batches,
                    python=sys.version.split()[0], numpy=np.__version__,
                    n_rows=sum(b["n"] for b in batches),
                    code_hashes=common.code_hashes(
                        sorted((ROOT / "v2").glob("*.py")) +
                        sorted((ROOT / "v2" / "stage5").glob("*.py")) +
                        [ROOT / p for p in
                         ("simulation.py", "metrics.py", "proposed.py",
                          "proposed_event.py", "proposed_improved.py",
                          "proposed_rollout.py", "proposed_rollout_ablation.py",
                          "proposed_improved.py", "robots.py", "sheep.py",
                          "ttc.py", "config.py", "scenarios.py",
                          "run_experiments.py", "run_revised_experiments.py",
                          "baselines_vh.py", "baselines_outmost.py",
                          "baselines_pcbf.py", "corrections.py")]))
    common.write_json(common.MATRIX_DIR / f"_{args.tag}_batches.json", manifest)
    tot = sum(b["wall_s"] for b in batches if not b.get("cached"))
    print(f"\nTOTAL {sum(b['n'] for b in batches)} rows in {total:.1f} s "
          f"({total / 3600:.2f} h); new-batch wall {tot:.1f} s", flush=True)


if __name__ == "__main__":
    main()
