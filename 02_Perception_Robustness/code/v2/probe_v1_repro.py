"""Probe: does the *unmodified* V1 tree reproduce the archived S3 cell bit-exactly?

Run from the code root:
    python -m v2.probe_v1_repro            # S3 / Ours_rollout / seed 200
    python -m v2.probe_v1_repro --scn S1 --seed 200

The archived per-method detail tables live in
`results_cea_matrix/_parts/<alg>/detail.csv`.  This probe re-runs one cell
through the frozen V1 code path (`simulation.run_scene` + `RolloutProposed`
exactly as `run_revised_experiments.py` / `run_experiments.run_task` do) and
compares **every non-timing column** bit-for-bit.

Stage 1 acceptance test A is only meaningful if this probe passes: it proves
the environment (Python 3.13.7 / numpy 2.5.2 / _rollout.dll / BLAS threading)
still reproduces the archive, so any later difference can be attributed to the
V2 layer rather than to the interpreter.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402  (must be installed exactly as the runner does)

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from scenarios import SCENARIOS  # noqa: E402
from simulation import run_scene  # noqa: E402
from metrics import summarize  # noqa: E402

from run_experiments import ALGS, KEYS  # noqa: E402
import run_revised_experiments  # noqa: E402,F401  (registers ALGS['Ours_rollout'])

# 墙钟计时列不参与逐位比较
TIMING_KEYS = ("ctrl_ms",)


def archived_row(path_csv, scenario, alg, seed):
    with open(path_csv, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if (row["scenario"] == scenario and row["alg"] == alg
                    and int(row["seed"]) == seed):
                return row
    raise KeyError(f"{scenario}/{alg}/{seed} not in {path_csv}")


def as_float(txt):
    if isinstance(txt, (int, float, np.floating, np.integer, bool)):
        return float(txt)
    t = (txt or "").strip()
    if t == "" or t.lower() == "nan":
        return math.nan
    if t.lower() == "inf":
        return math.inf
    if t.lower() == "-inf":
        return -math.inf
    if t in ("True", "true"):
        return 1.0
    if t in ("False", "false"):
        return 0.0
    return float(t)


def same(a, b):
    """Bit-exact comparison: numeric columns compare as doubles (NaN==NaN),
    non-numeric columns (rr_min_tag / rr_min_pair) compare as strings."""
    if isinstance(a, str) and not _is_number(a):
        return str(a) == str(b)
    fa, fb = as_float(a), as_float(b)
    if math.isnan(fa) or math.isnan(fb):
        return math.isnan(fa) and math.isnan(fb)
    if math.isinf(fa) or math.isinf(fb):
        return fa == fb
    return fa == fb


def _is_number(txt):
    try:
        as_float(txt)
        return True
    except (TypeError, ValueError):
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scn", default="S3")
    ap.add_argument("--alg", default="Ours_rollout")
    ap.add_argument("--seed", type=int, default=200)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    csv_p = Path(args.csv) if args.csv else \
        ROOT / "results_cea_matrix" / "_parts" / args.alg / "detail.csv"
    arc = archived_row(csv_p, args.scn, args.alg, args.seed)

    scn = SCENARIOS[args.scn]()
    ctrl = ALGS[args.alg](**scn.get("params", {}))
    res = run_scene(scn["cfg"], scn["path"], scn["obstacles"], ctrl,
                    seed=args.seed, T_max=scn["T_max"], s0=scn["s0"])
    m = summarize(res)

    bad = []
    for k in KEYS:
        if k in TIMING_KEYS:
            continue
        got = m.get(k, math.nan)
        if not same(got, arc[k]):
            bad.append((k, arc[k], got))
    print(f"cell = {args.scn}/{args.alg}/seed {args.seed}")
    print(f"columns compared (non-timing) = {len(KEYS) - len(TIMING_KEYS)}")
    if bad:
        print(f"BIT-EXACT: NO  ({len(bad)} mismatch)")
        for k, a, g in bad:
            print(f"   {k:20s} archived={a!r:24s} rerun={g!r}")
        return 1
    print("BIT-EXACT: YES  (all non-timing columns identical)")
    print(f"   (archived ctrl_ms={arc['ctrl_ms']}, rerun ctrl_ms={m['ctrl_ms']:.3f}"
          f" -- timing column, excluded by design)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
