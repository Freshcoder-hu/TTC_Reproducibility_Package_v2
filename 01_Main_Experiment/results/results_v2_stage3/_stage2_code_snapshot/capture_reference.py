"""Capture full-trajectory reference traces (Stage-2 acceptance test A baseline).

Run **before** any Stage-2 edit (it was run against the accepted Stage-1 code):

    python -X utf8 -m v2.capture_reference --out results_v2_stage2/reference

It stores, for three spot cells, the complete plant state after every
integration step (alive mask, sheep pos/head/speed, robot pos/head/speed/omega,
obstacle positions) as a ``.npz`` plus the summary row as JSON.  Stage 2 must
reproduce those arrays bit-for-bit with the new code in M0 mode -- a much
stronger statement than "the summary row matches", because every one of the
~1.06 M logged floats is compared.

The capture itself only *reads* the state (``record_states=True``), so the
existence of this module cannot change any number.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import compat_v1cfg, default_v2cfg  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402

# (name, scenario, seed, mode, projection)
CELLS = [
    ("S3_s200_v1compat", "S3", 200, "compat", "joint"),
    ("S3_s200_v2", "S3", 200, "v2", "joint"),
    ("S1_s201_v2", "S1", 201, "v2", "joint"),
    ("S3_s200_v2_seq", "S3", 200, "v2", "sequential"),
]


def _pack(trace):
    """List of per-tick tuples -> flat dict of arrays (float64 all through)."""
    cols = ["t", "step", "alive", "spos", "shead", "sspeed",
            "rpos", "rhead", "rspeed", "romega", "opos"]
    out = {c: np.array([row[i] for row in trace]) for i, c in enumerate(cols)}
    for c in cols:
        out[c] = np.ascontiguousarray(out[c])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results_v2_stage2/reference")
    ap.add_argument("--only", nargs="*", default=None)
    args = ap.parse_args()
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)

    for name, sid, seed, mode, proj in CELLS:
        if args.only and name not in args.only:
            continue
        v2cfg = compat_v1cfg() if mode == "compat" else default_v2cfg()
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=v2cfg,
                            compat=(mode == "compat"), projection=proj,
                            return_res=True, record_states=True)
        arr = _pack(res["trace_states"])
        np.savez_compressed(out / f"{name}.npz", **arr)
        meta = dict(cell=name, scenario=sid, seed=seed, mode=mode,
                    projection=proj, config_id=v2cfg.config_id,
                    n_ticks=len(res["trace_states"]),
                    lost=res["lost"], ended=res["terminated_by"],
                    summary={k: (None if isinstance(v, float) and not
                                 np.isfinite(v) else v)
                             for k, v in row.items()
                             if isinstance(v, (int, float, str, bool))})
        (out / f"{name}.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"{name:18s} ticks={len(res['trace_states']):5d} "
              f"config_id={v2cfg.config_id} lost={res['lost']} "
              f"end={res['terminated_by']}", flush=True)


if __name__ == "__main__":
    main()
