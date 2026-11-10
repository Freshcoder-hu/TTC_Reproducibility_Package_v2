"""Capture the Stage-2 whole-trajectory reference for Stage-3 acceptance test A.

Run **before any Stage-3 edit** (it must run against the accepted Stage-2 code):

    python -X utf8 -m v2.capture_reference_s3

It stores, for four spot cells, the complete plant state after every integration
step (alive mask, sheep pos/head/speed, robot pos/head/speed/omega, obstacle
positions) as a ``.npz`` plus the summary row as JSON.

Stage 3 must reproduce those arrays **bit-for-bit** when its sensing interface
is configured to the V1-equivalent (full visibility, exact, zero delay, no
loss) and the TTC thresholds are set to the Stage-2 values.  That is the
inertness claim (acceptance test A): the whole sensing interface can be switched
on and configured to "no degradation" and nothing moves.

The capture itself only *reads* state (``record_states=True``), so the existence
of this module cannot change any number.

Two configurations are captured:

``stage2``  the shipped Stage-2 parameter set with the **M1** animal model (the
            author-fixed main model for Stage 3) and the Stage-2 TTC tuple
            (T_PRE, T_DIVERT, T_REC, h) = (12, 6, 15, 2.5) s.  This is the
            reference every Stage-3 run must reproduce with sensing inert.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage2_v2cfg  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage3"
REF = OUT / "reference"

# Shipped Stage-2 values (results_v2_stage2/calibration_summary.json), M1 row.
SHIPPED_S2 = dict(m1_sense_range=6.0, m1_gain=3.0, m1_decay="linear",
                  m1_decay_len=2.0, m_v_max=2.0,
                  m2_react_delay=0.30, m2_gain_sigma=0.20, m2_slow_gain=0.5)

# Stage-2 TTC tuple == the V1 frozen tuple (Proposed defaults).
STAGE2_TTC = dict(t_pre=12.0, t_divert=6.0, t_rec=15.0, hyst=2.5)

CELLS = [
    ("S2_s1000", "S2", 1000),
    ("S3_s1000", "S3", 1000),
    ("S3_s1001", "S3", 1001),
    ("S1_s1002", "S1", 1002),
]

TRACE_COLS = ["t", "step", "alive", "spos", "shead", "sspeed",
              "rpos", "rhead", "rspeed", "romega", "opos"]

WATCH = ["simulation.py", "proposed.py", "proposed_event.py",
         "proposed_improved.py", "proposed_rollout.py", "robots.py",
         "sheep.py", "config.py", "corrections.py", "ttc.py", "metrics.py",
         "scenarios.py", "path.py", "obstacle.py", "run_experiments.py",
         "baselines_outmost.py", "baselines_pcbf.py", "baselines_vh.py",
         "v2/config_v2.py", "v2/sim_v2.py", "v2/controller_v2.py",
         "v2/runner_v2.py", "v2/metrics_v2.py", "v2/animal_models.py",
         "v2/animal_m0.py", "v2/predictor_v2.py", "v2/speed_policy_v2.py",
         "v2/fleet_v2.py", "v2/feasibility_v2.py", "v2/limits_v2.py",
         "v2/contacts_v2.py", "v2/corridor_v2.py", "v2/robot_safety_v2.py"]


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16].upper()


def pack(trace):
    out = {c: np.array([row[i] for row in trace]) for i, c in enumerate(
        TRACE_COLS)}
    return {c: np.ascontiguousarray(v) for c, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--no-snapshot", action="store_true")
    args = ap.parse_args()
    out = Path(args.out)
    ref = out / "reference"
    ref.mkdir(parents=True, exist_ok=True)

    if not args.no_snapshot:
        # ---- content hashes (pre-edit evidence) ----
        hashes = {p: sha16(ROOT / p) for p in WATCH}
        (out / "code_hashes_pre_stage3.json").write_text(
            json.dumps(hashes, indent=2), encoding="utf-8")
        # ---- a verbatim copy of the accepted Stage-2 v2/ tree ----
        snap = out / "_stage2_code_snapshot"
        if snap.exists():
            shutil.rmtree(snap)
        shutil.copytree(ROOT / "v2", snap,
                        ignore=shutil.ignore_patterns("__pycache__"))
        print(f"snapshot: {snap}  ({len(list(snap.glob('*.py')))} py files)",
              flush=True)

    # NOTE: no threshold fields exist yet in the Stage-2 V2Cfg; the controller
    # defaults are exactly STAGE2_TTC, which is what the reference therefore
    # uses.  Stage 3 must reproduce it with the tuple passed explicitly.
    v2cfg = stage2_v2cfg("M1", **SHIPPED_S2)
    print(f"stage2 cfg_id={v2cfg.config_id}  (thresholds {STAGE2_TTC}, "
          "controller defaults)", flush=True)
    for name, sid, seed in CELLS:
        if args.only and name not in args.only:
            continue
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=v2cfg,
                            return_res=True, record_states=True)
        arr = pack(res["trace_states"])
        np.savez_compressed(ref / f"{name}.npz", **arr)
        meta = dict(cell=name, scenario=sid, seed=seed,
                    config_id=v2cfg.config_id,
                    animal_model="M1", ttc=STAGE2_TTC,
                    shipped=SHIPPED_S2,
                    n_ticks=len(res["trace_states"]),
                    n_values=int(sum(v.size for v in arr.values())),
                    lost=res["lost"], ended=res["terminated_by"],
                    summary={k: (None if isinstance(v, float)
                                 and not np.isfinite(v) else v)
                             for k, v in row.items()
                             if isinstance(v, (int, float, str, bool))})
        (ref / f"{name}.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"{name:12s} ticks={len(res['trace_states']):5d} "
              f"values={meta['n_values']:8d} lost={res['lost']} "
              f"end={res['terminated_by']} rho={row.get('rho_avg')}",
              flush=True)


if __name__ == "__main__":
    main()
