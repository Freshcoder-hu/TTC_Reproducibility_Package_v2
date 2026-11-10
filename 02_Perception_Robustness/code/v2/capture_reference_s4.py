"""Capture the **Stage-3** whole-trajectory reference before any Stage-4 edit.

Run this **first** (it must run against the accepted Stage-3 code):

    python -X utf8 -m v2.capture_reference_s4

It writes into a *new* directory (``results_v2_stage4/``) only; nothing that
existed before Stage 4 is written to.  What it records:

``reference/stage3_<cell>.npz|json``
    Whole-plant state after every integration step for a spot cell of the
    **shipped Stage-3 configuration** (M1, sensing 25 m / 70 m, frozen tuple
    (12, 8, 15, 3) s) on a **V1 scene definition**.  Stage-4 acceptance test A
    re-runs exactly that cell through the *Stage-4 runner* and demands bit
    equality: the new geometry machinery must be inert.

``reference/v1eq_<cell>.npz|json``
    The same cell with the sensing interface switched to the V1-equivalent and
    the Stage-2 TTC tuple.  This is the arm the Stage-3 archive already holds
    (``results_v2_stage3/reference/``); it is captured again here so that test A
    can compare Stage-4 against *both* a fresh pre-edit capture and the Stage-3
    archive (the latter read-only).

``code_hashes_pre_stage4.json`` + ``_stage3_code_snapshot/``
    SHA-256 (first 16 hex) of every V1 file and every ``v2/`` file, and a
    verbatim copy of the accepted Stage-3 ``v2/`` tree, so the "nothing that
    existed moved" claim is checkable after the fact.
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

from v2.config_v2 import STAGE2_TTC, stage3_v2cfg  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage4"
REF = OUT / "reference"

TRACE_COLS = ["t", "step", "alive", "spos", "shead", "sspeed",
              "rpos", "rhead", "rspeed", "romega", "opos"]

# V1 files: must stay byte-identical (Stage-1/2/3 invariant).
V1_WATCH = ["simulation.py", "proposed.py", "proposed_event.py",
            "proposed_improved.py", "proposed_rollout.py", "robots.py",
            "sheep.py", "config.py", "corrections.py", "ttc.py", "metrics.py",
            "scenarios.py", "path.py", "obstacle.py", "run_experiments.py",
            "run_revised_experiments.py", "baselines_outmost.py",
            "baselines_pcbf.py", "baselines_vh.py", "proposed_rollout_ablation.py"]

# V2 files accepted at the end of Stage 3 (must not be edited by Stage 4).
V2_WATCH = ["v2/config_v2.py", "v2/sim_v2.py", "v2/runner_v2.py",
            "v2/metrics_v2.py", "v2/controller_v2.py", "v2/sensing_v2.py",
            "v2/observability_v2.py", "v2/calibrate_ttc_v2.py",
            "v2/animal_models.py", "v2/animal_m0.py", "v2/predictor_v2.py",
            "v2/speed_policy_v2.py", "v2/fleet_v2.py", "v2/feasibility_v2.py",
            "v2/limits_v2.py", "v2/contacts_v2.py", "v2/corridor_v2.py",
            "v2/robot_safety_v2.py", "v2/acceptance_s3.py",
            "v2/capture_reference_s3.py", "v2/plots_s3.py",
            "v2/verify_report_s3.py", "v2/regress_s2_readonly.py"]

# shipped Stage-3 interface (exact quality, 25 m / 70 m, frozen tuple)
S3_SHIPPED = dict(sense_mode="range", sense_r_obs=70.0, sense_r_animal=25.0,
                  sense_err=0.0, sense_delay=0.0, sense_loss=0.0,
                  sense_ttl=0.30)
# V1-equivalent interface + Stage-2 tuple (the Stage-3 test-A arm)
V1_SENSE = dict(sense_mode="range", sense_r_obs=1e9, sense_r_animal=10.0,
                sense_err=0.0, sense_delay=0.0, sense_loss=0.0, sense_ttl=0.0)

CELLS = [
    # name, V1 scenario id, seed, config
    ("stage3_S3_s1000", "S3", 1000, dict(**S3_SHIPPED)),
    ("stage3_S2_s1000", "S2", 1000, dict(**S3_SHIPPED)),
    ("stage3_S1_s1002", "S1", 1002, dict(**S3_SHIPPED)),
    ("v1eq_S3_s1000", "S3", 1000, dict(**V1_SENSE, **STAGE2_TTC)),
]


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
        hashes = {"v1": {p: sha16(ROOT / p) for p in V1_WATCH},
                  "v2": {p: sha16(ROOT / p) for p in V2_WATCH}}
        (out / "code_hashes_pre_stage4.json").write_text(
            json.dumps(hashes, indent=2), encoding="utf-8")
        snap = out / "_stage3_code_snapshot"
        if snap.exists():
            shutil.rmtree(snap)
        shutil.copytree(ROOT / "v2", snap,
                        ignore=shutil.ignore_patterns("__pycache__"))
        print(f"snapshot: {snap}  ({len(list(snap.glob('*.py')))} py files)",
              flush=True)

    for name, sid, seed, kw in CELLS:
        if args.only and name not in args.only:
            continue
        v2cfg = stage3_v2cfg("M1", **kw)
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=v2cfg,
                            return_res=True, record_states=True)
        arr = pack(res["trace_states"])
        np.savez_compressed(ref / f"{name}.npz", **arr)
        meta = dict(cell=name, scenario=sid, seed=seed,
                    config_id=v2cfg.config_id, animal_model="M1",
                    overrides=kw, n_ticks=len(res["trace_states"]),
                    n_values=int(sum(v.size for v in arr.values())),
                    lost=res["lost"], ended=res["terminated_by"],
                    summary={k: (None if isinstance(v, float)
                                 and not np.isfinite(v) else v)
                             for k, v in row.items()
                             if isinstance(v, (int, float, str, bool))})
        (ref / f"{name}.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"{name:18s} ticks={len(res['trace_states']):5d} "
              f"values={meta['n_values']:8d} lost={res['lost']} "
              f"end={res['terminated_by']} rho={row.get('rho_avg')}",
              flush=True)


if __name__ == "__main__":
    main()
