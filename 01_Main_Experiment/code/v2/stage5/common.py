"""Frozen Stage-5 definitions: methods, scenes, seeds, endpoints, paths.

The configuration is the Stage-3/4 frozen record **unchanged**
(``stage3_v2cfg("M1")`` -> ``config_id = 89fe865b2e1d``): r_robot 0.45,
a_lin 4.0, a_lat 2.0, v_max 4.5 with the layered speed policy in pre/divert,
d_rr_safe 2.0, 36:12, M1 animal (linear decay k = 3.0), animal 25 m / obstacle
70 m range with the quality knobs off, TTC tuple (12, 8, 15, 3) s, W = 8 m
half-width, horizons 200 s / 320 s.  Stage 5 changes **no** parameter; the only
code-side additions are measurement columns.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent          # <pkg>/01_Main_Experiment/code
# The Stage-5 archive lives beside ``code/`` in this package, but directly under
# ``code/`` in the original build tree.  Accept either, preferring the shipped one.
_OUT_CANDIDATES = (ROOT.parent / "results" / "results_v2_stage5",
                   ROOT / "results_v2_stage5")
OUT = next((p for p in _OUT_CANDIDATES if p.is_dir()), _OUT_CANDIDATES[0])
MATRIX_DIR = OUT / "matrix"
ABL_DIR = OUT / "ablation"
TREC_DIR = OUT / "trec_sweep"
C1_DIR = OUT / "c1_audit"
CHECK_DIR = OUT / "checks"
REPRO_DIR = OUT / "reproduced"

# ---------------------------------------------------------------------------
# the formal design
# ---------------------------------------------------------------------------
METHODS = ["Ours_rollout",
           "B1", "B2",
           "VH-Path", "VH-Danger", "VH-Danger-path",
           "B-Outmost", "B-PCBF"]

# fixed order = the order of every table in the report
SCENARIOS = ["V2S1", "V2S1N",
             "V2S2", "V2S2R3", "V2S2R2", "V2S2R1",
             "V2S3",
             "V2S4S", "V2S4M", "V2S4L"]

# scenarios that carry an obstacle: the only ones on which
# clearance / TTC / preemption are *defined*
OBSTACLE_SCENARIOS = ["V2S2", "V2S2R3", "V2S2R2", "V2S2R1",
                      "V2S3", "V2S4S", "V2S4M", "V2S4L"]

# the risk sub-family: single-threat risk levels + the sequential scene.
# Equal weight over these five is the "risk-scenario equal weight" column set.
RISK_SCENARIOS = ["V2S2", "V2S2R3", "V2S2R2", "V2S2R1", "V2S3"]

SEEDS = list(range(2000, 2030))          # evaluation set, untouched by Stages 1-4
CALIB_SEEDS = list(range(1000, 1005))    # calibration set (T_REC sweep, C1 audit)

W_HALF = 8.0                             # half-width [m]; max_dev/W uses W = 8

# ---------------------------------------------------------------------------
# ablation ladder (the four rungs the manuscript claims)
# ---------------------------------------------------------------------------
ABLATION_SCENARIOS = ["V2S2R2", "V2S3"]  # the V2 equivalents of S2R2 / S3
ABLATION_RUNGS = ["R0_four_state", "R1_handover", "R2_event", "R3_rollout"]
ABLATION_LABELS = {
    "R0_four_state": "R0 four-state only (no handover, no event, no rollout)",
    "R1_handover": "R1 + handover of the avoidance slot",
    "R2_event": "R2 + event layer (= 'w/o rollout', Ours_event)",
    "R3_rollout": "R3 + rollout planner (= Ours_rollout, the shipped method)",
}

# ---------------------------------------------------------------------------
# endpoints
# ---------------------------------------------------------------------------
# ``scenes``: "all" = the 10-scenario equal-weight average; "obstacle" = the 8
# obstacle-bearing scenarios (clearance / TTC are undefined, not zero, on the
# two obstacle-free scenes, so they are averaged over the 8 where they exist).
ENDPOINTS = [
    dict(key="C", col="collision_rate", label="C", nd=3,
         better="lower", scenes="all",
         desc="contact fraction (animals lost / n0)"),
    dict(key="clr_so", col="min_surface_clearance_so", label="min clr_so [m]",
         nd=2, better="higher", scenes="obstacle",
         desc="worst animal-obstacle surface clearance"),
    dict(key="ttc_min", col="ttc_min", label="min TTC [s]", nd=2,
         better="higher", scenes="obstacle",
         desc="worst animal-obstacle time-to-collision"),
    dict(key="rho", col="rho_avg", label="rho", nd=3, better="higher",
         scenes="all", desc="mean path-keeping / cohesion ratio"),
    dict(key="devW", col="max_dev_norm", label="max_dev/W", nd=3,
         better="lower", scenes="all",
         desc="worst lateral path deviation / W"),
    dict(key="ctrl_ms", col="ctrl_ms", label="cycle [ms]", nd=2, better="lower",
         scenes="all", desc="controller wall clock per control cycle"),
    dict(key="exec_time", col="exec_time", label="goal time [s]", nd=1,
         better="lower", scenes="all", censored=True,
         desc="time to the success criterion (NaN = right-censored)"),
]

# binary endpoints -> exact McNemar, never Wilcoxon
BINARY_ENDPOINTS = [
    dict(key="arrival", col="arrival", label="arrival",
         desc="reached the goal before the horizon"),
    dict(key="joint", col="joint_completion", label="joint completion",
         desc="Nguyen completion (d_goal <= 2 m and spread <= 12 m)"),
]

# endpoints reported in the main table but *not* tested: a method without a
# state machine has no switch_count, so the column is n/a for it and a paired
# test would silently drop most of the matrix.
DESCRIPTIVE_ONLY = ["switch_count", "n_infeasible", "n_infeasible_S",
                    "n_infeasible_P", "n_contact_C", "preempt_ctrl"]

#: the only wall-clock columns.  Everything else (including ``exec_time``,
#: ``sim_time_s`` and ``n_ctrl_ticks``) is deterministic and must match
#: bit-for-bit in the inertness and determinism checks.
TIMING_COLS = ["ctrl_ms", "wall_s"]


def endpoint_cols(scenes="all"):
    """``[(key, col, better, nd, label)]`` for the requested scene weighting."""
    out = []
    for e in ENDPOINTS:
        if scenes == "obstacle" and e["scenes"] != "obstacle":
            continue
        if scenes == "all" and e["scenes"] == "obstacle":
            continue
        out.append(e)
    return out


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def sha16(path):
    """Same digest the Stage-4 acceptance snapshot uses."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16].upper()


def code_hashes(files):
    return {Path(f).name: sha16(f) for f in files if Path(f).exists()}


def ensure_dirs():
    for d in (OUT, MATRIX_DIR, ABL_DIR, TREC_DIR, C1_DIR, CHECK_DIR, REPRO_DIR):
        d.mkdir(parents=True, exist_ok=True)


def write_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, ensure_ascii=False, default=float),
                   encoding="utf-8")
    tmp.replace(path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))
