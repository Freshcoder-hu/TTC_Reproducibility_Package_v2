"""Shared paths and metric definitions for the V7 paper build.

Read-only on the experiment archive: nothing in this package writes to
the experiment archive or any ``results_*`` directory.
"""
from __future__ import annotations

import os
from pathlib import Path

# package root = three levels up from paper/scripts/
_PKG = Path(__file__).resolve().parents[3]

# --- archive (frozen, read-only) -------------------------------------------
CODE = str(_PKG / "01_Main_Experiment" / "code")
RESULTS = os.path.join(os.path.dirname(CODE), "results")
S5 = os.path.join(RESULTS, "results_v2_stage5")
S4 = os.path.join(RESULTS, "results_v2_stage4")
S3 = os.path.join(RESULTS, "results_v2_stage3")

# --- paper tree -------------------------------------------------------------
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
TABLES = os.path.join(ROOT, "tables")
FIGURES = os.path.join(ROOT, "figures")

METHODS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger",
           "VH-Danger-path", "B-Outmost", "B-PCBF"]

# display labels (Elsevier style, no underscores)
LABEL = {
    "Ours_rollout": "Ours",
    "B1": "B1",
    "B2": "B2",
    "VH-Path": "VH-Path",
    "VH-Danger": "VH-Danger",
    "VH-Danger-path": "VH-Danger-path",
    "B-Outmost": "B-Outmost",
    "B-PCBF": "B-PCBF",
}

SCENARIOS = ["V2S1", "V2S1N", "V2S2", "V2S2R3", "V2S2R2", "V2S2R1",
             "V2S3", "V2S4S", "V2S4M", "V2S4L"]
RISK = ["V2S2", "V2S2R3", "V2S2R2", "V2S2R1", "V2S3", "V2S4S", "V2S4M",
        "V2S4L"]

# endpoint -> column; direction +1 = higher is better
ENDPOINTS = {
    "C": ("collision_rate", -1),
    "clr_so": ("min_surface_clearance_so", +1),
    "ttc_min": ("ttc_min", +1),
    "rho": ("rho_avg", +1),
    "devW": ("max_dev_norm", -1),
    "ctrl_ms": ("ctrl_ms", -1),
    "exec_time": ("exec_time", -1),
}
EP_LABEL = {
    "C": "Contact fraction $C$",
    "clr_so": "Min. surface clearance [m]",
    "ttc_min": "Min. TTC [s]",
    "rho": "Path keeping $\\rho$",
    "devW": "max dev/$W$",
    "ctrl_ms": "Cycle time [ms]",
    "exec_time": "Time to goal [s]",
}

SEEDS_EVAL = list(range(2000, 2030))
SEEDS_CAL = list(range(1000, 1020))
CONFIG_ID = "89fe865b2e1d"
