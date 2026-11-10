#!/usr/bin/env python3
"""Recompute the published headline numbers from the shipped records.

Everything here is read-only and standard-library only.  Run from anywhere:

    python tools/spotcheck.py

It prints the graphical-abstract numbers and the three headline counts and
exits non-zero if any of them disagrees with the values printed in the paper.

Graphical abstract (s5.tex lines 13-15 and 44-46):
    contacted-animal fraction, ``version=safe`` / ``factor=exact``,
    60 runs per method, two scenes pooled:
        Ours_rollout     "Proposed"          0.05463   54/60 joint completion
        B1               "Path-only"         0.248611  51/60
        B2               "Distance-reactive" 0.244444  49/60
Headline counts:
        2400 main runs          8 methods x 300 seeds (2000-2029 x 10 scenes)
        2340 robustness cells   1980 perception + 360 exact-observation control
        24   timing runs        3 methods x 2 scenes x 4 observation conditions
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FORMAL = ROOT / "02_Perception_Robustness" / "results" / "formal.jsonl"
MATRIX = ROOT / "01_Main_Experiment" / "results" / "results_v2_stage5" / "matrix"
TIMING = (ROOT / "03_Online_Warning_and_Timing" / "analysis" /
          "online_measurement")

#: method id -> the label used in the manuscript
LABEL = {"Ours_rollout": "Proposed", "B1": "Path-only", "B2": "Distance-reactive"}

#: (method, expected contacted-animal fraction, expected joint completions)
EXPECTED = [
    ("Ours_rollout", 0.05462962962962963, 54),
    ("B1",           0.24861111111111112, 51),
    ("B2",           0.24444444444444446, 49),
]
N_PER_METHOD = 60

#: reported to 5 / 6 / 6 decimals in the graphical abstract
ROUNDED = {"Ours_rollout": 0.05463, "B1": 0.248611, "B2": 0.244444}

fails = []


def check(ok, message):
    print(("  [PASS] " if ok else "  [FAIL] ") + message)
    if not ok:
        fails.append(message)


def load_jsonl(path):
    rows = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def mean(values):
    return sum(values) / len(values)


def contacted_fraction():
    print("1. Graphical abstract - contacted-animal fraction (version=safe, factor=exact)")
    rows = load_jsonl(FORMAL)
    sub = [r for r in rows if r.get("version") == "safe" and r.get("factor") == "exact"]
    print(f"   source: {FORMAL.relative_to(ROOT)}")
    print(f"   rows in file           : {len(rows)}")
    print(f"   version=safe,factor=exact : {len(sub)}  (3 methods x {N_PER_METHOD})")
    check(len(sub) == 3 * N_PER_METHOD, f"safe/exact population is 3 x {N_PER_METHOD} = {3 * N_PER_METHOD}")

    for method, want_fraction, want_joint in EXPECTED:
        rows_m = [r for r in sub if r["method"] == method]
        fractions = [r["collision_rate"] for r in rows_m]
        got = mean(fractions)
        joint = sum(1 for r in rows_m if r.get("joint_completion") is True)
        print(f"   {method:<13s} ({LABEL[method]:<18s}) n={len(rows_m)}")
        print(f"      sum(collision_rate) = {sum(fractions):.12f}   "
              f"mean = {got:.12f}   ->  {ROUNDED[method]}")
        print(f"      joint_completion    = {joint}/{len(rows_m)}")
        check(len(rows_m) == N_PER_METHOD, f"{method}: {N_PER_METHOD} runs")
        check(abs(got - want_fraction) < 1e-12,
              f"{method}: mean collision_rate {got:.12f} == {want_fraction:.12f}")
        check(abs(round(got, 5 if method == "Ours_rollout" else 6) - ROUNDED[method]) < 1e-15,
              f"{method}: rounds to {ROUNDED[method]} as printed")
        check(joint == want_joint, f"{method}: {joint}/{len(rows_m)} joint completions == {want_joint}/60")


def headline_counts():
    print("\n2. Headline counts")
    # -- 2400 main runs -----------------------------------------------------
    files = sorted(MATRIX.glob("*.csv"))
    per_method = {}
    for path in files:
        with path.open(encoding="utf-8-sig", newline="") as fh:
            per_method[path.stem] = sum(1 for _ in csv.DictReader(fh))
    total = sum(per_method.values())
    print(f"   main matrix: {len(files)} method files, rows per method "
          f"{sorted(set(per_method.values()))}, total {total}")
    check(len(files) == 8, f"8 method files (got {len(files)})")
    check(all(n == 300 for n in per_method.values()), "every method has 300 runs")
    check(total == 2400, f"2400 main runs (got {total})")

    # -- 2340 robustness cells ---------------------------------------------
    rows = load_jsonl(FORMAL)
    perception = sum(1 for r in rows if r.get("version") == "safe")
    control = sum(1 for r in rows if r.get("version") in ("archive", "strict"))
    print(f"   robustness: {len(rows)} cells = {perception} perception + "
          f"{control} exact-observation control")
    check(len(rows) == 2340, f"2340 robustness cells (got {len(rows)})")
    check(perception == 1980, f"1980 perception cells (got {perception})")
    check(control == 360, f"360 exact-observation control cells (got {control})")
    check(perception + control == len(rows), "perception + control partitions the file")

    # -- 24 timing runs -----------------------------------------------------
    timing_rows = load_jsonl(TIMING / "timing_runs.jsonl")
    traces = sorted((TIMING / "timing_traces").glob("*.npz"))
    with (TIMING / "summary.json").open(encoding="utf-8") as fh:
        summary = json.load(fh)
    print(f"   timing: {len(timing_rows)} serial runs, {len(traces)} timing traces, "
          f"summary.serial_timing_runs={summary.get('serial_timing_runs')}")
    check(len(timing_rows) == 24, f"24 serial timing runs (got {len(timing_rows)})")
    check(len(traces) == 24, f"24 timing traces (got {len(traces)})")
    check(summary.get("serial_timing_runs") == 24, "summary.json agrees on 24")


def main():
    print("TTC shepherding - shipped-record spot-check")
    print("=" * 72)
    contacted_fraction()
    headline_counts()
    print("\n" + "=" * 72)
    if fails:
        print(f"RESULT: FAIL ({len(fails)} check(s))")
        for f in fails:
            print("   -", f)
        return 1
    print("RESULT: PASS - all graphical-abstract numbers and headline counts "
          "reproduce from the shipped records")
    return 0


if __name__ == "__main__":
    sys.exit(main())
