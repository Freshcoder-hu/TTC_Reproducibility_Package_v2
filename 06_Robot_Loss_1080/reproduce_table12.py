#!/usr/bin/env python3
"""Reproduce the Table 12 / Fig. 9 key numbers from ``detail.csv``.

Everything here is read-only and standard-library only.  Run from the package
root (or anywhere):

    python 06_Robot_Loss_1080/reproduce_table12.py

It recomputes, from the 1,080 per-run rows and nothing else:

  * the animals-with-obstacle-contact fraction in V2S3 for the proposed
    controller, F0 -> F3: 1.67 % -> 3.15 % -> 3.24 % -> 6.94 %, which is the
    "rising from 1.67 % to ... 6.94 %" statement behind Fig. 9(a) and the
    V2S3 rows of Table 12;
  * the goal-and-cohesion completion of the same 12 cells (76.7 / 73.3 /
    73.3 / 70.0 %);
  * the 1080/1080 arrival count, with no timeout and no censored run.

Expected values are those printed in the submitted article (s5.tex, Table 12
and the Fig. 9 caption).  Exit code is non-zero if any check fails.
"""
from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DETAIL = HERE / "detail.csv"
TABLE = HERE / "table_failure.csv"

METHODS = ["Ours_rollout", "B1", "B2"]
SCENES = ["V2S1", "V2S2", "V2S3"]
CONDITIONS = ["F0", "F1", "F2", "F3"]

# Table 12, Ours: GC [%] and obstacle contact [%] by scene x condition.
EXPECTED_GC = {
    "V2S1": {"F0": "86.7", "F1": "83.3", "F2": "76.7", "F3": "83.3"},
    "V2S2": {"F0": "90.0", "F1": "86.7", "F2": "80.0", "F3": "86.7"},
    "V2S3": {"F0": "76.7", "F1": "73.3", "F2": "73.3", "F3": "70.0"},
}
EXPECTED_CONTACT = {
    "V2S1": {"F0": "0.00", "F1": "0.00", "F2": "0.00", "F3": "0.00"},
    "V2S2": {"F0": "0.56", "F1": "0.56", "F2": "0.56", "F3": "0.46"},
    "V2S3": {"F0": "1.67", "F1": "3.15", "F2": "3.24", "F3": "6.94"},
}

fails = []


def check(ok, message):
    print(("  [PASS] " if ok else "  [FAIL] ") + message)
    if not ok:
        fails.append(message)


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def num(value):
    return float((value or "").strip())


def is_true(value):
    return (value or "").strip() in ("1", "True", "true")


def cells(rows, method):
    out = {}
    for scene in SCENES:
        for cond in CONDITIONS:
            out[(scene, cond)] = [
                r for r in rows
                if r["method"] == method and r["scenario"] == scene
                and r["condition"] == cond
            ]
    return out


def main():
    rows = read_csv(DETAIL)
    print("Table 12 / Fig. 9 - robot-loss panel")
    print(f"   source: {DETAIL.relative_to(HERE.parent).as_posix()}")
    print(f"   rows in file: {len(rows)}  (columns: {len(rows[0])})")

    print("\n1. Design")
    methods = sorted({r["method"] for r in rows})
    scenes = sorted({r["scenario"] for r in rows})
    conditions = sorted({r["condition"] for r in rows})
    seeds = sorted({int(r["seed"]) for r in rows})
    check(len(rows) == 1080, f"1080 runs (got {len(rows)})")
    check(methods == sorted(METHODS), f"3 methods {methods}")
    check(scenes == SCENES, f"3 scenes {scenes}")
    check(conditions == CONDITIONS, f"4 conditions {conditions}")
    check(seeds == list(range(2000, 2030)),
          f"seeds 2000-2029 (got {seeds[0]}-{seeds[-1]})")
    grid = {(r["method"], r["scenario"], r["condition"]) for r in rows}
    check(len(grid) == 36, f"36 cells = 3 x 3 x 4 (got {len(grid)})")
    check(all(sum(1 for r in rows
                  if (r["method"], r["scenario"], r["condition"]) == cell) == 30
              for cell in grid), "30 paired seeds in every cell")
    f3 = [r for r in rows if r["scenario"] == "V2S1" and r["condition"] == "F3"]
    check(all(r.get("f3_applicable", "1").strip() == "0" for r in f3),
          "V2S1-F3 is flagged f3_applicable = 0 (it repeats the F1 schedule)")

    print("\n2. Arrival and censoring")
    arrived = sum(1 for r in rows if is_true(r["arrival"]))
    timed_out = sum(1 for r in rows if is_true(r["timeout"]))
    censored = sum(1 for r in rows if is_true(r["censored"]))
    print(f"   arrival: {arrived}/{len(rows)}   timeout: {timed_out}   censored: {censored}")
    check(arrived == 1080, f"1080/1080 arrived (got {arrived})")
    check(timed_out == 0, f"no timeout (got {timed_out})")
    check(censored == 0, f"no right-censored run (got {censored})")

    print("\n3. Fig. 9(a): animals with obstacle contact, Ours, by scene x condition [%]")
    per = cells(rows, "Ours_rollout")
    print("   scene   F0     F1     F2     F3")
    for scene in SCENES:
        values = {cond: 100.0 * statistics.mean(
            num(r["collision_rate"]) for r in per[(scene, cond)])
            for cond in CONDITIONS}
        print(f"   {scene:<6s}  " + "  ".join(f"{values[c]:>5.2f}" for c in CONDITIONS))
        for cond in CONDITIONS:
            check(f"{values[cond]:.2f}" == EXPECTED_CONTACT[scene][cond],
                  f"{scene}-{cond}: obstacle contact {values[cond]:.2f} % "
                  f"== {EXPECTED_CONTACT[scene][cond]} %")
    v2s3 = {cond: 100.0 * statistics.mean(
        num(r["collision_rate"]) for r in per[("V2S3", cond)])
        for cond in CONDITIONS}
    check(f"{v2s3['F0']:.2f}" == "1.67" and f"{v2s3['F3']:.2f}" == "6.94",
          "V2S3 rises from 1.67 % (F0) to 6.94 % (F3) across F0 -> F3")
    print(f"   V2S3: 1.67 % -> 3.15 % -> 3.24 % -> 6.94 % across F0 -> F3 "
          f"(got {v2s3['F0']:.2f} -> {v2s3['F3']:.2f})")

    print("\n4. Table 12, Ours: goal-and-cohesion completion [%]")
    print("   scene   F0     F1     F2     F3")
    for scene in SCENES:
        values = {}
        for cond in CONDITIONS:
            sub = per[(scene, cond)]
            values[cond] = 100.0 * sum(1 for r in sub if is_true(r["joint_completion"])) / len(sub)
        print(f"   {scene:<6s}  " + "  ".join(f"{values[c]:>5.1f}" for c in CONDITIONS))
        for cond in CONDITIONS:
            check(f"{values[cond]:.1f}" == EXPECTED_GC[scene][cond],
                  f"{scene}-{cond}: GC {values[cond]:.1f} % "
                  f"== {EXPECTED_GC[scene][cond]} %")

    print("\n5. Cross-check against the shipped table_failure.csv")
    table = {(r["scene"], r["condition"]): r for r in read_csv(TABLE)
             if r["method"] == "Ours_rollout"}
    check(len(table) == 12, f"12 Ours rows in table_failure.csv (got {len(table)})")
    for scene in SCENES:
        for cond in CONDITIONS:
            sub = per[(scene, cond)]
            row = table[(scene, cond)]
            contact = 100.0 * statistics.mean(num(r["collision_rate"]) for r in sub)
            gc = 100.0 * sum(1 for r in sub if is_true(r["joint_completion"])) / len(sub)
            same = (abs(contact - float(row["animals_lost_pct"])) < 1e-9
                    and abs(gc - 100.0 * float(row["joint"])) < 1e-9
                    and abs(100.0 * float(row["arrival"]) - 100.0) < 1e-9)
            check(same, f"{scene}-{cond}: contact / GC / arrival == table_failure.csv")

    print()
    if fails:
        print(f"RESULT: FAIL - {len(fails)} check(s) did not reproduce")
        return 1
    print("RESULT: PASS - Table 12 / Fig. 9 key numbers reproduce from "
          "06_Robot_Loss_1080/detail.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
