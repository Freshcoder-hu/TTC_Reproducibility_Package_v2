#!/usr/bin/env python3
"""Reproduce the Table 11 / Fig. 8 key numbers from ``detail.csv``.

Everything here is read-only and standard-library only.  Run from the package
root (or anywhere):

    python 05_Curved_Path_240/reproduce_table11.py

It recomputes, from the 240 per-run rows and nothing else:

  * the per-method mean animal-obstacle contact fraction ``C`` and the
    goal-and-cohesion (GC) completion count on V2S5B -- the two columns the
    article prints in Table 11 and plots in Fig. 8(b);
  * the 30-seed mean / sd of the literal ``D_turn`` and ``E_turn``;
  * the obstacle-free control V2S5A (``C = 0`` in all 4 x 30 runs);
  * the warning that the centroid-window ``D_turn`` exists for seed 5000
    only, so it is a single-seed illustration, not a panel mean.

Expected values are those printed in the submitted article (s5.tex, Table 11).
Exit code is non-zero if any check fails.
"""
from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DETAIL = HERE / "detail.csv"
TABLE = HERE / "table_three_turn.csv"

METHODS = ["Ours_rollout", "B1", "B2", "VH-Path"]
LABEL = {"Ours_rollout": "Ours", "B1": "B1", "B2": "B2", "VH-Path": "VH-Path"}

# Table 11, V2S5B: C mean +/- sd, runs with C > 0, D_turn mean +/- sd,
# E_turn mean / max, GC completions of 30, effective encounters of 30.
EXPECTED = {
    "Ours_rollout": dict(c="0.0028", csd="0.0112", cpos=2, d="0.988", dsd="0.187",
                         e="0.045", emax="0.904", gc=27, nenc=27),
    "B1":           dict(c="0.0250", csd="0.0616", cpos=7, d="1.082", dsd="0.520",
                         e="0.161", emax="1.947", gc=25, nenc=29),
    "B2":           dict(c="0.0231", csd="0.0632", cpos=5, d="1.016", dsd="0.316",
                         e="0.087", emax="1.340", gc=26, nenc=28),
    "VH-Path":      dict(c="0.1639", csd="0.1935", cpos=17, d="2.052", dsd="0.792",
                         e="1.052", emax="3.306", gc=17, nenc=18),
}

# The four V2S5B centroid-window values quoted in the article's note.
CENTROID_WINDOW_V2S5B = {"Ours_rollout": "0.979", "B1": "1.205",
                         "B2": "0.973", "VH-Path": "1.089"}

fails = []


def check(ok, message):
    print(("  [PASS] " if ok else "  [FAIL] ") + message)
    if not ok:
        fails.append(message)


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def num(value):
    value = (value or "").strip()
    if value in ("", "nan", "NaN"):
        return float("nan")
    return float(value)


def is_true(value):
    return (value or "").strip() in ("1", "True", "true")


def main():
    rows = read_csv(DETAIL)
    print("Table 11 / Fig. 8 - confirmatory three-turn panel")
    print(f"   source: {DETAIL.relative_to(HERE.parent).as_posix()}")
    print(f"   rows in file: {len(rows)}  (columns: {len(rows[0])})")

    print("\n1. Design")
    scenes = sorted({r["scenario"] for r in rows})
    methods = sorted({r["method"] for r in rows})
    seeds = sorted({int(r["seed"]) for r in rows})
    check(len(rows) == 240, f"240 runs (got {len(rows)})")
    check(scenes == ["V2S5A", "V2S5B"], f"2 scenes {scenes}")
    check(methods == sorted(METHODS),
          f"4 method rows {methods} (3 main + VH-Path extension)")
    check(seeds == list(range(5000, 5030)), f"seeds 5000-5029 (got {seeds[0]}-{seeds[-1]})")
    cells = {(r["scenario"], r["method"]) for r in rows}
    check(len(cells) == 8, f"8 scene x method cells (got {len(cells)})")
    check(all(sum(1 for r in rows if (r["scenario"], r["method"]) == cell) == 30
              for cell in cells), "30 paired seeds in every cell")
    check(all(r["config_id"] == "b35d5401c119" for r in rows),
          "one frozen configuration b35d5401c119")
    check(sum(1 for r in rows if is_true(r["censored"])) == 0,
          "no right-censored run")
    check(sum(1 for r in rows if is_true(r["arrival"])) == 240,
          "240/240 arrivals")

    print("\n2. Table 11 on V2S5B, from detail.csv")
    print("   method        n    C mean +/- sd      runs C>0   D_turn mean +/- sd   "
          "E_turn mean / max   GC      n_enc")
    for method in METHODS:
        sub = [r for r in rows if r["scenario"] == "V2S5B" and r["method"] == method]
        c = [num(r["C_sampled"]) for r in sub]
        d = [num(r["D_turn"]) for r in sub]
        e = [num(r["E_turn"]) for r in sub]
        exp = EXPECTED[method]
        c_mean, c_sd = statistics.mean(c), statistics.stdev(c)
        d_mean, d_sd = statistics.mean(d), statistics.stdev(d)
        e_mean, e_max = statistics.mean(e), max(e)
        c_pos = sum(1 for x in c if x > 0)
        gc = sum(1 for r in sub if is_true(r["joint_completion"]))
        nenc = sum(1 for r in sub if is_true(r["effective_encounter"]))
        print(f"   {LABEL[method]:<12s} {len(sub):>3d}   "
              f"{c_mean:.4f} +/- {c_sd:.4f}   {c_pos:>3d}/30      "
              f"{d_mean:.3f} +/- {d_sd:.3f}      "
              f"{e_mean:.3f} / {e_max:.3f}     {gc:>2d}/30   {nenc:>2d}/30")
        check(f"{c_mean:.4f}" == exp["c"] and f"{c_sd:.4f}" == exp["csd"],
              f"{LABEL[method]}: C = {c_mean:.4f} +/- {c_sd:.4f} "
              f"== Table 11's {exp['c']} +/- {exp['csd']}")
        check(c_pos == exp["cpos"],
              f"{LABEL[method]}: contact in {c_pos}/30 runs == {exp['cpos']}/30")
        check(f"{d_mean:.3f}" == exp["d"] and f"{d_sd:.3f}" == exp["dsd"],
              f"{LABEL[method]}: D_turn = {d_mean:.3f} +/- {d_sd:.3f} "
              f"== {exp['d']} +/- {exp['dsd']}")
        check(f"{e_mean:.3f}" == exp["e"] and f"{e_max:.3f}" == exp["emax"],
              f"{LABEL[method]}: E_turn = {e_mean:.3f} / {e_max:.3f} "
              f"== {exp['e']} / {exp['emax']}")
        check(gc == exp["gc"],
              f"{LABEL[method]}: goal-and-cohesion completion {gc}/30 == {exp['gc']}/30")
        check(nenc == exp["nenc"],
              f"{LABEL[method]}: effective encounters {nenc}/30 == {exp['nenc']}/30")

    print("\n3. Cross-check against the shipped table_three_turn.csv")
    table = {r["method"]: r for r in read_csv(TABLE) if r["scene"] == "V2S5B"}
    check(sorted(table) == sorted(METHODS), "same four method rows")
    for method in METHODS:
        sub = [r for r in rows if r["scenario"] == "V2S5B" and r["method"] == method]
        row = table[method]
        c = [num(r["C_sampled"]) for r in sub]
        d = [num(r["D_turn"]) for r in sub]
        e = [num(r["E_turn"]) for r in sub]
        same = (
            abs(statistics.mean(c) - float(row["C_mean"])) < 1e-12
            and abs(statistics.stdev(c) - float(row["C_sd"])) < 1e-12
            and abs(statistics.mean(d) - float(row["D_turn_mean"])) < 1e-12
            and abs(statistics.stdev(d) - float(row["D_turn_sd"])) < 1e-12
            and abs(statistics.mean(e) - float(row["E_turn_mean"])) < 1e-12
            and abs(max(e) - float(row["E_turn_max"])) < 1e-12
            and sum(1 for r in sub if is_true(r["joint_completion"]))
            == int(row["joint_completed"])
            and sum(1 for r in sub if is_true(r["effective_encounter"]))
            == int(row["n_encounter"])
        )
        check(same, f"{LABEL[method]}: recomputed aggregates == table_three_turn.csv")

    print("\n4. Obstacle-free control V2S5A")
    control = [r for r in rows if r["scenario"] == "V2S5A"]
    check(len(control) == 120, f"120 runs (4 x 30) (got {len(control)})")
    check(all(num(r["C_sampled"]) == 0.0 for r in control),
          "C = 0 in all 120 runs")
    check(all(is_true(r["arrival"]) for r in control), "120/120 arrivals")
    for method in METHODS[:3]:
        sub = [r for r in control if r["method"] == method]
        gc = sum(1 for r in sub if is_true(r["joint_completion"]))
        check(gc == 26, f"{LABEL[method]}: GC 26/30 in the control (got {gc}/30)")
    identical = True
    for seed in range(5000, 5030):
        per_seed = {(round(num(r["D_turn"]), 12), round(num(r["rho"]), 12))
                    for r in control
                    if int(r["seed"]) == seed and r["method"] in METHODS[:3]}
        identical = identical and len(per_seed) == 1
    check(identical,
          "the three main methods are bit-identical on the control (D_turn, rho, per seed)")

    print("\n5. Centroid-window D_turn: seed 5000 only")
    window = [r for r in rows if (r["D_turn_centroid_window"] or "").strip()
              not in ("", "nan", "NaN")]
    check(len(window) == 8,
          f"exactly 8 rows carry D_turn_centroid_window (2 scenes x 4 methods); got {len(window)}")
    check({int(r["seed"]) for r in window} == {5000},
          "every centroid-window value is from seed 5000")
    for method in METHODS:
        sub = [r for r in window
               if r["scenario"] == "V2S5B" and r["method"] == method]
        value = f"{num(sub[0]['D_turn_centroid_window']):.3f}"
        check(value == CENTROID_WINDOW_V2S5B[method],
              f"{LABEL[method]}: V2S5B seed-5000 centroid-window D_turn = {value} "
              f"== {CENTROID_WINDOW_V2S5B[method]}")
    print("   NOTE: the other 29 seeds carry no centroid-window value, so these four")
    print("   numbers are a single-seed illustration, not panel means. The literal")
    print("   D_turn (section 2) is the only turn metric defined for all 30 seeds.")

    print()
    if fails:
        print(f"RESULT: FAIL - {len(fails)} check(s) did not reproduce")
        return 1
    print("RESULT: PASS - Table 11 / Fig. 8 key numbers reproduce from "
          "05_Curved_Path_240/detail.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
