"""Verify the ported obstacle-detection gate before any big run.

Three checks, all through the *production* runner (`run_revised_experiments.py`),
so the code path exercised here is exactly the one used by the 30 m matrix:

  A. INERTNESS.  R = 10000 m must reproduce `results_cea_matrix/_parts/<alg>/
     detail.csv` BIT-EXACTLY on every non-timing column (all 8 methods, 3
     obstacle-using scenarios, seed 200).  Timing (`ctrl_ms`) is excluded by
     construction -- it is wall clock.

  B. GATE ACTIVITY.  R = 10 m must differ from R = 30 m for the methods that
     consume obstacle state, and must be identical for those that do not.
     (Which is which is *measured*, not assumed.)

  C. FOV SENSITIVITY.  R = 30 m range-only vs R = 30 m + sensor cone (the v4
     lineage's exact rule).  Reported, not used for the main series.

Usage:
    python verify_gate_30m.py            # runs all checks (writes results_gate_checks)
    python verify_gate_30m.py --report   # re-print from existing outputs
"""
import csv
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parent
CHK = ROOT / "results_gate_checks"
BASE = ROOT / "results_cea_matrix" / "_parts"
RUNNER = "run_revised_experiments.py"
ALGS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
        "B-Outmost", "B-PCBF"]
TIMING_COLS = {"ctrl_ms"}
ENV = dict(os.environ, OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
           OPENBLAS_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1",
           PYTHONIOENCODING="utf-8", PYTHONUTF8="1")


def call(out, scenarios, seeds, offset, algs, radius, fov=False, jobs=12):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, "-X", "utf8", "-u", RUNNER,
           "--scenarios", *scenarios,
           "--seeds", str(seeds), "--seed-offset", str(offset),
           "--algs", *algs, "--jobs", str(jobs), "--out", str(out),
           "--sensing-radius", str(radius)]
    if fov:
        cmd.append("--gate-fov")
    print("$ " + " ".join(cmd[2:]), flush=True)
    r = subprocess.run(cmd, cwd=ROOT, env=ENV)
    if r.returncode != 0:
        raise SystemExit(f"runner failed rc={r.returncode}: {out}")
    return out / "detail.csv"


def read_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    assert rows, f"empty {path}"
    return rows


def key(r):
    return (r["scenario"], r["alg"], str(r["seed"]))


def compare(a_rows, b_rows, label, skip=("ctrl_ms",), a_name="", b_name=""):
    """Cell-by-cell string comparison of two detail.csv row sets, keyed."""
    A = {key(r): r for r in a_rows}
    B = {key(r): r for r in b_rows}
    only_a = sorted(set(A) - set(B))
    only_b = sorted(set(B) - set(A))
    common = sorted(set(A) & set(B))
    cols = [c for c in a_rows[0].keys() if c not in skip]
    diff_cells, diff_rows, same_rows = [], [], []
    for k in common:
        ra, rb = A[k], B[k]
        rowdiff = [c for c in cols if str(ra.get(c)) != str(rb.get(c))]
        if rowdiff:
            diff_rows.append(k)
            diff_cells += [(k, c, ra.get(c), rb.get(c)) for c in rowdiff]
        else:
            same_rows.append(k)
    out = dict(label=label, a=a_name, b=b_name, n_a=len(A), n_b=len(B),
               only_in_a=only_a, only_in_b=only_b, n_common=len(common),
               columns_compared=cols, n_cols=len(cols),
               n_rows_identical=len(same_rows),
               n_rows_differing=len(diff_rows),
               differing_rows=[list(map(str, k)) for k in diff_rows],
               differing_columns=sorted({c for _, c, _, _ in diff_cells}),
               n_cells_differing=len(diff_cells),
               first_diffs=[dict(scenario=k[0], alg=k[1], seed=k[2], column=c,
                                 a=str(x), b=str(y))
                            for k, c, x, y in diff_cells[:12]])
    print(f"\n--- {label} ---\n  A={a_name}  B={b_name}")
    print(f"  rows A={len(A)} B={len(B)} common={len(common)} "
          f"onlyA={len(only_a)} onlyB={len(only_b)}  cols={len(cols)}")
    print(f"  identical rows: {len(same_rows)}   differing rows: {len(diff_rows)}"
          f"   differing cells: {len(diff_cells)}")
    if diff_cells:
        print("  differing columns:", out["differing_columns"])
        for d in out["first_diffs"][:6]:
            print("   ", d)
    return out


def baseline_rows(scenarios, seeds):
    rows = []
    for alg in ALGS:
        p = BASE / alg / "detail.csv"
        if not p.exists():
            raise SystemExit(f"missing baseline {p}")
        for r in read_rows(p):
            if r["scenario"] in scenarios and int(r["seed"]) in seeds:
                rows.append(r)
    return rows


def main():
    report = {}
    CHK.mkdir(exist_ok=True)

    # ---------- A. inertness ----------
    sc_inert = ["S2R2", "S2R1", "S3"]
    seeds = [200]
    print("=== A. INERTNESS  R = 10000 m vs full-visibility baseline ===",
          flush=True)
    p = call(CHK / "inert_R10000", sc_inert, len(seeds), seeds[0], ALGS, 10000)
    report["inertness"] = compare(
        read_rows(p), baseline_rows(sc_inert, set(seeds)),
        "A. inertness (R=10000 vs baseline)", a_name="R=10000",
        b_name="results_cea_matrix/_parts")

    # ---------- B. gate activity ----------
    sc_act = ["S2R1", "S3"]
    seeds2 = [200, 201]
    print("\n=== B. GATE ACTIVITY  R = 10 m vs R = 30 m ===", flush=True)
    p10 = call(CHK / "gate_R10", sc_act, len(seeds2), seeds2[0], ALGS, 10)
    p30 = call(CHK / "gate_R30", sc_act, len(seeds2), seeds2[0], ALGS, 30)
    r10, r30 = read_rows(p10), read_rows(p30)
    rep = compare(r10, r30, "B. gate activity (R=10 vs R=30)",
                  a_name="R=10", b_name="R=30")
    per_alg = {}
    for alg in ALGS:
        a = [r for r in r10 if r["alg"] == alg]
        b = [r for r in r30 if r["alg"] == alg]
        sub = compare(a, b, f"B.{alg}", a_name=f"R=10/{alg}", b_name=f"R=30/{alg}") \
            if a and b else None
        if sub:
            per_alg[alg] = dict(n_rows=sub["n_common"],
                                identical=sub["n_rows_differing"] == 0,
                                n_rows_differing=sub["n_rows_differing"],
                                differing_columns=sub["differing_columns"],
                                n_cells_differing=sub["n_cells_differing"])
    rep["per_alg"] = per_alg
    report["gate_activity"] = rep

    # ---------- C. FOV sensitivity ----------
    sc_fov = ["S2R1"]
    algs_fov = ["Ours_rollout", "B2", "VH-Danger", "B-Outmost"]
    print("\n=== C. FOV SENSITIVITY  R = 30 m range-only vs +cone ===", flush=True)
    pf = call(CHK / "fov_R30", sc_fov, 1, 200, algs_fov, 30, fov=True)
    a = [r for r in read_rows(pf)]
    b = [r for r in r30 if r["scenario"] == "S2R1" and r["alg"] in algs_fov
         and int(r["seed"]) == 200]
    report["fov"] = compare(a, b, "C. FOV sensitivity (R=30 +cone vs range-only)",
                            a_name="R=30+cone", b_name="R=30 range-only")

    (CHK / "gate_checks.json").write_text(json.dumps(report, indent=2),
                                          encoding="utf-8")
    ok_inert = report["inertness"]["n_rows_differing"] == 0 and \
        report["inertness"]["n_cells_differing"] == 0
    print("\n" + "=" * 72)
    print(f"INERTNESS PASSED: {ok_inert}   "
          f"({report['inertness']['n_rows_identical']}/"
          f"{report['inertness']['n_common']} rows bit-identical, "
          f"{report['inertness']['n_cols']} non-timing columns)")
    print("GATE ACTIVITY at R=10 vs R=30:")
    for alg, v in per_alg.items():
        print(f"  {alg:15s} {'CHANGED' if not v['identical'] else 'identical'}"
              f"  rows_differ={v['n_rows_differing']}/{v['n_rows']}"
              f"  cols={v['differing_columns']}")
    print(f"wrote {CHK/'gate_checks.json'}")
    if not ok_inert:
        raise SystemExit("INERTNESS FAILED -- do not run the 30 m matrix")


if __name__ == "__main__":
    main()
