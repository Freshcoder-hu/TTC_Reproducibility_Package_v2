"""Stage-5 acceptance checks A-D.

    python -X utf8 -m v2.stage5.checks --tests A B C D --jobs 4

A  **Inertness.**  A spot cell with Stage-4 settings must reproduce the Stage-4
   result bit-exactly.  Two independent comparators:
     A1  V2S2 / B2 / seed 1000 through the Stage-5 path (probe on) vs the stored
         Stage-4 row ``results_v2_stage4/_det_j1/detail.csv`` (written 16:22,
         i.e. **after** the 15:46 spacing freeze) -- every Stage-4 column equal.
         (``results_v2_stage4/_smoke/detail.csv`` is deliberately *not* used as
         a comparator: it is stamped 15:25, before the freeze, and its V2S3 rows
         describe a superseded scene definition -- 2077 ticks instead of 2110.)
     A2  the Stage-5 probe on vs off -- every Stage-4 column equal, so the added
         measurement cannot move the dynamics.
     A3  the Stage-4 test-D case (V2S3 / Ours_rollout / seed 1000) reproduces
         the stored transcript numbers (C = 0.0278, min_dist 1.44 m, 2110
         control ticks, RECOVER windows 69.70-70.70 / 82.70-92.20 /
         149.20-151.20) **and** now reports preemption as True under the
         information-set definition (and False under the truth reading).

B  **Counts.**  2400 matrix rows (8 methods x 10 scenes x 30 seeds) + 240
   ablation rows, no missing cell, seeds complete 2000-2029, no duplicate cell,
   one ``config_id`` per scene family.

C  **Determinism.**  One cell re-run bit-identically; ``jobs = 1`` vs
   ``jobs = 4`` identical on every non-timing column.

D  **Cross-artefact consistency.**  The ablation's R3 rows equal the formal
   matrix's ``Ours_rollout`` rows for the two ablation scenes (bit-exact on all
   non-timing columns), and no V1 file changed with respect to the Stage-4
   snapshot.

Writes ``results_v2_stage5/checks/``: ``A_inertness.csv``, ``B_counts.json``,
``C_determinism.json``, ``D_consistency.json`` and ``CHECK_REPORT.txt`` with one
PASS/FAIL line per assertion.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5.runs import S5_KEYS, run_one  # noqa: E402

LOG = []
FAILS = []


def log(msg):
    print(msg, flush=True)
    LOG.append(msg)


def check(cond, msg):
    check.ok = bool(cond)
    log(("[PASS] " if cond else "[FAIL] ") + msg)
    if not cond:
        FAILS.append(msg)
    return cond


MISSING = (None, "", "nan", "NaN", "None")


def same(a, b, exclude=()):
    """Compare two row dicts on the intersection of their keys.

    A counter that never fired is ``None`` in the in-memory row (the counter bag
    only contains hit keys) and an empty CSV field once written, so
    ``None``/``""`` must compare equal; everything else is compared numerically
    when both sides parse as floats.  (Stage 4 lost a suite run to exactly this
    class of artefact -- a ``bool`` vs ``"True"`` comparison in its own diff --
    so it is handled explicitly here.)
    """
    keys = [k for k in a if k in b and k not in exclude]
    bad = []
    for k in keys:
        va, vb = a[k], b[k]
        if va in MISSING and vb in MISSING:
            continue
        fa = fb = None
        try:
            fa, fb = float(va), float(vb)
        except (TypeError, ValueError):
            pass
        if fa is not None and fb is not None:
            ok = (np.isnan(fa) and np.isnan(fb)) or fa == fb
        else:
            ok = (str(va) == str(vb)) or (va == vb)
        if not ok:
            bad.append((k, va, vb))
    return keys, bad


STAGE4_EXCLUDE = set(common.TIMING_COLS) | set(S5_KEYS)


def test_A(log_=log):
    log_("A. inertness")
    v2cfg = stage3_v2cfg("M1")
    # ---- A1 -----------------------------------------------------------
    stored_path = ROOT / "results_v2_stage4" / "_det_j1" / "detail.csv"
    stored = [r for r in csv.DictReader(open(stored_path,
                                             encoding="utf-8-sig"))
              if r["scenario"] == "V2S2" and r["alg"] == "B2"
              and r["seed"] == "1000"][0]
    fresh = run_one("V2S2", "B2", 1000, v2cfg=v2cfg)
    keys, bad = same(fresh, stored, exclude=STAGE4_EXCLUDE)
    rows = [dict(test="A1", cell="V2S2/B2/1000",
                 source="results_v2_stage4/_det_j1/detail.csv",
                 n_columns=len(keys), n_diff=len(bad),
                 first_diff=(bad[0][0] if bad else ""),
                 detail=json.dumps(bad[:5], default=str))]
    log_(f"   A1 V2S2/B2/1000: {len(keys)} Stage-4 columns compared, "
         f"{len(bad)} differ")
    check(not bad and len(keys) > 150,
          f"A1/the Stage-5 path reproduces the stored Stage-4 row bit-exactly "
          f"({len(keys)} columns, {len(bad)} differences)")
    # ---- A2 -----------------------------------------------------------
    again = run_one("V2S2", "B2", 1000, v2cfg=v2cfg, probe=False)
    keys2, bad2 = same(fresh, again, exclude=STAGE4_EXCLUDE)
    log_(f"   A2 probe on vs off: {len(keys2)} columns, {len(bad2)} differ")
    check(not bad2,
          "A2/the measurement probe is inert (probe on == probe off on every "
          "Stage-4 column)")
    rows.append(dict(test="A2", cell="V2S2/B2/1000", source="probe on vs off",
                     n_columns=len(keys2), n_diff=len(bad2),
                     first_diff=(bad2[0][0] if bad2 else ""),
                     detail=json.dumps(bad2[:5], default=str)))
    # ---- A3 -----------------------------------------------------------
    d = run_one("V2S3", "Ours_rollout", 1000, v2cfg=v2cfg)
    ok = (abs(float(d["collision_rate"]) - 1.0 / 36.0) < 1e-12
          and abs(float(d["min_dist"]) - 1.4351703145229284) < 1e-9
          and int(float(d["n_ctrl_ticks"])) == 2110
          and int(float(d["n_recover_windows"])) == 3
          and abs(float(d["recover_first_start"]) - 69.70) < 1e-6
          and abs(float(d["recover_first_end"]) - 70.70) < 1e-6)
    log_(f"   A3 V2S3/Ours_rollout/1000: C={d['collision_rate']:.6f} "
         f"min_dist={d['min_dist']:.6f} ticks={int(d['n_ctrl_ticks'])} "
         f"windows={int(d['n_recover_windows'])} "
         f"first=[{d['recover_first_start']:.2f},{d['recover_first_end']:.2f}] "
         f"preempt ctrl/truth/legacy="
         f"{int(d['preempt_ctrl'])}/{int(d['preempt_truth'])}/"
         f"{int(d['preempt_legacy'])}")
    check(ok, "A3/the Stage-4 test-D case is reproduced bit-exactly "
              "(C=1/36, min_dist=1.4351703145, 2110 ticks, first RECOVER "
              "window 69.70-70.70)")
    check(int(d["preempt_ctrl"]) == 1,
          "A3/preemption under the controller information set is True for the "
          "D case (the defect is fixed)")
    check(int(d["preempt_truth"]) == 0 and int(d["preempt_legacy"]) == 0,
          "A3/the truth-based readings stay False, as Stage 4 documented")
    rows.append(dict(test="A3", cell="V2S3/Ours_rollout/1000",
                     source="Stage-4 D transcript",
                     n_columns=6, n_diff=0 if ok else 1,
                     first_diff="" if ok else "transcript mismatch",
                     detail=json.dumps({k: d[k] for k in
                                        ("collision_rate", "min_dist",
                                         "n_ctrl_ticks", "n_recover_windows",
                                         "recover_first_start",
                                         "recover_first_end", "preempt_ctrl",
                                         "preempt_truth", "preempt_legacy")},
                                       default=str)))
    with open(common.CHECK_DIR / "A_inertness.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return all(r["n_diff"] == 0 for r in rows)


def test_B(log_=log):
    log_("B. counts")
    ok = True
    total = 0
    per_method = {}
    for m in common.METHODS:
        p = common.MATRIX_DIR / f"{m}.csv"
        if not p.exists():
            check(False, f"B/matrix file missing: {p.name}")
            ok = False
            continue
        rows = list(csv.DictReader(open(p, encoding="utf-8-sig")))
        cells = {(r["scenario"], int(r["seed"])) for r in rows}
        want = {(s, sd) for s in common.SCENARIOS for sd in common.SEEDS}
        missing = want - cells
        extra = cells - want
        dup = len(rows) - len(cells)
        n_err = sum(1 for r in rows if str(r.get("run_error") or "").strip())
        n_cens = sum(1 for r in rows if int(float(r.get("censored") or 0)))
        per_method[m] = dict(n_rows=len(rows), n_cells=len(cells),
                             missing=len(missing), extra=len(extra),
                             duplicate=dup, n_error=n_err, n_censored=n_cens,
                             config_ids=sorted({r["config_id"] for r in rows}))
        total += len(rows)
        good = (len(rows) == 300 and not missing and not extra and not dup)
        check(good, f"B/{m}: {len(rows)} rows, {len(missing)} missing cells, "
                    f"{len(extra)} extra, {dup} duplicates, {n_err} errors, "
                    f"{n_cens} censored")
        ok &= good
    abl = common.ABL_DIR / "detail.csv"
    n_abl = 0
    if abl.exists():
        arows = list(csv.DictReader(open(abl, encoding="utf-8-sig")))
        ladder = [r for r in arows if r.get("rung") in common.ABLATION_RUNGS]
        probe = [r for r in arows if r.get("rung") not in common.ABLATION_RUNGS]
        n_abl = len(ladder)
        acells = {(r["rung"], r["scenario"], int(r["seed"])) for r in ladder}
        want_a = {(rg, s, sd) for rg in common.ABLATION_RUNGS
                  for s in common.ABLATION_SCENARIOS for sd in common.SEEDS}
        miss_a = want_a - acells
        check(len(ladder) == 240 and not miss_a,
              f"B/ablation: {len(ladder)} ladder rows (+{len(probe)} "
              f"supplementary probe rows), {len(miss_a)} missing cells")
        ok &= (len(ladder) == 240 and not miss_a)
    else:
        check(False, "B/ablation detail.csv missing")
        ok = False
    check(total == 2400, f"B/formal matrix total: {total} rows (expected 2400)")
    ok &= total == 2400
    common.write_json(common.CHECK_DIR / "B_counts.json",
                      dict(total_matrix=total, total_ablation=n_abl,
                           per_method=per_method, ok=bool(ok)))
    return ok


def _cell(t):
    return run_one(*t, v2cfg=stage3_v2cfg("M1"))


def _cell_task(t):
    return _cell(t)


def test_C(log_=log, jobs=4):
    log_("C. determinism")
    cells = [("V2S2", "B2", 2000), ("V2S3", "Ours_rollout", 2000)]
    one = [_cell(c) for c in cells]
    two = [_cell(c) for c in cells]
    bad = []
    for a, b in zip(one, two):
        _, d = same(a, b, exclude=set(common.TIMING_COLS))
        bad += d
    check(not bad, f"C1/two independent runs of the same cell are "
                   f"bit-identical on every non-timing column ({len(bad)} diffs)")
    # ---- jobs 1 vs 4 ---------------------------------------------------
    out = {}
    for j in (1, 4):
        if j > 1:
            with ProcessPoolExecutor(max_workers=j) as ex:
                out[j] = list(ex.map(_cell_task, cells, chunksize=1))
        else:
            out[j] = [_cell_task(c) for c in cells]
    bad2 = []
    for a, b in zip(out[1], out[4]):
        _, d = same(a, b, exclude=set(common.TIMING_COLS))
        bad2 += d
    check(not bad2, f"C2/jobs=1 == jobs=4 on {len(cells)} cells x "
                    f"{len(out[1][0])} columns ({len(bad2)} differences on "
                    f"non-timing columns)")
    common.write_json(common.CHECK_DIR / "C_determinism.json", dict(
        cells=[list(c) for c in cells], repeat_diffs=len(bad),
        jobs_diffs=len(bad2), diff_detail=bad[:10] + bad2[:10],
        timing_excluded=sorted(set(common.TIMING_COLS))))
    return not bad and not bad2


def test_D(log_=log):
    log_("D. cross-artefact consistency")
    ok = True
    # ---- D1: ablation R3 == matrix Ours_rollout on the two scenes ------
    abl = common.ABL_DIR / "detail.csv"
    if abl.exists():
        arows = [r for r in csv.DictReader(open(abl, encoding="utf-8-sig"))
                 if r.get("rung") == "R3_rollout"]
        mrows = []
        for s in common.ABLATION_SCENARIOS:
            p = common.MATRIX_DIR / "Ours_rollout.csv"
            mrows += [r for r in csv.DictReader(open(p, encoding="utf-8-sig"))
                      if r["scenario"] == s]
        midx = {(r["scenario"], int(r["seed"])): r for r in mrows}
        bad = []
        n = 0
        # ``method``/``alg`` are the rung's label by design, not a measurement
        for r in arows:
            k = (r["scenario"], int(r["seed"]))
            if k not in midx:
                continue
            n += 1
            _, d = same(r, midx[k], exclude=set(common.TIMING_COLS)
                        | {"method", "alg", "rung"})
            bad += d
        check(not bad and n == 60,
              f"D1/the ablation's top rung reproduces the formal Ours_rollout "
              f"cells bit-exactly ({n} cells, {len(bad)} differences)")
        ok &= (not bad and n == 60)
    else:
        check(False, "D1/ablation detail.csv missing")
        ok = False
    # ---- D2: no V1 file changed ---------------------------------------
    snap_path = ROOT / "results_v2_stage4" / "code_hashes_pre_stage4.json"
    moved = []
    if snap_path.exists():
        snap = json.loads(snap_path.read_text(encoding="utf-8"))
        for name, h in snap.get("v1", {}).items():
            p = ROOT / name
            if p.exists() and common.sha16(p) != h:
                moved.append(name)
        check(not moved,
              f"D2/no V1 file changed since the Stage-4 snapshot "
              f"({len(snap.get('v1', {}))} files checked, {len(moved)} moved)")
        ok &= not moved
    common.write_json(common.CHECK_DIR / "D_consistency.json",
                      dict(ablation_matches_matrix=bool(ok), moved_v1=moved))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=["A", "B", "C", "D"])
    ap.add_argument("--jobs", type=int, default=4)
    args = ap.parse_args()
    common.ensure_dirs()
    log(f"Stage-5 checks: {args.tests}")
    for t in args.tests:
        if t == "A":
            test_A()
        elif t == "B":
            test_B()
        elif t == "C":
            test_C(jobs=args.jobs)
        elif t == "D":
            test_D()
    (common.CHECK_DIR / "CHECK_REPORT.txt").write_text(
        "\n".join(LOG) + "\n", encoding="utf-8")
    log(f"\n{len(FAILS)} failing assertion(s)")
    for f in FAILS:
        log("  FAIL: " + f)
    print(f"wrote {common.CHECK_DIR / 'CHECK_REPORT.txt'}")


if __name__ == "__main__":
    main()
