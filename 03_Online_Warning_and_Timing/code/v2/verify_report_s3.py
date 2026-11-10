"""Cross-check V2_STAGE3_REPORT.md against the artefacts it quotes.

Every number in the report that a reader might want to re-derive has a machine
source under ``results_v2_stage3/``; this script asserts the two agree, so the
document cannot silently drift from the data (the same principle as
``config_v2.PARAM_TABLE`` for the parameter tables).

    python -X utf8 -m v2.verify_report_s3
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from v2.config_v2 import (STAGE2_TTC, STAGE3_TTC, SENSE_DELAY_LEVELS,  # noqa: E402
                          SENSE_LOSS_LEVELS, SENSE_RANGE_LEVELS, stage3_v2cfg)

OUT = ROOT / "results_v2_stage3"
REPORT = ROOT / "V2_STAGE3_REPORT.md"

CHECKS = []


def check(name, ok, detail=""):
    CHECKS.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}"
                                                      if detail else ""))


def rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def main():
    rep = REPORT.read_text(encoding="utf-8")
    cal = json.loads((OUT / "ttc_calibration.json").read_text(encoding="utf-8"))
    acc = json.loads((OUT / "ACCEPTANCE.json").read_text(encoding="utf-8"))

    # ---- 1. the frozen tuple, in code, in JSON and in the report ----
    fr = cal["frozen"]
    check("frozen tuple: config_v2 == ttc_calibration.json",
          all(abs(float(STAGE3_TTC[k]) - float(fr[k])) < 1e-9 for k in fr),
          str(STAGE3_TTC))
    check("frozen tuple: stage3_v2cfg applies it",
          all(abs(getattr(stage3_v2cfg("M1"), k) - float(fr[k])) < 1e-9
              for k in fr))
    check("report quotes the frozen tuple",
          "(12, 8, 15, 3)" in rep and "(T_PRE, T_DIVERT, T_REC, h) = (12.0, 8.0, "
          "15.0, 3.0)" in rep)
    check("report quotes the Stage-2 tuple", str(STAGE2_TTC["ttc_pre"]) in rep)

    # ---- 2. calibration gates ----
    check("all calibration gates pass in the JSON",
          all(g["ok"] for g in cal["gates"]),
          "; ".join(g["name"] for g in cal["gates"] if not g["ok"]) or "all ok")
    m = cal["measured"]
    check("design-envelope p10 in the JSON is 13.01 s",
          abs(m["p10_design_envelope"] - 13.005) < 1e-9,
          f"{m['p10_design_envelope']:.3f}")
    check("report quotes p10 = 13.01 s", "13.01 s" in rep)
    check("report quotes the 16 s failure margin (-2.99 s)",
          "-2.99" in rep, f"{m['p10_design_envelope'] - 16.0:+.2f}")

    # ---- 3. observable-time table ----
    tab = rows(OUT / "observability" / "observable_table_R70.csv")
    check("observable table has 15 cells (5 geometries x 3 speeds)",
          len(tab) == 15, str(len(tab)))
    check("observable table covers 3/4/6 m/s",
          {float(r["speed"]) for r in tab} == {3.0, 4.0, 6.0})
    n18 = sum(1 for r in tab if float(r["T_obs_p10"]) >= 18.0)
    print(f"    ({n18} cells reach a 18 s warning at 70 m)")
    for r in tab:
        g, v = r["geometry"], float(r["speed"])
        p10 = float(r["T_obs_p10"])
        if f"| {p10:.2f} |" not in rep:
            # allow the bolded / suffixed forms used in the report table
            if f"{p10:.2f}" not in rep:
                check(f"report quotes the p10 of {g}@{v:g}", False, f"{p10:.2f}")
                break
    else:
        check("report quotes every cell's p10", True, "15/15")

    # ---- 4. range sweep ----
    sw = rows(OUT / "observability" / "observable_sweep.csv")
    n_expect = len(SENSE_RANGE_LEVELS) * len(SENSE_DELAY_LEVELS) * \
        len(SENSE_LOSS_LEVELS)
    check(f"range x delay x loss sweep has {n_expect} rows",
          len(sw) == n_expect, str(len(sw)))
    check("report quotes the required range (88.9 / 121.2 / 69.2 m)",
          all(x in rep for x in ("88.9", "121.2", "69.2")),
          str(cal["required_range"]["env_16"]))

    # ---- 5. pilot ----
    pil = rows(OUT / "E_pilot.csv")
    check("pilot has 4 arms x 2 scenarios x 2 seeds x 2 algs = 32 runs",
          len(pil) == 32, str(len(pil)))
    summ = rows(OUT / "E_pilot_summary.csv")
    def get(arm, alg, sid, key):
        for r in summ:
            if r["arm"] == arm and r["alg"] == alg and r["scenario"] == sid:
                return float(r[key])
        return float("nan")
    d_full = get("S2_fullvis", "Ours_rollout", "S3", "C") - \
        get("S2_fullvis", "B2", "S3", "C")
    d_gate = get("S3_gated_realQ", "Ours_rollout", "S3", "C") - \
        get("S3_gated_realQ", "B2", "S3", "C")
    check("pilot: Ours-vs-B2 advantage survives gating (S3)",
          d_gate <= 0.0, f"dC full={d_full:+.3f} gated={d_gate:+.3f}")

    # ---- 6. acceptance suite ----
    npass = sum(1 for a in acc if a["ok"])
    check(f"acceptance suite {npass}/{len(acc)} checks pass",
          npass == len(acc), str([a["name"] for a in acc if not a["ok"]]))

    # ---- 7. provenance ----
    check("pre-edit hashes recorded",
          (OUT / "code_hashes_pre_stage3.json").exists())
    check("Stage-2 code snapshot present",
          (OUT / "_stage2_code_snapshot" / "sim_v2.py").exists())
    refs = list((OUT / "reference").glob("*.npz"))
    check("4 Stage-2 reference traces", len(refs) == 4, str(len(refs)))

    bad = [c for c in CHECKS if not c[1]]
    print(f"\nSUMMARY: {len(CHECKS) - len(bad)}/{len(CHECKS)} checks passed")
    for n, _, d in bad:
        print(f"  FAILED: {n}  {d}")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
