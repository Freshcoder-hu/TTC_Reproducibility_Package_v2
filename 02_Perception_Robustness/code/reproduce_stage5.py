"""Regenerate every number of the Stage-5 report from the CSVs (acceptance D).

    python -X utf8 reproduce_stage5.py                  # tables + numbers
    python -X utf8 reproduce_stage5.py --write-report    # assemble the report
    python -X utf8 reproduce_stage5.py --check-report    # verify the document

The report is *assembled* from a prose template (``v2/stage5/report.py``) and
the tables this script regenerates from the Stage-5 CSVs, so the document cannot
drift from the data: ``--write-report`` rewrites ``V2_STAGE5_REPORT.md`` and
``--check-report`` fails if the file on disk differs from the regenerated one.

Inputs (all under ``results_v2_stage5/``)
    matrix/<method>.csv            8 x 300 formal runs
    matrix/_formal_batches.json    per-batch wall clock
    ablation/detail.csv            4 rungs x 2 scenes x 30 seeds
    trec_sweep/*.csv|.json         the T_REC sweep
    c1_audit/*.csv|.json           the C1 audit
    checks/*.csv|.json             acceptance A-D and the probes

Outputs
    results_v2_stage5/reproduced/<table>.md    one file per table
    results_v2_stage5/reproduced/report_numbers.json
    V2_STAGE5_REPORT.md                        (only with --write-report)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from v2.stage5 import common, stats, tables  # noqa: E402


def numbers(extra=None):
    """The report's headline numbers, as a JSON-able dict."""
    extra = extra or {}
    rows = tables.matrix()
    out = dict(
        config_id=(rows[0]["config_id"] if rows else ""),
        seeds=[min(common.SEEDS), max(common.SEEDS)],
        n_seeds=len(common.SEEDS), n_scenarios=len(common.SCENARIOS),
        n_methods=len(common.METHODS), n_runs=len(rows),
        family_sizes=stats.family_sizes(),
        per_method={},
    )
    for m in common.METHODS:
        sub = [r for r in rows if r.get("method") == m]
        out["per_method"][m] = dict(
            n=len(sub),
            C=tables._mean(sub, "collision_rate")[0],
            clr_so=tables._mean(sub, "min_surface_clearance_so")[0],
            ttc_min=tables._mean(sub, "ttc_min")[0],
            rho=tables._mean(sub, "rho_avg")[0],
            devW=tables._mean(sub, "max_dev_norm")[0],
            ctrl_ms=tables._mean(sub, "ctrl_ms")[0],
            arrival=tables._mean(sub, "arrival")[0],
            joint=tables._mean(sub, "joint_completion")[0],
            censored=sum(1 for r in sub if int(tables._num(
                r.get("censored")) or 0)),
            errors=sum(1 for r in sub if str(r.get("run_error") or "").strip()),
            infeasible=tables._mean(sub, "n_infeasible")[0],
            inf_S=tables._mean(sub, "n_infeasible_S")[0],
            inf_P=tables._mean(sub, "n_infeasible_P")[0],
            contact_C=tables._mean(sub, "n_contact_C")[0],
            preempt_ctrl=sum(1 for r in sub
                             if tables._num(r.get("preempt_ctrl")) == 1),
            preempt_truth=sum(1 for r in sub
                              if tables._num(r.get("preempt_truth")) == 1),
            preempt_legacy=sum(1 for r in sub
                               if tables._num(r.get("preempt_legacy")) == 1),
            c1_low_bind=tables._cnt(sub, "bind_C1_low_sheep")[0],
        )
    comp = extra.get("comp") or []
    out["n_sig_seed"] = sum(1 for r in comp if r["sig_seed"])
    out["n_sig_run"] = sum(1 for r in comp if r["sig_run"])
    out["n_disagree"] = sum(1 for r in comp if r["disagree"])
    out["n_sign_flip"] = sum(1 for r in comp if r["sign_flip"])
    # per-scene aggregates of the shipped method (for the prose)
    out["per_scene"] = {}
    for s in common.SCENARIOS:
        sub = [r for r in rows
               if r.get("method") == "Ours_rollout" and r.get("scenario") == s]
        if not sub:
            continue
        for key, col in (("C", "collision_rate"),
                         ("clr_so", "min_surface_clearance_so"),
                         ("ttc_min", "ttc_min"), ("rho", "rho_avg"),
                         ("devW", "max_dev_norm"),
                         ("arrival", "arrival"),
                         ("joint", "joint_completion"),
                         ("switch", "switch_count"),
                         ("cycle", "ctrl_ms")):
            out["per_scene"][f"{s}/{key}"] = tables._mean(sub, col)[0]
    out["disagreements"] = [dict(endpoint=r["endpoint"], baseline=r["baseline"],
                                 p_seed=r["p_seed_holm"], p_run=r["p_run_holm"],
                                 med_seed=r["med_seed"], med_run=r["med_run"])
                            for r in comp if r["disagree"]]
    # ablation contrast R2 -> R3 on V2S3
    abl = tables._rows(common.ABL_DIR / "detail.csv")
    if abl:
        out["ablation"] = {}
        for rg in common.ABLATION_RUNGS:
            for s in common.ABLATION_SCENARIOS:
                sub = [r for r in abl if r.get("rung") == rg
                       and r.get("scenario") == s]
                if not sub:
                    continue
                out["ablation"][f"{s}/{rg}"] = dict(
                    C=tables._mean(sub, "collision_rate")[0],
                    clr_so=tables._mean(sub, "min_surface_clearance_so")[0],
                    ttc_min=tables._mean(sub, "ttc_min")[0],
                    rho=tables._mean(sub, "rho_avg")[0],
                    devW=tables._mean(sub, "max_dev_norm")[0],
                    arrival=tables._mean(sub, "arrival")[0],
                    joint=tables._mean(sub, "joint_completion")[0])
    # timing
    p = common.MATRIX_DIR / "_formal_batches.json"
    if p.exists():
        man = json.loads(p.read_text(encoding="utf-8"))
        out["timing"] = dict(
            total_wall_s=man.get("total_wall_s"),
            batches={b["method"]: dict(wall_s=b.get("wall_s"),
                                       s_per_cell=b.get("s_per_cell"),
                                       n=b.get("n"),
                                       censored=b.get("n_censored"),
                                       errors=b.get("n_error"))
                     for b in man.get("batches", [])})
        sp = {int(r["jobs"]): float(r["speedup"])
              for r in tables._rows(common.CHECK_DIR / "speedup.csv")
              if str(r.get("speedup") or "").strip()}
        sp_ctrl = {int(r["jobs"]): float(r["ctrl_ms_mean"])
                   for r in tables._rows(common.CHECK_DIR / "speedup.csv")
                   if str(r.get("ctrl_ms_mean") or "").strip()}
        tw = float(man.get("total_wall_s") or 0.0) / 3600.0
        if 12 in sp:
            out["proj"] = dict(
                jobs12_h=tw,
                speedup12=sp.get(12), speedup4=sp.get(4),
                jobs1_h=(tw * sp[12] if 12 in sp else None),
                jobs4_h=(tw * sp[12] / sp[4] if (4 in sp and 12 in sp) else None),
                eff4=(sp.get(4, 0) / 4.0), eff12=(sp.get(12, 0) / 12.0),
                ctrl_ms_serial=sp_ctrl.get(1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-report", action="store_true")
    ap.add_argument("--check-report", action="store_true")
    ap.add_argument("--report", default=str(ROOT / "V2_STAGE5_REPORT.md"))
    args = ap.parse_args()
    common.ensure_dirs()
    tabs, extra = tables.write_all()
    nums = numbers(extra)
    common.write_json(common.REPRO_DIR / "report_numbers.json", nums)
    print(f"wrote {len(tabs)} tables + report_numbers.json to {common.REPRO_DIR}")

    from v2.stage5 import report as rpt
    text = rpt.render(tabs, nums)
    if args.write_report:
        Path(args.report).write_text(text, encoding="utf-8")
        print(f"wrote {args.report} ({len(text.splitlines())} lines)")
    if args.check_report:
        p = Path(args.report)
        if not p.exists():
            print("CHECK-REPORT: FAIL (report missing)")
            return 1
        have = p.read_text(encoding="utf-8")
        if have.replace("\r\n", "\n") == text.replace("\r\n", "\n"):
            print("CHECK-REPORT: PASS (the document is byte-identical to the "
                  "regenerated one)")
            return 0
        # locate the first difference for a useful message
        a = have.replace("\r\n", "\n").splitlines()
        b = text.replace("\r\n", "\n").splitlines()
        for i in range(min(len(a), len(b))):
            if a[i] != b[i]:
                print(f"CHECK-REPORT: FAIL at line {i + 1}\n  file: {a[i][:160]}"
                      f"\n  regen: {b[i][:160]}")
                return 1
        print(f"CHECK-REPORT: FAIL (line count {len(a)} vs {len(b)})")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
