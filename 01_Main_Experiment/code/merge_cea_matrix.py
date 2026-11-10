"""Merge the Stage A method batches into the single stage directory.

Reads  results_cea_matrix/_parts/<alg>/detail.csv  (one per method batch, written
by run_revised_experiments.py) and rebuilds, in results_cea_matrix/:

  detail.csv           2400 per-seed rows (8 methods x 10 scenarios x 30 seeds)
  summary.csv          per scenario x method, mean/std for every KEYS column
  summary_core.csv     per scenario x method, CORE_KEYS only
  main_table.csv       scenario-equal mean/sd of the 9 headline metrics
  main_table_by_scenario.csv   same metrics, not aggregated over scenarios
  events.csv           risk-event timing evidence (t_risk_on / t_divert_on /
                       ttc_lead / per-obstacle event order / preemption)
  integrity.json       row/seed/algorithm audit of the merged matrix

Usage: python merge_cea_matrix.py [--root results_cea_matrix]
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from run_experiments import KEYS, CORE_KEYS

ALGS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger",
        "VH-Danger-path", "B-Outmost", "B-PCBF"]
SCENARIOS = ["S1", "S1N", "S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]
SEEDS = list(range(200, 230))

HEADLINE = [
    ("C", "collision_rate"),        # collision rate (fraction of the 36 sheep hit)
    ("d_min", "min_dist"),          # sheep-obstacle minimum distance [m]
    ("TTC_min", "ttc_min"),         # minimum TTC [s]
    ("rho", "rho_avg"),             # path-keeping fraction
    ("max_dev", "max_dev"),         # maximum lateral path deviation [m]
    ("success", "success"),         # task success (goal reached)
    ("completion", "completion"),   # Nguyen-2026 completion criterion
    ("switch", "switch_count"),     # FSM mode switches
    ("ctrl_ms", "ctrl_ms"),         # per-cycle controller time (jobs=4: NOT for the paper)
]

EVENT_COLS = ["t_risk_on", "t_divert_on", "t_dist_trigger", "ttc_lead",
              "dur_pre", "dur_divert", "event_risk_on_1", "event_risk_on_2",
              "event_risk_on_3", "recover_start_1", "preempt_ok", "event_order_ok"]

_BOOLISH = {"True": 1.0, "False": 0.0, "true": 1.0, "false": 0.0,
            "TRUE": 1.0, "FALSE": 0.0, "1": 1.0, "0": 0.0}


def numeric(v):
    """Bool-safe numeric coercion (detail.csv stores booleans as True/False)."""
    s = pd.Series(v)
    if s.dtype == object:
        s = s.replace(_BOOLISH)
    return pd.to_numeric(s, errors="coerce").to_numpy(dtype=float)


def stat(v):
    """(mean, std) for numeric columns; mode for purely categorical ones."""
    a = numeric(v)
    a = a[np.isfinite(a)]
    if len(a):
        return float(a.mean()), float(a.std())
    s = pd.Series(v).dropna().astype(str)
    s = s[~s.isin(["", "?"])]
    if not len(s):
        return np.nan, np.nan
    mode = s.mode()
    return (mode.iloc[0] if len(mode) else np.nan), np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_matrix")
    args = ap.parse_args()
    root = Path(args.root)
    parts = root / "_parts"

    frames = []
    for alg in ALGS:
        p = parts / alg / "detail.csv"
        if not p.exists():
            raise SystemExit(f"missing batch: {p}")
        frames.append(pd.read_csv(p))
    d = pd.concat(frames, ignore_index=True)

    # ---------------- integrity audit ----------------
    problems = []
    got_algs = sorted(d.alg.unique())
    if got_algs != sorted(ALGS):
        problems.append(f"methods {got_algs} != {sorted(ALGS)}")
    for bad in ("Ours", "Ours_w/o_PRE"):
        if bad in set(d.alg):
            problems.append(f"forbidden method in main matrix: {bad}")
    if sorted(d.scenario.unique()) != sorted(SCENARIOS):
        problems.append(f"scenarios {sorted(d.scenario.unique())}")
    counts = d.groupby(["scenario", "alg"]).seed.nunique()
    if (counts != 30).any():
        problems.append(f"cells without 30 seeds: {counts[counts != 30].to_dict()}")
    if not all(set(d[(d.scenario == s) & (d.alg == a)].seed) == set(SEEDS)
               for s in SCENARIOS for a in ALGS):
        problems.append("seed block is not exactly 200-229 in every cell")
    if len(d) != 2400:
        problems.append(f"expected 2400 rows, got {len(d)}")
    dup = int(d.duplicated(["scenario", "alg", "seed"]).sum())
    if dup:
        problems.append(f"{dup} duplicated (scenario, alg, seed) rows")

    d = d.sort_values(["scenario", "alg", "seed"]).reset_index(drop=True)
    d.to_csv(root / "detail.csv", index=False, encoding="utf-8-sig")

    # ---------------- per scenario x method summaries ----------------
    srows, crows, erows = [], [], []
    for (sid, alg), g in d.groupby(["scenario", "alg"], sort=False):
        name = g.scenario_name.iloc[0]
        means, stds = {}, {}
        for k in KEYS:
            if k not in g.columns:
                means[k] = stds[k] = np.nan
                continue
            means[k], stds[k] = stat(g[k])
        full = dict(scenario=sid, scenario_name=name, alg=alg)
        full.update({f"{k}_mean": means[k] for k in KEYS})
        full.update({f"{k}_std": stds[k] for k in KEYS})
        srows.append(full)
        core = dict(scenario=sid, scenario_name=name, alg=alg)
        core.update({f"{k}_mean": means[k] for k in CORE_KEYS})
        core.update({f"{k}_std": stds[k] for k in CORE_KEYS})
        crows.append(core)
        ev = dict(scenario=sid, scenario_name=name, alg=alg)
        for c in EVENT_COLS:
            ev[c] = stat(g[c])[0] if c in g.columns else np.nan
        ev["mean_recovery_events"] = stat(g["recovery_events"])[0] \
            if "recovery_events" in g.columns else np.nan
        ev["n_collision_seeds"] = int((numeric(g["collision_rate"]) > 0).sum())
        ev["n_runs"] = len(g)
        erows.append(ev)

    pd.DataFrame(srows).to_csv(root / "summary.csv", index=False,
                               encoding="utf-8-sig")
    pd.DataFrame(crows).to_csv(root / "summary_core.csv", index=False,
                               encoding="utf-8-sig")
    pd.DataFrame(erows).to_csv(root / "events.csv", index=False,
                               encoding="utf-8-sig")

    # ---------------- headline tables ----------------
    by_s = pd.DataFrame(
        [{**{"scenario": r["scenario"], "alg": r["alg"]},
          **{name: r[f"{key}_mean"] for name, key in HEADLINE}}
         for r in srows])
    by_s.to_csv(root / "main_table_by_scenario.csv", index=False,
                encoding="utf-8-sig")

    eq = by_s.groupby("alg")[[n for n, _ in HEADLINE]].agg(["mean", "std"])
    flat = {}
    for n, _ in HEADLINE:
        flat[f"{n}_mean"] = eq[(n, "mean")]
        flat[f"{n}_std"] = eq[(n, "std")]
    eq_df = pd.DataFrame(flat).reindex(ALGS)
    eq_df.index.name = "alg"
    eq_df.to_csv(root / "main_table.csv", encoding="utf-8-sig")

    integrity = dict(rows=int(len(d)), methods=ALGS, scenarios=SCENARIOS,
                     seeds=[SEEDS[0], SEEDS[-1]], seeds_per_cell=30,
                     cells=int(len(counts)), problems=problems)
    (root / "integrity.json").write_text(json.dumps(integrity, indent=2))

    print(f"rows={len(d)}  cells={len(counts)}  problems={problems or 'none'}")
    print("\nscenario-equal main table:")
    print(eq_df.round(3).to_string())
    print(f"\nwrote {root}/{{detail,summary,summary_core,main_table,"
          f"main_table_by_scenario,events}}.csv + integrity.json")


if __name__ == "__main__":
    main()
