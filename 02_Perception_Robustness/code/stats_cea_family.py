"""Stage D -- statistics for the NEW 8-method family.

Family definition (this file, and nothing else):
    1 "Ours"  = Ours_rollout (the Rollout controller)
  + 7 baselines = B1, B2, VH-Path, VH-Danger, VH-Danger-path, B-Outmost, B-PCBF
  => family size 7 paired comparisons per metric.

Explicitly NOT mixed with: the old 64/72-test family (which contained the old
`Ours` / `Ours_w/o_PRE`), the limited-perception family, or the Distance x PRE
family.  Those live in other result directories.

Protocol
  * paired by seed (200-229) inside each scenario: Wilcoxon signed-rank, two-sided
  * "scenario-equal": one number per scenario (mean paired difference), then
    Wilcoxon over the 10 scenario-level differences (n=10, exact)
  * binary metrics (success, completion): exact McNemar (binomial test on the
    discordant pairs); scenario-equal binary = exact McNemar over the pooled
    300 pairs (descriptive, ignores within-scenario correlation)
  * Holm correction WITHIN the 7 baselines of one metric (and, for the
    per-scenario block, within the 7 baselines of one scenario x metric).
    A secondary global-Holm column over all 7 x 8 scenario-equal tests is
    reported for transparency but is NOT the pre-registered family.

`ctrl_ms` is deliberately excluded from the significance family: the matrix ran
with --jobs 4, so its wall-clock timing is load-inflated.  Stage C's strict
serial run is the authoritative source for ctrl_ms.

Usage: python stats_cea_family.py --root results_cea_matrix
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon

OURS = "Ours_rollout"
BASELINES = ["B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
             "B-Outmost", "B-PCBF"]
SCENARIOS = ["S1", "S1N", "S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]

# metric -> (label, lower_is_better, binary)
METRICS = [
    ("collision_rate", "C", True, False),
    ("min_dist", "d_min", False, False),
    ("ttc_min", "TTC_min", False, False),
    ("rho_avg", "rho", False, False),
    ("max_dev", "max_dev", True, False),
    ("switch_count", "switch", True, False),
    ("success", "success", False, True),
    ("completion", "completion", False, True),
]
DESCRIPTIVE = [("ctrl_ms", "ctrl_ms", True, False)]

_BOOLISH = {"True": 1.0, "False": 0.0, "true": 1.0, "false": 0.0}


def coerce(s):
    s = s.replace(_BOOLISH) if s.dtype == object else s
    return pd.to_numeric(s, errors="coerce").to_numpy(dtype=float)


def holm(pvals):
    """Holm step-down adjusted p-values (monotone, capped at 1)."""
    p = np.asarray(pvals, dtype=float)
    m = len(p)
    order = np.argsort(p)
    adj = np.empty(m, dtype=float)
    running = 0.0
    for rank, i in enumerate(order):
        val = (m - rank) * p[i]
        running = max(running, val)
        adj[i] = min(1.0, running)
    return adj


def wilcoxon_p(delta):
    d = np.asarray(delta, dtype=float)
    d = d[np.isfinite(d)]
    if len(d) == 0:
        return np.nan, "no_data"
    if np.all(d == 0):
        return 1.0, "all_zero"
    try:
        r = wilcoxon(d, alternative="two-sided", zero_method="wilcox", method="auto")
        return float(r.pvalue), getattr(r, "method", "wilcoxon")
    except ValueError:
        # all differences identical non-zero -> exact sign test degenerate
        k = int((d > 0).sum())
        n = int((d != 0).sum())
        return float(min(1.0, 2 * binomtest(min(k, n - k), n, 0.5).pvalue)), "sign"


def mcnemar_exact(a, b):
    """Exact McNemar on paired binary vectors a (Ours) and b (baseline)."""
    a = np.asarray(a) > 0.5
    b = np.asarray(b) > 0.5
    n01 = int(np.sum(a & ~b))     # Ours wins
    n10 = int(np.sum(~a & b))     # baseline wins
    disc = n01 + n10
    if disc == 0:
        return 1.0, n01, n10, 0
    p = 2 * binomtest(min(n01, n10), disc, 0.5).pvalue
    return float(min(1.0, p)), n01, n10, disc


def verdict(mean_delta, lower_better):
    if not np.isfinite(mean_delta) or mean_delta == 0:
        return "tie"
    better = (mean_delta < 0) if lower_better else (mean_delta > 0)
    return "better" if better else "worse"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_matrix")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    root = Path(args.root)
    out = Path(args.out) if args.out else root / "analysis"
    out.mkdir(parents=True, exist_ok=True)

    d = pd.read_csv(root / "detail.csv")
    pivot = {}
    for (sid, alg), g in d.groupby(["scenario", "alg"]):
        pivot[(sid, alg)] = g.set_index("seed").sort_index()

    rows_pc, rows_eq, rows_desc = [], [], []
    for col, label, lower_better, binary in METRICS + DESCRIPTIVE:
        is_formal = not all(col == c for c, *_ in DESCRIPTIVE)  # ctrl_ms excluded
        for scen in SCENARIOS:
            sub = []
            for base in BASELINES:
                o = pivot[(scen, OURS)]
                b = pivot[(scen, base)]
                common = o.index.intersection(b.index)
                if binary:
                    p, n01, n10, disc = mcnemar_exact(coerce(o.loc[common, col]),
                                                      coerce(b.loc[common, col]))
                    od = coerce(o.loc[common, col]).mean()
                    bd = coerce(b.loc[common, col]).mean()
                    delta = od - bd
                    rec = dict(metric=label, baseline=base, scenario=scen,
                               n=len(common), ours_mean=od, base_mean=bd,
                               delta=delta, wins=n01, losses=n10,
                               discordant=disc, p_raw=p, test="exact_McNemar")
                else:
                    od = coerce(o.loc[common, col])
                    bd = coerce(b.loc[common, col])
                    delta_arr = od - bd
                    p, method = wilcoxon_p(delta_arr)
                    rec = dict(metric=label, baseline=base, scenario=scen,
                               n=len(common), ours_mean=float(np.nanmean(od)),
                               base_mean=float(np.nanmean(bd)),
                               delta=float(np.nanmean(delta_arr)),
                               wins=int(np.nansum(delta_arr < 0)),
                               losses=int(np.nansum(delta_arr > 0)),
                               ties=int(np.nansum(delta_arr == 0)),
                               p_raw=p, test=method)
                rec["direction"] = verdict(rec["delta"], lower_better)
                rec["formal_family"] = is_formal
                sub.append(rec)
            adj = holm([r["p_raw"] for r in sub])
            for r, a in zip(sub, adj):
                r["p_holm_within_scenario"] = a
                r["sig_0.05"] = bool(a < 0.05)
            rows_pc.extend(sub)

        # ---- scenario-equal ----
        eqsub = []
        for base in BASELINES:
            per_scen_delta, pooled_a, pooled_b = [], [], []
            for scen in SCENARIOS:
                o = pivot[(scen, OURS)]
                b = pivot[(scen, base)]
                common = o.index.intersection(b.index)
                oa, ba = coerce(o.loc[common, col]), coerce(b.loc[common, col])
                per_scen_delta.append(float(np.nanmean(oa - ba)))
                pooled_a.append(oa)
                pooled_b.append(ba)
            if binary:
                p, n01, n10, disc = mcnemar_exact(np.concatenate(pooled_a),
                                                  np.concatenate(pooled_b))
                # how many individual scenarios are significant on their own
                n_sig = sum(1 for r in rows_pc if r["metric"] == label
                            and r["baseline"] == base and r["sig_0.05"])
                rec = dict(metric=label, baseline=base, n_scenarios=len(SCENARIOS),
                           n_pairs=len(np.concatenate(pooled_a)),
                           delta=float(np.mean(per_scen_delta)),
                           delta_median=float(np.median(per_scen_delta)),
                           wins=n01, losses=n10, discordant=disc, p_raw=p,
                           test="exact_McNemar_pooled300",
                           n_scenarios_sig=n_sig)
            else:
                p, method = wilcoxon_p(per_scen_delta)
                n_sig = sum(1 for r in rows_pc if r["metric"] == label
                            and r["baseline"] == base and r["sig_0.05"])
                rec = dict(metric=label, baseline=base, n_scenarios=len(SCENARIOS),
                           delta=float(np.mean(per_scen_delta)),
                           delta_median=float(np.median(per_scen_delta)),
                           wins=int(np.sum(np.array(per_scen_delta) < 0)),
                           losses=int(np.sum(np.array(per_scen_delta) > 0)),
                           ties=int(np.sum(np.array(per_scen_delta) == 0)),
                           p_raw=p, test=method + "_n10", n_scenarios_sig=n_sig)
            rec["direction"] = verdict(rec["delta"], lower_better)
            rec["formal_family"] = is_formal
            eqsub.append(rec)
        adj = holm([r["p_raw"] for r in eqsub])
        for r, a in zip(eqsub, adj):
            r["p_holm_within_metric"] = a
            r["sig_0.05"] = bool(a < 0.05)
        rows_eq.extend(eqsub)

    pc = pd.DataFrame(rows_pc)
    eq = pd.DataFrame(rows_eq)

    # secondary: global Holm over all formal scenario-equal tests
    mask = eq.formal_family.to_numpy(dtype=bool)
    eq["p_holm_global_8metrics"] = np.nan
    eq.loc[mask, "p_holm_global_8metrics"] = holm(eq.loc[mask, "p_raw"].to_numpy())

    pc.to_csv(out / "stats_per_scenario.csv", index=False, encoding="utf-8-sig")
    eq.to_csv(out / "stats_scenario_equal.csv", index=False, encoding="utf-8-sig")

    fam = eq[eq.formal_family]
    summary = dict(
        family="CEA-8: 1 Ours_rollout + 7 baselines",
        ours=OURS, baselines=BASELINES, family_size=len(BASELINES),
        metrics=[m[0] for m in METRICS],
        excluded_from_family=["ctrl_ms (jobs=4 load-inflated; see Stage C)"],
        scenarios=SCENARIOS, seeds="200-229 (30 paired seeds per scenario)",
        n_tests_scenario_equal=int(len(fam)),
        n_tests_per_scenario=int(len(pc[pc.formal_family])),
        n_sig_scenario_equal_holm=int(fam["sig_0.05"].sum()),
        n_sig_scenario_equal_global=int((eq.p_holm_global_8metrics < 0.05).sum()),
        n_sig_per_scenario_cells=int(pc[pc.formal_family]["sig_0.05"].sum()),
        not_mixed_with=["old 64/72-test family (old Ours / Ours_w/o_PRE)",
                        "limited-perception family", "Distance x PRE family"],
    )
    (out / "family_definition.json").write_text(json.dumps(summary, indent=2))

    # ---- markdown ----
    lines = ["### Family CEA-8: 1 Ours + 7 baselines (Holm within this family only)",
             "",
             f"* family size: **{len(BASELINES)}** paired comparisons per metric",
             f"* scenario-equal tests: {summary['n_tests_scenario_equal']} "
             f"({len(SCENARIOS)} scenarios, Wilcoxon n=10 / pooled McNemar)",
             f"* per-scenario cells: {summary['n_tests_per_scenario']}",
             f"* significant after Holm (scenario-equal): "
             f"**{summary['n_sig_scenario_equal_holm']}** / "
             f"{summary['n_tests_scenario_equal']}",
             f"* significant after per-scenario Holm cells: "
             f"**{summary['n_sig_per_scenario_cells']}** / "
             f"{summary['n_tests_per_scenario']}",
             ""]
    for label in [m[1] for m in METRICS]:
        sub = eq[(eq.metric == label) & eq.formal_family]
        lines.append(f"**{label}**")
        lines.append("")
        lines.append("| vs | delta (Ours-base) | direction | p_raw | p_Holm | sig | "
                     "scenarios sig |")
        lines.append("|---|---|---|---|---|---|---|")
        for _, r in sub.iterrows():
            lines.append(
                f"| {r.baseline} | {r.delta:+.3f} | {r.direction} | "
                f"{r.p_raw:.4g} | {r.p_holm_within_metric:.4g} | "
                f"{'yes' if r['sig_0.05'] else 'no'} | {int(r.get('n_scenarios_sig', 0))} |")
        lines.append("")
    (out / "stats_report.md").write_text("\n".join(lines), encoding="utf-8")

    print("\n".join(lines[:12]))
    print(f"\nwrote {out}/stats_per_scenario.csv, stats_scenario_equal.csv, "
          f"family_definition.json, stats_report.md")
    print(f"\nscenario-equal, formal metrics:")
    show = fam[["metric", "baseline", "delta", "direction", "p_raw",
                "p_holm_within_metric", "sig_0.05"]]
    print(show.to_string(index=False))


if __name__ == "__main__":
    main()
