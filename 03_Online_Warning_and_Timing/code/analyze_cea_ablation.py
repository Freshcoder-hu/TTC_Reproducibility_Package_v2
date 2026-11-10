"""Paired comparison of the Stage B ablation rungs against the full Rollout.

Each rung is compared with `Ours_rollout` on the same seeds (paired), per
scenario: Wilcoxon signed-rank for continuous metrics, exact McNemar for
binary ones.  Holm within the rung family of one metric (5 rungs).

Usage: python analyze_cea_ablation.py --root results_cea_ablation
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon

FULL = "Ours_rollout"
RUNGS = ["R0_base", "w/o_Rollout", "w/o_event_coord", "horizon_6", "horizon_18"]
SCENARIOS = ["S2R2", "S3"]
METRICS = [("collision_rate", "C", True, False),
           ("min_dist", "d_min", False, False),
           ("completion", "completion", False, True),
           ("max_dev", "max_dev", True, False),
           ("switch_count", "switch", True, False)]
_BOOLISH = {"True": 1.0, "False": 0.0, "true": 1.0, "false": 0.0}


def num(s):
    s = s.replace(_BOOLISH) if s.dtype == object else s
    return pd.to_numeric(s, errors="coerce").to_numpy(dtype=float)


def holm(p):
    p = np.asarray(p, float)
    order = np.argsort(p)
    adj = np.empty(len(p))
    run = 0.0
    for rank, i in enumerate(order):
        run = max(run, (len(p) - rank) * p[i])
        adj[i] = min(1.0, run)
    return adj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_ablation")
    args = ap.parse_args()
    root = Path(args.root)
    out = root / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    d = pd.read_csv(root / "detail.csv")
    rows = []
    for scen in SCENARIOS:
        for col, label, lower_better, binary in METRICS:
            sub = []
            for rung in RUNGS:
                a = d[(d.scenario == scen) & (d.variant == FULL)].set_index("seed")
                b = d[(d.scenario == scen) & (d.variant == rung)].set_index("seed")
                common = a.index.intersection(b.index)
                va, vb = num(a.loc[common, col]), num(b.loc[common, col])
                delta = va - vb
                if binary:
                    n01 = int(np.sum((va > .5) & ~(vb > .5)))
                    n10 = int(np.sum(~(va > .5) & (vb > .5)))
                    disc = n01 + n10
                    p = 1.0 if disc == 0 else float(min(1.0, 2 * binomtest(
                        min(n01, n10), disc, .5).pvalue))
                    test = "exact_McNemar"
                else:
                    dd = delta[np.isfinite(delta)]
                    if len(dd) == 0 or np.all(dd == 0):
                        p = 1.0
                    else:
                        p = float(wilcoxon(dd, alternative="two-sided",
                                           zero_method="wilcox",
                                           method="auto").pvalue)
                    test = "Wilcoxon"
                md = float(np.nanmean(delta))
                better = (md < 0) if lower_better else (md > 0)
                sub.append(dict(scenario=scen, metric=label, rung=rung,
                                n=len(common), full_mean=float(np.nanmean(va)),
                                rung_mean=float(np.nanmean(vb)),
                                delta_full_minus_rung=md,
                                direction=("full better" if better and md != 0
                                           else ("rung better" if md != 0 else "tie")),
                                p_raw=p, test=test))
            adj = holm([r["p_raw"] for r in sub])
            for r, a_ in zip(sub, adj):
                r["p_holm_within_rungs"] = a_
                r["sig_0.05"] = bool(a_ < 0.05)
            rows.extend(sub)
    df = pd.DataFrame(rows)
    df.to_csv(out / "ablation_paired.csv", index=False, encoding="utf-8-sig")

    lines = ["| Scenario | metric | rung | full | rung | delta (full-rung) | "
             "direction | p_raw | p_Holm | sig |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for _, r in df.iterrows():
        nd = 2 if r.metric in ("C", "completion") else 3
        lines.append(f"| {r.scenario} | {r.metric} | {r.rung} | "
                     f"{r.full_mean:.{nd}f} | {r.rung_mean:.{nd}f} | "
                     f"{r.delta_full_minus_rung:+.{nd}f} | {r.direction} | "
                     f"{r.p_raw:.4g} | {r.p_holm_within_rungs:.4g} | "
                     f"{'yes' if r['sig_0.05'] else 'no'} |")
    (out / "ablation_paired.md").write_text("\n".join(lines), encoding="utf-8")
    print(df.to_string(index=False))
    print(f"\nwrote {out}/ablation_paired.csv and ablation_paired.md")


if __name__ == "__main__":
    main()
