"""Ablation ladder at the gated radius (S2R2, S3; 5 rungs; paired vs `rollout`).

Rungs (as produced by `run_improvement.py --variants Ours handover event rollout B2`):
    Ours      four-state FSM only (the old Ours)
    handover  + handover coordination
    event     + event coordination      (= w/o Rollout)
    rollout   + rolling-horizon planner (= full Ours)
    B2        reference baseline

Usage:
    python analyze_ablation_30m.py --root results_cea_ablation_30m
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ORDER = ["Ours", "handover", "event", "rollout", "B2"]
LAB = {"Ours": "Ours（原始四状态）", "handover": "Ours_handover",
       "event": "Ours_event（= w/o Rollout）",
       "rollout": "Ours_rollout（完整 Ours）", "B2": "B2"}
MET = ["collision_rate", "min_dist", "ttc_min", "rho_avg", "max_dev",
       "completion", "success", "switch_count", "ctrl_ms"]


def holm(p):
    p = np.asarray(p, float)
    pf = np.where(np.isfinite(p), p, 1.0)
    order = np.argsort(pf, kind="stable")
    adj = np.empty(len(p))
    run = 0.0
    for rank, i in enumerate(order):
        run = max(run, (len(p) - rank) * pf[i])
        adj[i] = min(1.0, run)
    return adj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_ablation_30m")
    args = ap.parse_args()
    root = Path(args.root)
    d = pd.read_csv(root / "detail.csv", encoding="utf-8-sig")
    if "variant" not in d.columns:
        d["variant"] = d["alg"]
    met = [c for c in MET if c in d.columns]
    scen = [s for s in ("S2R2", "S3") if s in set(d.scenario)]
    print(f"行数 {len(d)} | 变体 {sorted(d.variant.unique())} | "
          f"场景 {sorted(d.scenario.unique())} | 种子 {d.seed.nunique()} "
          f"({d.seed.min()}-{d.seed.max()})")

    tabs, pair_rows = [], []
    for s in scen:
        sub = d[d.scenario == s]
        rows = []
        for v in ORDER:
            q = sub[sub.variant == v]
            if not len(q):
                continue
            r = {"变体": LAB.get(v, v)}
            for c in met:
                r[c] = float(pd.to_numeric(q[c], errors="coerce").mean())
            rows.append(r)
        t = pd.DataFrame(rows).set_index("变体")
        tabs.append((s, t))
        print(f"\n=== {s}（n={sub.seed.nunique()} seeds）===")
        print(t.round(4).to_string())
        for v in ("event", "handover", "Ours", "B2"):
            a = sub[sub.variant == "rollout"].set_index("seed")
            b = sub[sub.variant == v].set_index("seed")
            for c in ("collision_rate", "min_dist", "completion"):
                if c not in a.columns:
                    continue
                j = pd.concat([pd.to_numeric(a[c], errors="coerce").rename("a"),
                               pd.to_numeric(b[c], errors="coerce").rename("b")],
                              axis=1).dropna()
                if len(j) < 8:
                    continue
                diff = (j.a - j.b).values
                nz = diff[diff != 0]
                if len(nz) == 0:
                    p, test = 1.0, "identical"
                elif np.all(nz > 0) or np.all(nz < 0):
                    p, test = min(1.0, 2.0 ** (1 - len(nz))), "exact_sign"
                else:
                    try:
                        p = float(stats.wilcoxon(j.a, j.b, zero_method="wilcox",
                                                 correction=False).pvalue)
                        test = "wilcoxon"
                    except Exception:
                        p, test = np.nan, "failed"
                pair_rows.append(dict(scenario=s, rung=v, metric=c, n=len(j),
                                      rollout=float(j.a.mean()),
                                      rung_value=float(j.b.mean()),
                                      delta=float(np.mean(diff)), p_raw=p,
                                      test=test,
                                      n_seeds_diff=int(len(nz))))
    P = pd.DataFrame(pair_rows)
    if len(P):
        P["p_holm"] = holm(P.p_raw.values)
        fin = P.p_raw.notna().values
        assert np.all(P.loc[fin, "p_holm"].values >= P.loc[fin, "p_raw"].values)
        P["sig_0.05"] = P.p_holm < 0.05
        P.to_csv(root / "ablation_paired.csv", index=False, encoding="utf-8-sig")
        print(f"\n=== 配对检验（rollout − 对照），Holm 族 m={len(P)} ===")
        print(P[["scenario", "rung", "metric", "delta", "p_raw", "p_holm",
                 "sig_0.05"]].round(5).to_string(index=False))

    with (root / "ablation_table.md").open("w", encoding="utf-8") as f:
        for s, t in tabs:
            f.write(f"\n### {s}\n\n")
            f.write("| 变体 | C | d_min (m) | rho | max_dev (m) | completion |\n")
            f.write("|---|---|---|---|---|---|\n")
            for name, r in t.iterrows():
                f.write(f"| {name} | {r.collision_rate:.4f} | {r.min_dist:.4f} | "
                        f"{r.rho_avg:.4f} | {r.max_dev:.3f} | "
                        f"{r.completion:.4f} |\n")
    t_all = pd.concat([t.assign(scenario=s) for s, t in tabs])
    t_all.to_csv(root / "ablation_ladder.csv", encoding="utf-8-sig")
    print(f"\n已写 {root}/ablation_ladder.csv, ablation_paired.csv, "
          f"ablation_table.md")


if __name__ == "__main__":
    main()
