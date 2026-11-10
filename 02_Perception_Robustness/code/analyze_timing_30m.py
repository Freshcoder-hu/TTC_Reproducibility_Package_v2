"""Strict-serial per-cycle cost at the gated radius (median / mean / IQR, ratio to B1).

Input must come from a `--jobs 1` run with BLAS pinned to one thread and no other
load; that is the only wall clock in this study that may be quoted as a
per-cycle cost (parallel batches are reported for magnitude only).

Usage:
    python analyze_timing_30m.py --root results_cea_timing_30m
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
ALGS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
        "B-Outmost", "B-PCBF"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_timing_30m")
    args = ap.parse_args()
    root = Path(args.root)
    d = pd.read_csv(root / "detail.csv", encoding="utf-8-sig")
    if "alg" not in d.columns:
        d["alg"] = d["variant"]
    d["ctrl_ms"] = pd.to_numeric(d["ctrl_ms"], errors="coerce")
    d = d.dropna(subset=["ctrl_ms"])
    print(f"行数 {len(d)} | 方法 {d.alg.nunique()} | "
          f"场景 {sorted(d.scenario.unique())} | 种子 {d.seed.nunique()}")
    print("jobs=1 串行，BLAS 单线程\n")
    rows = []
    for alg, s in d.groupby("alg"):
        x = s.ctrl_ms.values
        rows.append({"方法": alg, "n": len(x), "seeds": s.seed.nunique(),
                     "scenarios": s.scenario.nunique(),
                     "中位数 (ms)": float(np.median(x)),
                     "均值 (ms)": float(x.mean()),
                     "标准差": float(x.std(ddof=1)) if len(x) > 1 else 0.0,
                     "p25": float(np.percentile(x, 25)),
                     "p75": float(np.percentile(x, 75))})
    t = pd.DataFrame(rows).sort_values("中位数 (ms)")
    base = t[t.方法 == "B1"]["中位数 (ms)"]
    if len(base):
        t["相对 B1"] = t["中位数 (ms)"] / float(base.iloc[0])
    print("=== 严格串行单周期耗时（R = 30 m）===")
    print(t.round(3).to_string(index=False))
    t.to_csv(root / "timing_table.csv", index=False, encoding="utf-8-sig")
    # per scenario median, for the B1 ratio stability check
    piv = d.pivot_table(index="scenario", columns="alg", values="ctrl_ms",
                        aggfunc="median")
    piv.to_csv(root / "timing_by_scenario.csv", encoding="utf-8-sig")
    print("\n每场景中位数 (ms/周期):")
    print(piv.round(2).to_string())
    print(f"\n已写 {root}/timing_table.csv, timing_by_scenario.csv")


if __name__ == "__main__":
    main()
