"""Compare the 10-scenario and 9-scenario (de-duplicated) aggregations.

V2S4M repeats the V2S2 geometry at 36:12; its rows are identical to V2S2, so the
ten-scenario equal weighting counts that encounter twice.  This script reports
both aggregations and whether any paired contrast changes its Holm decision.
It writes nothing: it is the evidence for the paper's choice of primary
aggregation.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pandas as pd
from scipy.stats import wilcoxon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_data import holm, load_matrix, seed_diff
from common_paths import ENDPOINTS, METHODS, RISK, SCENARIOS

SCEN9 = ["V2S1", "V2S1N", "V2S2", "V2S2R3", "V2S2R2", "V2S2R1", "V2S3",
         "V2S4S", "V2S4L"]


def agg(df, scen):
    rows = []
    risk = [s for s in RISK if s in scen]
    for m in METHODS:
        d = df[(df.method == m) & (df.scenario.isin(scen))]
        dr = d[d.scenario.isin(risk)]
        rows.append(dict(
            method=m,
            C=d.collision_rate.mean(),
            clr=dr.min_surface_clearance_so.mean(),
            ttc=dr.ttc_min.mean(),
            rho=d.rho_avg.mean(),
            devW=d.max_dev_norm.mean(),
            arrival=d.arrival.mean(),
            joint=d.joint_completion.mean(),
            ctrl_ms=d.ctrl_ms.mean(),
        ))
    return pd.DataFrame(rows).set_index("method")


def contrasts(df, scen):
    out = {}
    for ep, (col, better) in ENDPOINTS.items():
        risk = [s for s in RISK if s in scen]
        use = risk if ep in ("clr_so", "ttc_min") else scen
        for b in METHODS[1:]:
            d, n = seed_diff(df, b, col, use)
            if n < 5:
                out[(ep, b)] = (np.nan, np.nan)
                continue
            p = 1.0 if np.allclose(d, 0) else float(
                wilcoxon(d, alternative="two-sided").pvalue)
            out[(ep, b)] = (float(np.median(d)), p)
    # Holm within the family
    keys = [k for k, v in out.items() if np.isfinite(v[1])]
    ps = np.array([out[k][1] for k in keys])
    hs = holm(ps)
    for k, h in zip(keys, hs):
        out[k] = (out[k][0], out[k][1], float(h))
    return out


def main():
    df = load_matrix()
    a10, a9 = agg(df, SCENARIOS), agg(df, SCEN9)
    print("=== 10-scenario (all ten) ===")
    print(a10.round(3).to_string())
    print("\n=== 9-scenario (de-duplicated) ===")
    print(a9.round(3).to_string())
    print("\n=== difference (9 - 10) ===")
    print((a9 - a10).round(4).to_string())

    c10, c9 = contrasts(df, SCENARIOS), contrasts(df, SCEN9)
    print("\nsignificant counts: 10-scen %d, 9-scen %d (of 48 computable)" % (
        sum(1 for v in c10.values() if len(v) == 3 and v[2] < 0.05),
        sum(1 for v in c9.values() if len(v) == 3 and v[2] < 0.05)))
    print("\nchanged decisions (alpha = 0.05, Holm):")
    changed = 0
    for k in sorted(set(c10) | set(c9)):
        v10, v9 = c10.get(k), c9.get(k)
        if v10 is None or v9 is None or len(v10) < 3 or len(v9) < 3:
            continue
        h10, h9 = v10[2], v9[2]
        if (h10 < 0.05) != (h9 < 0.05):
            changed += 1
            print(f"  {k[0]:9s} vs {k[1]:14s}: p_holm 10-scen "
                  f"{h10:.4g} -> 9-scen {h9:.4g}")
    print("changed:", changed)
    print("\ndev/W vs B1 p_holm: 10-scen %.3g, 9-scen %.3g" %
          (c10[("devW", "B1")][2], c9[("devW", "B1")][2]))
    print("C vs B1 p_holm: 10-scen %.3g, 9-scen %.3g" %
          (c10[("C", "B1")][2], c9[("C", "B1")][2]))


if __name__ == "__main__":
    main()
