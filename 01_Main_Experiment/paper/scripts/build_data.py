"""Build every paper table and the prose-number dictionary from the frozen V2 CSVs.

Inputs (read-only):  results_v2_stage5/{matrix,ablation,checks,trec_sweep}
Outputs:             paper_v7/{tables/*.csv, data/numbers.json}

Nothing here re-simulates: every number is a reduction of a stored CSV.
The statistics are the paper's *primary* family: seed-clustered (n = 30,
scenarios averaged equally within a seed), paired two-sided Wilcoxon
signed-rank test on the 30 seed-level differences, Holm within the family, plus
a percentile bootstrap 95 % CI of the paired median difference (10 000
resamples, fixed seed).  The run-level family is *descriptive* and is read from
the Stage-5 report where quoted.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common_paths import (CONFIG_ID, DATA, METHODS, RISK, SCENARIOS, S5,
                          TABLES, ENDPOINTS)

from scipy.stats import wilcoxon

SCEN9 = ["V2S1", "V2S1N", "V2S2", "V2S2R3", "V2S2R2", "V2S2R1", "V2S3",
         "V2S4S", "V2S4L"]   # V2S4M repeats V2S2: dropped from the primary

RNG = np.random.default_rng(12345)
NBOOT = 10000

# --- provenance of the statistics quoted in the prose -----------------------
# (copied from V2_STAGE5_REPORT.md sect. 7, not recomputed here, because the
#  report is the frozen analysis of record; verify_numbers.py cross-checks them)
REPORT = dict(
    n_seed_tests=49,
    n_seed_sig=38,
    n_run_sig=43,
    n_disagree=9,
    n_sign_reversal=2,
    mcnemar_joint_B1_seed_p=1.0000,
    mcnemar_joint_B1_run_p=1.0000,
    devW_B1_p_holm=0.0005,
    devW_B1_delta=0.064,
    clr_rs_ours=-0.48,
    clr_rr_ours=0.31,
    censored_total=377,
    preempt_ctrl_runs=25,
    preempt_truth_runs=0,
    preempt_legacy_runs=0,
    log_lag_max_s=5.83,
    crit_v1_runs=1269,
    crit_v1_den=1920,
    crit_v1_pct=66.1,
    crit_body_runs=1356,
    crit_body_den=1920,
    crit_body_pct=70.6,
    crit_added=87,
    crit_added_pp=4.5,
    trivial_serial_ms=1.68,
    cycle_ms=3.83,
    wall_h=3.00,
    c1_bind_cells=0,
    c1_stress_bind=500054,
)


def load_matrix() -> pd.DataFrame:
    frames = []
    # The 36:12 scenes carry the frozen configuration id; the two scale scenes
    # (18:6 and 60:20) hash their own ids because the group size is part of the
    # configuration record.  Both facts are asserted, not assumed.
    expected = {"V2S4S": "06897595d088", "V2S4L": "61dc9fae2dff"}
    for m in METHODS:
        df = pd.read_csv(os.path.join(S5, "matrix", f"{m}.csv"))
        for scn, g in df.groupby("scenario"):
            want = expected.get(scn, CONFIG_ID)
            assert (g["config_id"] == want).all(), (m, scn, g.config_id.unique())
        frames.append(df)
    df = pd.concat(frames, ignore_index=True)
    assert len(df) == 2400, len(df)
    assert set(df.method) == set(METHODS)
    assert sorted(df.seed.unique()) == list(range(2000, 2030))
    return df


def load_ablation() -> pd.DataFrame:
    df = pd.read_csv(os.path.join(S5, "ablation", "detail.csv"))
    assert len(df) == 300
    return df


def eq_weight(df: pd.DataFrame, col: str, scen: list[str]) -> float:
    """Mean over seeds within a scenario, then equal weight over scenarios."""
    sub = df[df.scenario.isin(scen)]
    per = sub.groupby("scenario")[col].mean()
    return float(per.mean())


def main_table(df: pd.DataFrame, scen=None) -> pd.DataFrame:
    scen = scen or SCENARIOS
    risk = [x for x in RISK if x in scen]
    rows = []
    for m in METHODS:
        d = df[(df.method == m) & (df.scenario.isin(scen))]
        rows.append(dict(
            method=m,
            C=eq_weight(d, "collision_rate", scen),
            clr_so=eq_weight(d.dropna(subset=["min_surface_clearance_so"]),
                             "min_surface_clearance_so", risk),
            ttc_min=eq_weight(d.dropna(subset=["ttc_min"]), "ttc_min", risk),
            rho=eq_weight(d, "rho_avg", scen),
            devW=eq_weight(d, "max_dev_norm", scen),
            arrival=eq_weight(d, "arrival", scen),
            joint=eq_weight(d, "joint_completion", scen),
            switches=float(d.switch_count.mean()),
            ctrl_ms=float(d.ctrl_ms.mean()),
            censored=int(d.censored.sum()),
            runs=len(d),
        ))
    return pd.DataFrame(rows)


def per_scenario(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for s in SCENARIOS:
        for m in METHODS:
            d = df[(df.method == m) & (df.scenario == s)]
            rows.append(dict(
                scenario=s, method=m,
                C=float(d.collision_rate.mean()),
                clr_so=float(d.min_surface_clearance_so.mean())
                if d.min_surface_clearance_so.notna().any() else np.nan,
                ttc_min=float(d.ttc_min.mean())
                if d.ttc_min.notna().any() else np.nan,
                rho=float(d.rho_avg.mean()),
                devW=float(d.max_dev_norm.mean()),
                arrival=float(d.arrival.mean()),
                joint=float(d.joint_completion.mean()),
            ))
    return pd.DataFrame(rows)


def ablation_table(abl: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for s in ["V2S2R2", "V2S3"]:
        for rung in ["R0_four_state", "R1_handover", "R2_event",
                     "R3_rollout", "R3_no_speed_policy"]:
            d = abl[(abl.scenario == s) & (abl.method == rung)]
            rows.append(dict(
                scenario=s, rung=rung, C=float(d.collision_rate.mean()),
                clr_so=float(d.min_surface_clearance_so.mean()),
                ttc_min=float(d.ttc_min.mean()), rho=float(d.rho_avg.mean()),
                devW=float(d.max_dev_norm.mean()),
                arrival=float(d.arrival.mean()),
                joint=float(d.joint_completion.mean()),
                switches=float(d.switch_count.mean()),
                bounded_ticks=float(d.sp_n_bounded_ticks.mean()),
                n=len(d),
            ))
    return pd.DataFrame(rows)


def seed_values(df: pd.DataFrame, method: str, col: str,
                scen: list[str]) -> pd.Series:
    """Seed-level mean of an endpoint over the scenarios with a defined value."""
    d = df[(df.method == method) & (df.scenario.isin(scen))]
    per = d.groupby(["seed", "scenario"])[col].mean().reset_index()
    per = per.dropna(subset=[col])
    return per.groupby("seed")[col].mean().reindex(range(2000, 2030))


def holm(p: np.ndarray) -> np.ndarray:
    order = np.argsort(p)
    out = np.empty_like(p)
    m = len(p)
    prev = 0.0
    for rank, i in enumerate(order):
        val = min(1.0, (m - rank) * p[i])
        prev = max(prev, val)
        out[i] = prev
    return out


def seed_diff(df: pd.DataFrame, base: str, col: str,
              scen: list[str]) -> tuple[np.ndarray, int]:
    """Paired seed-level differences (Ours - baseline), scenario-equal weights.

    A scene enters a seed's average only when *both* methods have a defined
    value there -- that is what makes the time-to-goal endpoint conditional on
    both members of a pair completing.
    """
    d = df[df.scenario.isin(scen)]
    piv = d.pivot_table(index=["seed", "scenario"], columns="method",
                        values=col, aggfunc="mean")
    piv = piv[[c for c in piv.columns if c in ("Ours_rollout", base)]]
    if "Ours_rollout" not in piv.columns or base not in piv.columns:
        return np.array([]), 0
    piv = piv.dropna()
    if piv.empty:
        return np.array([]), 0
    piv = piv.copy()
    piv["__diff__"] = piv["Ours_rollout"] - piv[base]
    per = piv.groupby(level=0)["__diff__"].mean()
    per = per.reindex(range(2000, 2030)).dropna()
    return per.to_numpy(), len(per)


def contrasts(df: pd.DataFrame, scen_all=None) -> pd.DataFrame:
    scen_all = scen_all or SCENARIOS
    risk = [x for x in RISK if x in scen_all]
    rows = []
    for ep, (col, better) in ENDPOINTS.items():
        scen = risk if ep in ("clr_so", "ttc_min") else scen_all
        for b in METHODS[1:]:
            d, n = seed_diff(df, b, col, scen)
            if n < 5:
                rows.append(dict(endpoint=ep, baseline=b, n=n,
                                 delta=np.nan, ci_lo=np.nan, ci_hi=np.nan,
                                 p=np.nan))
                continue
            if np.allclose(d, 0.0):
                p = 1.0
            else:
                p = float(wilcoxon(d, alternative="two-sided").pvalue)
            boots = np.array([np.median(RNG.choice(d, n, replace=True))
                              for _ in range(NBOOT)])
            rows.append(dict(
                endpoint=ep, baseline=b, n=n, delta=float(np.median(d)),
                mean_delta=float(np.mean(d)),
                ci_lo=float(np.percentile(boots, 2.5)),
                ci_hi=float(np.percentile(boots, 97.5)), p=p,
                better=("ours" if better * np.median(d) > 0 else "base"),
            ))
    out = pd.DataFrame(rows)
    ok = out.p.notna()
    out.loc[ok, "p_holm"] = holm(out.loc[ok, "p"].to_numpy())
    return out


def census(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for m in METHODS:
        d = df[df.method == m]
        rows.append(dict(
            method=m, runs=len(d),
            goal=int((d.terminated_by == "goal").sum()),
            timeout=int((d.terminated_by == "time").sum()),
            censored=int(d.censored.sum()), errors=int(d.run_error.notna().sum()),
            infeasible_events=float(d.n_infeasible.mean()),
            S=float(d.n_infeasible_S.mean()), P=float(d.n_infeasible_P.mean()),
            C_events=float(d.n_contact_C.mean()),
            clr_rs=float(d.min_surface_clearance_rs.mean()),
            clr_rr=float(d.min_surface_clearance_rr.mean()),
            contacts_ro=float(d.n_contact_events_ro.mean()),
            contacts_rs=float(d.n_contact_events_rs.mean()),
            contacts_rr=float(d.n_contact_events_rr.mean()),
            c1low_bind=float(d.bind_C1_low_sheep.mean()),
            c1hi_bind=float(d.bind_C1_high_sheep.mean()),
        ))
    return pd.DataFrame(rows)


def radar_table(main: pd.DataFrame) -> pd.DataFrame:
    """Min-max renormalisation *over the eight methods* (0 = worst, 1 = best)."""
    out = main[["method"]].copy()
    axes = {"C": -1, "clr_so": +1, "ttc_min": +1, "rho": +1, "devW": -1,
            "joint": +1}
    for ep, better in axes.items():
        v = main[ep].to_numpy(dtype=float)
        lo, hi = np.nanmin(v), np.nanmax(v)
        norm = (v - lo) / (hi - lo) if hi > lo else np.ones_like(v)
        if better < 0:
            norm = 1.0 - norm
        out[ep] = norm
    return out


def preemption_stats(df: pd.DataFrame) -> dict:
    d = df[df.method == "Ours_rollout"]
    per_seed = d[d.scenario == "V2S3"].groupby("seed").preempt_ctrl.max()
    return dict(runs_with_recover=int((d.n_recover_windows > 0).sum()),
                windows=int(d.n_recover_windows.sum()),
                closed=int(d.n_recover_complete.sum()),
                preempted=int(d.n_recover_preempted.sum()),
                seeds_v2s3_preempt=int((per_seed > 0).sum()))


def main() -> None:
    os.makedirs(DATA, exist_ok=True)
    os.makedirs(TABLES, exist_ok=True)
    df = load_matrix()
    abl = load_ablation()

    main10 = main_table(df, SCENARIOS)
    main9 = main_table(df, SCEN9)
    per = per_scenario(df)
    ab = ablation_table(abl)
    con = contrasts(df, SCEN9)          # primary family: de-duplicated
    con10 = contrasts(df, SCENARIOS)    # sensitivity: ten-scenario family
    cen = census(df)
    rad = radar_table(main9)

    main10.to_csv(os.path.join(TABLES, "table_main10.csv"), index=False)
    main9.to_csv(os.path.join(TABLES, "table_main9.csv"), index=False)
    con10.to_csv(os.path.join(TABLES, "table_contrasts10.csv"), index=False)
    per.to_csv(os.path.join(TABLES, "table_per_scenario.csv"), index=False)
    ab.to_csv(os.path.join(TABLES, "table_ablation.csv"), index=False)
    con.to_csv(os.path.join(TABLES, "table_contrasts.csv"), index=False)
    cen.to_csv(os.path.join(TABLES, "table_census.csv"), index=False)
    rad.to_csv(os.path.join(TABLES, "table_radar.csv"), index=False)

    # ---- headline numbers (everything the prose quotes) -------------------
    g = main9.set_index("method")
    g10 = main10.set_index("method")
    num = dict(
        config_id=CONFIG_ID,
        n_runs=int(len(df)), n_ablation=int(len(abl)),
        n_censored=int(df.censored.sum()),
        n_methods=len(METHODS), n_scenes=len(SCENARIOS), n_seeds=30,
        main9={m: {k: (None if pd.isna(v) else float(v))
                   for k, v in g.loc[m].items()} for m in METHODS},
        main10={m: {k: (None if pd.isna(v) else float(v))
                    for k, v in g10.loc[m].items()} for m in METHODS},
        contrasts10=con10.replace({np.nan: None}).to_dict(orient="records"),
        horizon=dict(
            L_140=140.0, L_220=220.0, W=8.0,
            T_max_140=200.0, T_max_220=320.0,
        ),
        contrasts=con.replace({np.nan: None}).to_dict(orient="records"),
        census=cen.replace({np.nan: None}).to_dict(orient="records"),
        ablation=ab.to_dict(orient="records"),
        preempt=preemption_stats(df),
        report=REPORT,
    )

    # derived facts the prose needs
    ours = g.loc["Ours_rollout"]
    b1 = g.loc["B1"]
    b2 = g.loc["B2"]
    num["derived"] = dict(
        b1_joint_minus_ours=float(b1["joint"] - ours["joint"]),
        b1_minus_ours_C=float(b1["C"] - ours["C"]),
        b2_minus_ours_C=float(b2["C"] - ours["C"]),
        b1_clr_minus_ours=float(b1["clr_so"] - ours["clr_so"]),
        b1_dev_minus_ours=float(b1["devW"] - ours["devW"]),
        contact_reduction_vs_b1=float(1.0 - ours["C"] / b1["C"]),
        contact_reduction_vs_b2=float(1.0 - ours["C"] / b2["C"]),
        rho_ours=float(ours["rho"]),
        vh_rho_min=float(g.loc[["VH-Path", "VH-Danger",
                                "VH-Danger-path"], "rho"].min()),
        vh_rho_max=float(g.loc[["VH-Path", "VH-Danger",
                                "VH-Danger-path"], "rho"].max()),
        vh_joint_min=float(g.loc[["VH-Path", "VH-Danger",
                                  "VH-Danger-path"], "joint"].min()),
        vh_joint_max=float(g.loc[["VH-Path", "VH-Danger",
                                  "VH-Danger-path"], "joint"].max()),
        vh_C_min=float(g.loc[["VH-Path", "VH-Danger",
                              "VH-Danger-path"], "C"].min()),
        vh_C_max=float(g.loc[["VH-Path", "VH-Danger",
                              "VH-Danger-path"], "C"].max()),
        outmost_C=float(g.loc["B-Outmost", "C"]),
        outmost_rho=float(g.loc["B-Outmost", "rho"]),
        outmost_joint=float(g.loc["B-Outmost", "joint"]),
        pcbf_C=float(g.loc["B-PCBF", "C"]),
        pcbf_arrival=float(g.loc["B-PCBF", "arrival"]),
        pcbf_censored=int(g.loc["B-PCBF", "censored"]),
        n_sig_seed=int((con.p_holm < 0.05).sum()),
        n_tests=int(con.p.notna().sum()),
    )
    # ablation deltas actually used in the text
    abi = ab.set_index(["scenario", "rung"])
    d = {}
    for s in ["V2S2R2", "V2S3"]:
        d[s] = dict(
            C=dict(R0=float(abi.loc[(s, "R0_four_state"), "C"]),
                   R1=float(abi.loc[(s, "R1_handover"), "C"]),
                   R2=float(abi.loc[(s, "R2_event"), "C"]),
                   R3=float(abi.loc[(s, "R3_rollout"), "C"]),
                   R3n=float(abi.loc[(s, "R3_no_speed_policy"), "C"])),
            clr=dict(R0=float(abi.loc[(s, "R0_four_state"), "clr_so"]),
                     R1=float(abi.loc[(s, "R1_handover"), "clr_so"]),
                     R2=float(abi.loc[(s, "R2_event"), "clr_so"]),
                     R3=float(abi.loc[(s, "R3_rollout"), "clr_so"]),
                     R3n=float(abi.loc[(s, "R3_no_speed_policy"), "clr_so"])),
            joint=dict(R0=float(abi.loc[(s, "R0_four_state"), "joint"]),
                       R1=float(abi.loc[(s, "R1_handover"), "joint"]),
                       R2=float(abi.loc[(s, "R2_event"), "joint"]),
                       R3=float(abi.loc[(s, "R3_rollout"), "joint"]),
                       R3n=float(abi.loc[(s, "R3_no_speed_policy"), "joint"])),
            devW=dict(R0=float(abi.loc[(s, "R0_four_state"), "devW"]),
                      R3=float(abi.loc[(s, "R3_rollout"), "devW"])),
            rho=dict(R0=float(abi.loc[(s, "R0_four_state"), "rho"]),
                     R3=float(abi.loc[(s, "R3_rollout"), "rho"])),
        )
    num["abl"] = d
    # per-run clearance distribution over the eight risk scenes: the mean of the
    # per-run minima is NOT a per-run guarantee
    per_run = {}
    for m in METHODS:
        r = df[(df.method == m) & (df.scenario.isin(RISK))]
        v = r.min_surface_clearance_so.to_numpy(dtype=float)
        v = v[np.isfinite(v)]
        per_run[m] = dict(n=int(v.size), neg=int((v <= 0).sum()),
                          worst=float(v.min()), mean=float(v.mean()))
    num["per_run_clearance"] = per_run
    num["abl_delta"] = dict(
        v2s2r2_R3_vs_R0_C=float(d["V2S2R2"]["C"]["R3"] - d["V2S2R2"]["C"]["R0"]),
        v2s2r2_R3_vs_R0_clr=float(d["V2S2R2"]["clr"]["R3"] - d["V2S2R2"]["clr"]["R0"]),
        v2s3_R2_to_R3_C=float(d["V2S3"]["C"]["R3"] - d["V2S3"]["C"]["R2"]),
        v2s3_R2_to_R3_clr=float(d["V2S3"]["clr"]["R3"] - d["V2S3"]["clr"]["R2"]),
        v2s3_R0_to_R2_C=float(d["V2S3"]["C"]["R2"] - d["V2S3"]["C"]["R0"]),
        v2s3_R3_vs_R0_C=float(d["V2S3"]["C"]["R3"] - d["V2S3"]["C"]["R0"]),
        v2s3_R3_vs_R0_clr=float(d["V2S3"]["clr"]["R3"] - d["V2S3"]["clr"]["R0"]),
        v2s3_R3_vs_R0_joint=float(d["V2S3"]["joint"]["R3"] - d["V2S3"]["joint"]["R0"]),
    )

    with open(os.path.join(DATA, "numbers.json"), "w", encoding="utf-8") as f:
        json.dump(num, f, indent=1, ensure_ascii=False)

    # ---- console check against V2_STAGE5_REPORT.md sect. 5.1 --------------
    print(main10.to_string(index=False, float_format=lambda x: f"{x:.3f}"))
    print()
    print("censored total:", int(df.censored.sum()))
    print("seed-clustered significant (recomputed):",
          int((con.p_holm < 0.05).sum()), "of", int(con.p.notna().sum()))
    print("abl V2S3 R2->R3: dC=%.3f dclr=%+.2f" %
          (d["V2S3"]["C"]["R3"] - d["V2S3"]["C"]["R2"],
           d["V2S3"]["clr"]["R3"] - d["V2S3"]["clr"]["R2"]))
    print("ablation 300 rows; formal 2400 rows")


if __name__ == "__main__":
    main()
