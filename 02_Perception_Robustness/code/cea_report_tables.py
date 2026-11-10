"""Render the markdown tables used by CEA_MATRIX_REPORT.md.

Reads whatever stages exist and writes results_cea_matrix/analysis/*.md:

  main_table.md      8 methods x (C, d_min, TTC_min, rho, max_dev, success,
                     completion, switch, ctrl_ms) -- scenario-equal mean +/- sd
  per_scenario.md    C / d_min / completion per scenario x method
  events.md          risk-event timing evidence (t_risk_on, t_divert_on,
                     t_dist_trigger, ttc_lead, event order, preemption)
  ablation.md        Stage B ladder
  timing.md          Stage C strict-serial ctrl_ms

Usage:
  python cea_report_tables.py --root results_cea_matrix \
      --ablation results_cea_ablation --timing results_cea_timing_serial
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

ALGS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger",
        "VH-Danger-path", "B-Outmost", "B-PCBF"]
SCENARIOS = ["S1", "S1N", "S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]
SD = "±"   # U+00B1


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def msd(mean, sd, nd=3, pct=False):
    if not np.isfinite(mean):
        return "n/a"
    if pct:
        return f"{100 * mean:.1f}{SD}{100 * sd:.1f}" if np.isfinite(sd) else f"{100 * mean:.1f}"
    return f"{mean:.{nd}f}{SD}{sd:.{nd}f}" if np.isfinite(sd) else f"{mean:.{nd}f}"


def fmt(x, nd=3, pct=False):
    if not np.isfinite(x):
        return "n/a"
    return f"{100 * x:.1f}" if pct else f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results_cea_matrix")
    ap.add_argument("--ablation", default="results_cea_ablation")
    ap.add_argument("--timing", nargs="*", default=["results_cea_timing_serial",
                                                    "results_cea_timing_matrix"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    root = Path(args.root)
    out = Path(args.out) if args.out else root / "analysis"
    out.mkdir(parents=True, exist_ok=True)

    # ---------------- serial ctrl_ms (authoritative) ----------------
    serial_rows = []
    for tdir in args.timing:
        p = Path(tdir) / "detail.csv"
        if not p.exists():
            continue
        t = pd.read_csv(p)
        label = Path(tdir).name
        for _, r in t.iterrows():
            serial_rows.append(dict(source=label, scenario=r.scenario, alg=r.alg,
                                    seed=int(r.seed),
                                    ctrl_ms=float(pd.to_numeric(r.ctrl_ms,
                                                                errors="coerce"))))
    sr = pd.DataFrame(serial_rows)
    serial = {}
    if len(sr):
        sr.to_csv(out / "timing_serial_raw.csv", index=False, encoding="utf-8-sig")
        g = sr.groupby(["scenario", "alg"]).ctrl_ms.agg(["mean", "std"])
        for (scen, alg), row in g.iterrows():
            serial[(scen, alg)] = (float(row["mean"]),
                                   float(row["std"]) if np.isfinite(row["std"]) else np.nan)

    # ---------------- main table ----------------
    mt = pd.read_csv(root / "main_table.csv").set_index("alg")
    header = ["Method", "C (%)", "d_min (m)", "TTC_min (s)", "rho", "max_dev (m)",
              "success (%)", "completion (%)", "switch", "ctrl_ms (serial)"]
    rows = []
    for alg in ALGS:
        r = mt.loc[alg]
        ser = [serial[(s, alg)][0] for s in SCENARIOS if (s, alg) in serial]
        ser_sd = [serial[(s, alg)][1] for s in SCENARIOS if (s, alg) in serial]
        cm = np.nanmean(ser) if ser else np.nan
        cs = np.nanmean(ser_sd) if ser_sd else np.nan
        name = f"**{alg}**" if alg == "Ours_rollout" else alg
        rows.append([name,
                     f"{100 * r.C_mean:.2f}{SD}{100 * r.C_std:.2f}",
                     msd(r.d_min_mean, r.d_min_std),
                     msd(r.TTC_min_mean, r.TTC_min_std, nd=2),
                     msd(r.rho_mean, r.rho_std),
                     msd(r.max_dev_mean, r.max_dev_std),
                     f"{100 * r.success_mean:.1f}",
                     f"{100 * r.completion_mean:.1f}",
                     msd(r.switch_mean, r.switch_std, nd=2),
                     msd(cm, cs, nd=2) if np.isfinite(cm) else "see Stage C"])
    (out / "main_table.md").write_text(md_table(header, rows), encoding="utf-8")

    # ---------------- per scenario ----------------
    ps = pd.read_csv(root / "main_table_by_scenario.csv")
    for metric, nd, pct in (("C", 3, True), ("d_min", 3, False),
                            ("completion", 3, True)):
        piv = ps.pivot(index="scenario", columns="alg", values=metric).reindex(SCENARIOS)
        rows = []
        for scen in SCENARIOS:
            if scen not in piv.index:
                continue
            cells = []
            for alg in ALGS:
                v = piv.loc[scen, alg] if alg in piv.columns else np.nan
                cells.append(fmt(v, nd=nd, pct=pct))
            rows.append([scen] + cells)
        (out / f"per_scenario_{metric}.md").write_text(
            md_table(["Scenario"] + ALGS, rows), encoding="utf-8")

    # ---------------- events ----------------
    evp = Path(root) / "events.csv"
    if evp.exists():
        ev = pd.read_csv(evp)
        rows = []
        for scen in SCENARIOS:
            for alg in ALGS:
                q = ev[(ev.scenario == scen) & (ev.alg == alg)]
                if not len(q):
                    continue
                r = q.iloc[0]
                name = f"**{alg}**" if alg == "Ours_rollout" else alg
                rows.append([scen, name,
                             fmt(r.t_risk_on, 1), fmt(r.t_divert_on, 1),
                             fmt(r.t_dist_trigger, 1), fmt(r.ttc_lead, 2),
                             fmt(r.dur_pre, 1), fmt(r.dur_divert, 1),
                             f"{100 * r.preempt_ok:.0f}" if np.isfinite(r.preempt_ok) else "n/a",
                             f"{100 * r.event_order_ok:.0f}" if np.isfinite(r.event_order_ok) else "n/a",
                             int(r.n_collision_seeds)])
        (out / "events.md").write_text(
            md_table(["Scenario", "Method", "t_risk_on", "t_divert_on",
                      "t_dist_trigger", "ttc_lead (s)", "dur_PRE", "dur_DIVERT",
                      "preempt_ok (%)", "event_order_ok (%)", "seeds w/ collision"],
                     rows), encoding="utf-8")

    # ---------------- ablation ----------------
    abp = Path(args.ablation) / "summary.csv"
    if abp.exists():
        ab = pd.read_csv(abp, header=[0, 1], index_col=[0, 1])
        cols = ["collision_rate", "min_dist", "ttc_min", "rho_avg", "max_dev",
                "success", "completion", "switch_count", "ctrl_ms"]
        rows = []
        for scen, alg in ab.index:
            r = ab.loc[(scen, alg)]
            def g(m, s="mean"):
                try:
                    return float(r[(m, s)])
                except (KeyError, TypeError):
                    return np.nan
            name = f"**{alg}**" if alg == "Ours_rollout" else alg
            rows.append([scen, name,
                         f"{100 * g('collision_rate'):.2f}",
                         msd(g("min_dist"), g("min_dist", "std")),
                         msd(g("ttc_min"), g("ttc_min", "std"), nd=2),
                         msd(g("rho_avg"), g("rho_avg", "std")),
                         msd(g("max_dev"), g("max_dev", "std")),
                         f"{100 * g('success'):.1f}",
                         f"{100 * g('completion'):.1f}",
                         msd(g("switch_count"), g("switch_count", "std"), nd=2),
                         msd(g("ctrl_ms"), g("ctrl_ms", "std"), nd=2)])
        (out / "ablation.md").write_text(
            md_table(["Scenario", "Rung", "C (%)", "d_min (m)", "TTC_min (s)",
                      "rho", "max_dev (m)", "success (%)", "completion (%)",
                      "switch", "ctrl_ms (jobs=4)"], rows), encoding="utf-8")

    # ---------------- timing ----------------
    if len(sr):
        rows = []
        for src in sorted(sr.source.unique()):
            for alg in ALGS:
                q = sr[(sr.source == src) & (sr.alg == alg)].ctrl_ms
                if not len(q):
                    continue
                name = f"**{alg}**" if alg == "Ours_rollout" else alg
                rows.append([src, name, len(q), f"{q.mean():.2f}",
                             f"{q.std(ddof=1):.2f}" if len(q) > 1 else "n/a",
                             f"{q.min():.2f}", f"{q.max():.2f}"])
        (out / "timing.md").write_text(
            md_table(["Timing run", "Method", "seeds", "ctrl_ms mean", "sd",
                      "min", "max"], rows), encoding="utf-8")

    # ---------------- deviation vs the author's quoted Linux numbers ----------------
    QUOTED = {("S3", "B2"): 21.76, ("S3", "Ours_rollout"): 12.31,
              ("S2R2", "B2"): 46.67, ("S2R2", "Ours_rollout"): 17.22}
    sp = root / "summary.csv"
    if sp.exists():
        s = pd.read_csv(sp)
        rows, material = [], []
        for (scen, alg), q in QUOTED.items():
            rec = s[(s.scenario == scen) & (s.alg == alg)]
            if not len(rec):
                continue
            m = 100 * float(rec.collision_rate_mean.iloc[0])
            sd = 100 * float(rec.collision_rate_std.iloc[0])
            d = m - q
            flag = "MATERIAL" if abs(d) >= 2.0 else ("close" if abs(d) < 1.0 else "small")
            if abs(d) >= 2.0:
                material.append(f"{scen}/{alg}: {m:.2f}% vs quoted {q:.2f}% ({d:+.2f} pp)")
            name = f"**{alg}**" if alg == "Ours_rollout" else alg
            rows.append([scen, name, f"{m:.2f}{SD}{sd:.2f}", f"{q:.2f}",
                         f"{d:+.2f}", flag])
        (out / "deviation.md").write_text(
            md_table(["Scenario", "Method", "this Windows port C (%)",
                      "author-quoted C (%)", "difference (pp)", "verdict"], rows),
            encoding="utf-8")
        (out / "deviation_material.json").write_text(
            json.dumps(dict(material=material,
                            rule="MATERIAL if |difference| >= 2 percentage points"),
                       indent=2))
        if material:
            print("\n*** MATERIAL DEVIATION FROM THE QUOTED LINUX NUMBERS ***")
            for m in material:
                print("   ", m)

    meta = dict(tables=[p.name for p in sorted(out.glob("*.md"))],
                serial_sources=args.timing)
    (out / "tables_manifest.json").write_text(json.dumps(meta, indent=2))
    print("wrote:", ", ".join(meta["tables"]))
    print("\n" + (out / "main_table.md").read_text(encoding="utf-8"))

if __name__ == "__main__":
    main()
