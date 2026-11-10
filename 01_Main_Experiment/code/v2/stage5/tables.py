"""Every table and number that appears in ``V2_STAGE5_REPORT.md``.

All functions read the Stage-5 CSVs (never the report) and return markdown, so
``reproduce_stage5.py`` can regenerate the report's tables from scratch and
``--check-report`` can diff them against the document.  Each table is emitted
with a ``<!-- REPRO:<name> -->`` marker line; the checker matches on that.

Table names
-----------
``main10``       per-method, 10-scenario equal weight
``mainrisk``     per-method, the 5 risk scenarios equally weighted
``per_scenario`` per (scenario, method)
``ablation``     the four-rung ladder (+ the no-speed-policy probe)
``stats_side``   the two statistical families side by side, 49 rows
``stats_binary`` exact McNemar, seed-level and run-level
``census``       timeouts / failures / infeasibility S-P-C / contacts
``preempt``      RECOVER windows and the two preemption readings
``trec``         the T_REC sweep
``c1``           the C1 audit
``timing``       per-batch wall clock and the projection
``gated``        the fix-1 equivalence proof
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

from v2.stage5 import common, stats


def _f(v, nd=2, dash="—"):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return dash
    if not np.isfinite(x):
        return dash
    return f"{x:.{nd}f}"


def _num(v):
    try:
        x = float(v)
        return x if np.isfinite(x) else np.nan
    except (TypeError, ValueError):
        return np.nan


def _rows(path, **kw):
    path = Path(path)
    if not path.exists():
        return []
    with open(path, encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def matrix(methods=None, root=None):
    """All matrix rows (concatenated per-method CSVs)."""
    return stats.load_matrix(methods=methods, root=root)


def _mean(rows, col, scenes=None):
    v = [_num(r.get(col)) for r in rows
         if scenes is None or r.get("scenario") in scenes]
    v = [x for x in v if np.isfinite(x)]
    return (float(np.mean(v)) if v else np.nan), len(v)


def _cnt(rows, col):
    """Mean of a *counter* column, where an absent value means "never fired".

    The counter bag in ``limits_v2`` only contains keys that were hit, so a
    zero-valued bound is written as an empty CSV field.  Reading that as 0 (and
    not as NaN) is what makes "C1 never binds" visible instead of blank.
    """
    v = [(_num(r.get(col)) if str(r.get(col) or "").strip() else 0.0)
         for r in rows]
    v = [x for x in v if np.isfinite(x)]
    return (float(np.mean(v)) if v else np.nan), len(v)


# ---------------------------------------------------------------------------
# 1. main tables
# ---------------------------------------------------------------------------
def main_table(rows, scenes, title, name):
    hdr = ("| method | C | min clr_so [m] | min TTC [s] | ρ | max_dev/W | "
           "arrival | joint compl. | switches | cycle [ms] | censored |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|"
    out = [f"<!-- REPRO:{name} -->", f"**{title}**", "", hdr, sep]
    for m in common.METHODS:
        sub = [r for r in rows if r.get("method") == m
               and r.get("scenario") in scenes]
        if not sub:
            continue
        C = _mean(sub, "collision_rate")[0]
        clr = _mean(sub, "min_surface_clearance_so")[0]
        ttc = _mean(sub, "ttc_min")[0]
        rho = _mean(sub, "rho_avg")[0]
        dev = _mean(sub, "max_dev_norm")[0]
        arr = _mean(sub, "arrival")[0]
        joint = _mean(sub, "joint_completion")[0]
        sw, n_sw = _mean(sub, "switch_count")
        cyc, n_cyc = _mean(sub, "ctrl_ms")
        cens = sum(1 for r in sub if int(_num(r.get("censored")) or 0))
        sw_s = _f(sw, 2) if n_sw else "n/a"
        out.append(f"| {m} | {_f(C, 3)} | {_f(clr)} | {_f(ttc)} | {_f(rho, 3)} "
                   f"| {_f(dev, 3)} | {_f(arr, 3)} | {_f(joint, 3)} | {sw_s} "
                   f"| {_f(cyc)} | {cens}/{len(sub)} |")
    out.append("")
    return "\n".join(out)


def per_scenario_table(rows):
    hdr = ("| scenario | method | C | min clr_so [m] | min TTC [s] | ρ | "
           "max_dev/W | arrival | joint compl. |")
    sep = "|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:per_scenario -->", hdr, sep]
    for s in common.SCENARIOS:
        for m in common.METHODS:
            sub = [r for r in rows
                   if r.get("method") == m and r.get("scenario") == s]
            if not sub:
                continue
            out.append(
                f"| {s} | {m} | {_f(_mean(sub, 'collision_rate')[0], 3)} "
                f"| {_f(_mean(sub, 'min_surface_clearance_so')[0])} "
                f"| {_f(_mean(sub, 'ttc_min')[0])} "
                f"| {_f(_mean(sub, 'rho_avg')[0], 3)} "
                f"| {_f(_mean(sub, 'max_dev_norm')[0], 3)} "
                f"| {_f(_mean(sub, 'arrival')[0], 3)} "
                f"| {_f(_mean(sub, 'joint_completion')[0], 3)} |")
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# 2. ablation
# ---------------------------------------------------------------------------
def ablation_table():
    rows = _rows(common.ABL_DIR / "detail.csv")
    if not rows:
        return "<!-- REPRO:ablation -->\n(no ablation data)\n"
    hdr = ("| scene | rung | C | min clr_so [m] | min TTC [s] | ρ | max_dev/W "
           "| arrival | joint compl. | switches | bounded ticks |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:ablation -->", hdr, sep]
    rungs = common.ABLATION_RUNGS + (["R3_no_speed_policy"]
                                     if any(r.get("rung") ==
                                            "R3_no_speed_policy" for r in rows)
                                     else [])
    for s in common.ABLATION_SCENARIOS:
        for rg in rungs:
            sub = [r for r in rows
                   if r.get("rung") == rg and r.get("scenario") == s]
            if not sub:
                continue
            sw, n_sw = _mean(sub, "switch_count")
            sp, n_sp = _mean(sub, "sp_n_bounded_ticks")
            out.append(
                f"| {s} | {rg} | {_f(_mean(sub, 'collision_rate')[0], 3)} "
                f"| {_f(_mean(sub, 'min_surface_clearance_so')[0])} "
                f"| {_f(_mean(sub, 'ttc_min')[0])} "
                f"| {_f(_mean(sub, 'rho_avg')[0], 3)} "
                f"| {_f(_mean(sub, 'max_dev_norm')[0], 3)} "
                f"| {_f(_mean(sub, 'arrival')[0], 3)} "
                f"| {_f(_mean(sub, 'joint_completion')[0], 3)} "
                f"| {(_f(sw, 1) if n_sw else 'n/a')} "
                f"| {(_f(sp, 1) if n_sp else '—')} |")
    # paired deltas vs R0
    out += ["", "| scene | contrast | ΔC | Δmin clr_so [m] | Δρ | "
                "Δmax_dev/W | Δarrival |", "|---|---|---|---|---|---|---|"]
    for s in common.ABLATION_SCENARIOS:
        for rg in rungs[1:]:
            a = {int(r["seed"]): r for r in rows
                 if r.get("rung") == rg and r.get("scenario") == s}
            b = {int(r["seed"]): r for r in rows
                 if r.get("rung") == "R0_four_state" and r.get("scenario") == s}
            keys = sorted(set(a) & set(b))
            if not keys:
                continue
            d = lambda col: float(np.nanmean(  # noqa: E731
                [_num(a[k].get(col)) - _num(b[k].get(col)) for k in keys]))
            out.append(f"| {s} | {rg} − R0 | {_f(d('collision_rate'), 3)} "
                       f"| {_f(d('min_surface_clearance_so'))} "
                       f"| {_f(d('rho_avg'), 3)} "
                       f"| {_f(d('max_dev_norm'), 3)} "
                       f"| {_f(d('arrival'), 3)} |")
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# 3. statistics
# ---------------------------------------------------------------------------
def stats_tables(baselines=None):
    rows = matrix()
    baselines = baselines or [m for m in common.METHODS
                              if m != "Ours_rollout"]
    seed = stats.apply_holm(stats.continuous_family(rows, baselines,
                                                    mode="seed",
                                                    weight="all"))
    run = stats.apply_holm(stats.continuous_family(rows, baselines,
                                                   mode="run", weight="all"))
    comp = stats.compare_families(seed, run)
    hdr = ("| endpoint | baseline | n | Δ median (Ours − base) | "
           "p (seed) | p_Holm (seed) | p (run) | p_Holm (run) | better | "
           "seed | run | disagree |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:stats_side -->",
           ("Paired Wilcoxon signed-rank, two-sided.  *seed* = the primary "
            "family (n = 30 seeds, scenes averaged within seed, family size "
            f"**{stats.family_sizes()['continuous_seed']}**); *run* = the "
            "descriptive run-level family (n = 300, pseudoreplication).  "
            "Δ median < 0 favours Ours when *better* = lower."),
           "", hdr, sep]
    for r in comp:
        out.append(
            f"| {r['endpoint']} | {r['baseline']} | {r['n_seed']} "
            f"| {_f(r['med_seed'], 3)} | {_f(r['p_seed'], 4)} "
            f"| {_f(r['p_seed_holm'], 4)} | {_f(r['p_run'], 4)} "
            f"| {_f(r['p_run_holm'], 4)} | {r['better']} "
            f"| {'yes' if r['sig_seed'] else 'no'} "
            f"| {'yes' if r['sig_run'] else 'no'} "
            f"| {'**yes**' if r['disagree'] else ''} |")
    out.append("")
    # sensitivity: risk-scenario weighting, seed family, uncorrected p
    risk = stats.continuous_family(rows, baselines, mode="seed",
                                   weight="risk")
    out += ["<!-- REPRO:stats_risk -->",
            ("Sensitivity: the same seed-clustered tests with the five risk "
             "scenarios equally weighted (uncorrected p, Holm not applied)."),
            "", "| endpoint | baseline | n | Δ median | p (seed, risk) |",
            "|---|---|---|---|---|"]
    for r in risk:
        out.append(f"| {r['endpoint']} | {r['baseline']} | {r['n']} "
                   f"| {_f(r['median_diff'], 3)} | {_f(r['p'], 4)} |")
    out.append("")
    return "\n".join(out), seed, run, comp, risk


def binary_table(baselines=None):
    rows = matrix()
    baselines = baselines or [m for m in common.METHODS
                              if m != "Ours_rollout"]
    s10 = stats.apply_holm(stats.binary_family(rows, baselines, mode="all10"))
    srun = stats.apply_holm(stats.binary_family(rows, baselines, mode="run"))
    srun_idx = {(r["endpoint"], r["baseline"]): r for r in srun}
    hdr = ("| endpoint | baseline | seed n01/n10 | p_Holm (seed, all-10) | "
           "run n01/n10 | p_Holm (run, 300 pairs) |")
    sep = "|---|---|---|---|---|---|"
    out = ["<!-- REPRO:stats_binary -->", hdr, sep]
    for r in s10:
        u = srun_idx[(r["endpoint"], r["baseline"])]
        out.append(f"| {r['endpoint']} | {r['baseline']} | "
                   f"{r['n01']}/{r['n10']} | {_f(r['p_holm'], 4)} | "
                   f"{u['n01']}/{u['n10']} | {_f(u['p_holm'], 4)} |")
    out += ["",
            "n01 = baseline-only successes, n10 = Ours-only successes "
            "(exact McNemar, two-sided binomial on the discordant pairs)."]
    out.append("")
    return "\n".join(out), s10, srun


# ---------------------------------------------------------------------------
# 4. census
# ---------------------------------------------------------------------------
def census_table():
    rows = matrix()
    hdr = ("| method | runs | goal | time | censored | errors | infeasible "
           "events | S | P | C | clr_rs [m] | clr_rr [m] | contacts ro/rs/rr | "
           "C1lo bind | C1hi bind |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:census -->", hdr, sep]
    for m in common.METHODS:
        sub = [r for r in rows if r.get("method") == m]
        if not sub:
            continue
        ended = [str(r.get("terminated_by") or "") for r in sub]
        inf = _mean(sub, "n_infeasible")[0]
        S = _mean(sub, "n_infeasible_S")[0]
        P = _mean(sub, "n_infeasible_P")[0]
        Cc = _mean(sub, "n_contact_C")[0]
        ro = _mean(sub, "n_contact_events_ro")[0]
        rs = _mean(sub, "n_contact_events_rs")[0]
        rr = _mean(sub, "n_contact_events_rr")[0]
        c1l = _cnt(sub, "bind_C1_low_sheep")[0]
        c1h = _cnt(sub, "bind_C1_high_sheep")[0]
        out.append(
            f"| {m} | {len(sub)} | {ended.count('goal')} | "
            f"{ended.count('time')} | "
            f"{sum(1 for r in sub if int(_num(r.get('censored')) or 0))} | "
            f"{sum(1 for r in sub if str(r.get('run_error') or '').strip())} "
            f"| {_f(inf, 1)} | {_f(S, 1)} | {_f(P, 1)} | {_f(Cc, 2)} | "
            f"{_f(_mean(sub, 'min_surface_clearance_rs')[0])} | "
            f"{_f(_mean(sub, 'min_surface_clearance_rr')[0])} | "
            f"{_f(ro, 1)}/{_f(rs, 1)}/{_f(rr, 1)} | {_f(c1l, 1)} | "
            f"{_f(c1h, 1)} |")
    out += ["",
            "All counts are **per run**, averaged over the 300 runs of the "
            "method; `goal`/`time` are termination classes and every run is in "
            "exactly one of them.  `infeasible events` is the number of "
            "hard-breach interventions by the execution layer; those events "
            "split into **resolved**, **S** (solver-infeasible: no candidate in "
            "the finite admissible set was safe and the unavoidability "
            "certificate did not apply — *not* a claim of physical "
            "unavoidability) and **P** (physically unavoidable under the stated "
            "assumptions, a sufficient certificate and therefore a lower bound).  "
            "**C** is a post-hoc control failure diagnosed once per contact "
            "episode by a myopic oracle, so it is a lower bound too; it is "
            "counted on animal-animal, robot-obstacle and robot-animal contacts "
            "alike, which is why it is larger than the `C` safety endpoint "
            "(animal-obstacle only).  `clr_rs` is the worst robot-animal surface "
            "clearance: **negative for every method that actually herds** "
            "(Ours, B1, B2, B-Outmost, B-PCBF) and positive only for the VH "
            "family, which stays away from the flock and therefore does not do "
            "the task — see §5.  `C1lo bind`/`C1hi bind` are counter columns: 0 "
            "means the bound never clipped, and for C1-low that is structural "
            "(§1.3), not a pass."]
    out.append("")
    return "\n".join(out)


def preempt_table():
    rows = matrix()
    hdr = ("| method | runs with RECOVER | windows | closed | preempted | "
           "open at end | preempt_ctrl = True | preempt_truth = True | "
           "legacy = True |")
    sep = "|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:preempt -->", hdr, sep]
    for m in common.METHODS:
        sub = [r for r in rows if r.get("method") == m]
        if not sub:
            continue
        n_win = sum(int(_num(r.get("n_recover_windows")) or 0) for r in sub)
        n_cl = sum(int(_num(r.get("n_recover_complete")) or 0) for r in sub)
        n_pr = sum(int(_num(r.get("n_recover_preempted")) or 0) for r in sub)
        n_op = sum(int(_num(r.get("n_recover_censored")) or 0) for r in sub)
        with_rec = sum(1 for r in sub
                       if int(_num(r.get("n_recover_windows")) or 0) > 0)
        ctrl = sum(1 for r in sub if _num(r.get("preempt_ctrl")) == 1)
        truth = sum(1 for r in sub if _num(r.get("preempt_truth")) == 1)
        legacy = sum(1 for r in sub if _num(r.get("preempt_legacy")) == 1)
        out.append(f"| {m} | {with_rec} | {n_win} | {n_cl} | {n_pr} | {n_op} "
                   f"| {ctrl} | {truth} | {legacy} |")
    out += ["",
            "Preemption is only *defined* for a method that has a RECOVER "
            "state; the baselines have none, so their zeros are structural.  "
            "`preempt_ctrl` is the paper's reading (controller information "
            "set), `preempt_truth` the truth-based sensitivity variant, "
            "`legacy` the frozen `metrics.preempt_ok` column (kept for "
            "provenance only — it is **not** a preemption statistic)."]
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# 5. fix 2 (T_REC) and fix 3 (C1)
# ---------------------------------------------------------------------------
def trec_table():
    rows = _rows(common.TREC_DIR / "trec_sweep_summary.csv")
    if not rows:
        return "<!-- REPRO:trec -->\n(no T_REC sweep data)\n"
    hdr = ("| scene | T_REC [s] | windows | closed | preempted | open at end | "
           "median closed dur [s] | max closed dur [s] | median re-entry delay "
           "[s] | C | min clr_so [m] | ρ | arrival | goal time [s] |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:trec -->", hdr, sep]
    for r in rows:
        out.append(
            f"| {r['scene']} | {_f(r['t_rec'], 1)} | {r['n_windows']} "
            f"| {r['n_closed']} | {r['n_preempted']} | {r['n_open_at_end']} "
            f"| {_f(r['dur_med'])} | {_f(r['dur_max'])} "
            f"| {_f(r['delay_med'])} | {_f(r['C'], 4)} "
            f"| {_f(r['clr_so'])} | {_f(r['rho'], 3)} "
            f"| {_f(r['arrival'], 2)} | {_f(r['exec_time'], 1)} |")
    rule = {}
    p = common.TREC_DIR / "trec_rule.json"
    if p.exists():
        rule = json.loads(p.read_text(encoding="utf-8")).get("rule", {})
    if rule:
        out += ["",
                f"Rule R3 on the closed windows of V2S3 at the frozen 15 s: "
                f"{rule.get('n_closed_episodes')} episodes, median "
                f"{_f(rule.get('median_closed_duration'), 2)} s → "
                f"T_REC = {_f(rule.get('t_rec_rule'), 1)} s."]
    out.append("")
    return "\n".join(out)


def c1_table():
    p = common.C1_DIR / "C1_audit.json"
    if not p.exists():
        return "<!-- REPRO:c1 -->\n(no C1 audit data)\n"
    d = json.loads(p.read_text(encoding="utf-8"))
    out = ["<!-- REPRO:c1 -->",
           "| configuration | a_sheep [m/s²] | ω_sheep [rad/s] | C1-low band "
           "s_lo [m/s] | binding cells |", "|---|---|---|---|---|"]
    for g in d.get("grid_summary", []):
        out.append(f"| {g['grid']} | {g['a_s']} | {g['omega_s']} "
                   f"| {g['s_lo']} | {g['n_bind']} |")
    sh = [r for r in d.get("runs", []) if r.get("kind") == "shipped"]
    st = [r for r in d.get("runs", []) if r.get("kind") == "stress"]
    out += ["",
            "| runs | config | bind_C1_low (total) | violation_C1_low | C | ρ |",
            "|---|---|---|---|---|---|"]
    for name, sub in (("shipped", sh), ("stress", st)):
        if not sub:
            continue
        out.append(f"| {name} | `{sub[0]['config_id']}` | "
                   f"{sum(int(r['bind_C1_low'] or 0) for r in sub)} | "
                   f"{sum(int(r['viol_C1_low'] or 0) for r in sub)} | "
                   f"{_f(np.mean([float(r['C']) for r in sub]), 4)} | "
                   f"{_f(np.mean([float(r['rho']) for r in sub]), 3)} |")
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# 6. timing
# ---------------------------------------------------------------------------
def timing_table():
    p = common.MATRIX_DIR / "_formal_batches.json"
    if not p.exists():
        p = common.MATRIX_DIR / "_batches.json"
    if not p.exists():
        return "<!-- REPRO:timing -->\n(no batch manifest)\n"
    d = json.loads(p.read_text(encoding="utf-8"))
    out = ["<!-- REPRO:timing -->",
           "| batch (method) | cells | wall [min] | s/cell | censored | "
           "errors |", "|---|---|---|---|---|---|"]
    tot = 0.0
    for b in d.get("batches", []):
        w = float(b.get("wall_s") or 0.0)
        tot += w
        out.append(f"| {b['method']} | {b['n']} | {w / 60:.1f} "
                   f"| {_f(b.get('s_per_cell'))} | {b.get('n_censored', '')} "
                   f"| {b.get('n_error', '')} |")
    out += ["",
            f"Measured total for the 2400-run matrix at `jobs = 12`: "
            f"**{tot / 3600:.2f} h** ({tot:.0f} s).  Per-cell wall clocks are "
            f"measured under 12-way concurrency; the clean serial per-cycle "
            f"cost is the `cycle [ms]` column of the timing sample "
            f"(`checks/speedup.csv`).", ""]
    sp = _rows(common.CHECK_DIR / "speedup.csv")
    if sp:
        out += ["| probe | runs | jobs | wall [s] | s/run | speed-up | "
                "efficiency |", "|---|---|---|---|---|---|---|"]
        for r in sp:
            out.append(f"| {r['cell']} | {r['n_runs']} | {r['jobs']} "
                       f"| {_f(r['wall_s'], 1)} | {_f(r['s_per_run'])} "
                       f"| {_f(r['speedup'], 2)}× | {_f(r['efficiency'], 2)} "
                       f"|")
        out.append("")
    return "\n".join(out)


def gated_table():
    rows = _rows(common.CHECK_DIR / "G_gated_ttc.csv")
    if not rows:
        return "<!-- REPRO:gated -->\n(no probe data)\n"
    hdr = ("| cell | ticks | pairs | max \\|pub − truth\\| pos [m] | "
           "max \\|TTC_pub − TTC_truth\\| [s] | max \\|TTC_logged − "
           "TTC_truth\\| (near band) [s] | onset shift [s] | preempt ctrl / "
           "truth(log) / truth(instant) / legacy |")
    sep = "|---|---|---|---|---|---|---|---|"
    out = ["<!-- REPRO:gated -->", hdr, sep]
    for r in rows:
        out.append(f"| {r['scenario']}/{r['method']}/{r['seed']} "
                   f"| {r['n_ticks']} | {r['n_pairs']} "
                   f"| {float(r['max_pub_pos_err']):.2e} "
                   f"| {float(r['max_ttc_pub_vs_truth_instant']):.2e} "
                   f"| {_f(r['max_ttc_logged_vs_truth_near_band'])} "
                   f"| {_f(r['max_onset_shift_s'])} "
                   f"| {r['preempt_ctrl']} / {r['preempt_truth_log']} / "
                   f"{r['preempt_truth_instant']} / {r['preempt_legacy']} |")
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# 7. everything, as a dict name -> markdown
# ---------------------------------------------------------------------------
def build_all():
    rows = matrix()
    tabs = {}
    tabs["main10"] = main_table(rows, common.SCENARIOS,
                               "All ten scenarios, equally weighted", "main10")
    tabs["mainrisk"] = main_table(rows, common.RISK_SCENARIOS,
                                 "The five risk scenarios, equally weighted",
                                 "mainrisk")
    tabs["per_scenario"] = per_scenario_table(rows)
    tabs["ablation"] = ablation_table()
    side, seed, run, comp, risk = stats_tables()
    tabs["stats_side"] = side
    bt, s10, srun = binary_table()
    tabs["stats_binary"] = bt
    tabs["census"] = census_table()
    tabs["preempt"] = preempt_table()
    tabs["trec"] = trec_table()
    tabs["c1"] = c1_table()
    tabs["timing"] = timing_table()
    tabs["gated"] = gated_table()
    return tabs, dict(rows=rows, seed=seed, run=run, comp=comp, risk=risk,
                      bin_seed=s10, bin_run=srun)


def write_all(out_dir=None):
    out_dir = Path(out_dir or common.REPRO_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    tabs, extra = build_all()
    for name, md in tabs.items():
        (out_dir / f"{name}.md").write_text(md, encoding="utf-8")
    return tabs, extra


if __name__ == "__main__":
    tabs, extra = write_all()
    print(f"wrote {len(tabs)} tables to {common.REPRO_DIR}")
    for k, v in tabs.items():
        print(f"  {k:14s} {len(v.splitlines()):4d} lines")
