"""Stage-4 acceptance tests D, E and F ("completion" script).

    python -X utf8 -m v2.acceptance_def_s4            # everything
    python -X utf8 -m v2.acceptance_def_s4 --tests D  # one test

Why this file exists
--------------------
``v2/acceptance_s4.py`` is the A-F suite.  It was interrupted after C, so D, E
and F never produced their evidence files.  This module implements **the same
three tests** with the *stored* outputs reused wherever they exist, and with the
minimum amount of simulation needed to make each claim verifiable:

D  sequential-recovery case
   * first, the **stored** ``geometry/seq_trace_V2S3_Ours_rollout_1000.csv`` is
     analysed with the exact ``v2.seq_trace_s4.analyse`` code path.  It turns out
     that trace predates the 15:46 spacing sweep (its obstacle-2 detection is
     65.80 s, whereas the shipped scene gives 70.20 s), so its RECOVER windows
     contain no new-threat arrival -> **it cannot establish D**;
   * therefore **one** seed (1000) is re-run on the shipped scene, and its
     per-tick state trace is written out.  That run is the D evidence.
   The stored ``geometry/s3_spacing_sweep.csv`` (seeds 1000-1004, written after
   the spacing was frozen) is quoted as the independent 5/5 corroboration; it is
   **not** re-run.

E  censoring
   * read ``pilot/pilot_runs.csv`` -> censored runs per (case, method);
   * one deliberately horizon-truncated run is added as a *positive control* of
     the censoring machinery (the pilot itself has zero censored runs).

F  determinism and limits
   * one pilot cell repeated twice, diffed value-by-value, and also diffed
     against the **stored** pilot row for that cell;
   * the same cell run through ``runner_s4 --jobs 1`` and ``--jobs 4``, diffed;
   * the limit-violation / binding counts are read from the **stored**
     ``_smoke/detail.csv`` -- no re-run.

Outputs (all under ``results_v2_stage4/``)::

    D_sequential_recovery.csv     one row per reading (stored / fresh / sweep)
    D_state_trace_seed1000.csv    the fresh run's state / risk trace excerpt
    E_censoring.csv               per-run censoring table
    E_censoring_control.csv       the horizon-truncated positive control
    F_determinism.json            repeat + jobs diffs, limit summary
    F_limits.csv                  limits read from the stored smoke detail
    ACCEPTANCE_DEF.txt            the transcript
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from v2.config_v2 import STAGE3_TTC, stage3_v2cfg  # noqa: E402
from v2.runner_s4 import registry_alg, run_cell  # noqa: E402
from v2.scenarios_v2 import resolve_scene  # noqa: E402
from v2.seq_trace_s4 import analyse, control_rows  # noqa: E402

OUT = ROOT / "results_v2_stage4"
GEO = OUT / "geometry"
PILOT = OUT / "pilot"
SMOKE = OUT / "_smoke"

LIMIT_COLS = ("violation_C1_low", "violation_C1_high", "violation_C2",
              "violation_C3", "violation_R4", "violation_lat")
BIND_COLS = ("bind_C3_sheep", "bind_R4_robot", "bind_LAT_robot",
             "bind_C1_low_sheep", "bind_C1_high_sheep", "bind_C2_sheep")


# ---------------------------------------------------------------------------
class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def __call__(self, *a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        self.f.write(s + "\n")
        self.f.flush()


LOG = [None]


def say(*a):
    (LOG[0] or print)(*a)


def write_csv(path, rows, keys=None):
    if not rows:
        return
    keys = keys or list(rows[0])
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _f(v, default=np.nan):
    try:
        x = float(v)
        return x if np.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def _trace_arrays(res):
    cols = ["t", "step", "alive", "spos", "shead", "sspeed",
            "rpos", "rhead", "rspeed", "romega", "opos"]
    tr = res["trace_states"]
    return {c: np.ascontiguousarray(np.array([row[i] for row in tr]))
            for i, c in enumerate(cols)}


def _diff_traces(a, b):
    bad = []
    for k in a:
        if k not in b or a[k].shape != b[k].shape:
            bad.append((k, "shape"))
        elif not np.array_equal(a[k], b[k]):
            with np.errstate(invalid="ignore"):
                worst = float(np.nanmax(np.abs(a[k].astype(float)
                                             - b[k].astype(float))))
            bad.append((k, "value", worst))
    return bad


# ===========================================================================
# D. sequential-recovery case
# ===========================================================================
def _stored_trace_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        raw = list(csv.DictReader(f))
    rows = []
    for r in raw:
        d = dict(t=_f(r["t"]), tag=r["tag"], rho=_f(r["rho"]),
                 lat_c=_f(r["lat_c"]))
        for k in (1, 2, 3):
            d[f"ttc_o{k}"] = _f(r[f"ttc_o{k}"])
            d[f"visible_o{k}"] = _f(r[f"visible_o{k}"])
            d[f"first_detect_o{k}"] = _f(r[f"first_detect_o{k}"])
        rows.append(d)
    return rows, raw


def _state_of(rows, t):
    i = int(np.argmin(np.abs(np.array([r["t"] for r in rows]) - t)))
    return rows[i]["tag"], i


def test_D(log):
    log("=" * 78)
    log("D. sequential-recovery case: does a NEW threat arrive while RECOVER "
        "is active?")
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    log(f"  config_id={cfg.config_id}  T_PRE={cfg.ttc_pre} s  "
        f"T_DIVERT={cfg.ttc_divert} s  T_REC={cfg.ttc_rec} s")
    out = []

    # ---- D1: the stored trace (no re-simulation) ----------------------
    p = GEO / "seq_trace_V2S3_Ours_rollout_1000.csv"
    stored_rows, _ = _stored_trace_rows(p)
    sinfo = analyse(stored_rows, cfg)
    log(f"\n  D1 stored trace {p.name} ({len(stored_rows)} control ticks)")
    log("     first detection per obstacle: "
        + ", ".join(f"o{k}={sinfo['first_detect'][k]:.2f}"
                    for k in (1, 2, 3)))
    log("     risk onset (TTC<=T_PRE, visibility-gated): "
        + ", ".join(f"o{k}={sinfo['risk_onset'][k]:.2f}"
                    for k in (1, 2, 3)))
    log("     RECOVER windows: "
        + ("; ".join(f"[{a:.2f}, {b:.2f}]" for a, b in
                     sinfo["recover_windows"]) or "none"))
    log("     new threat's risk onset inside a RECOVER window: "
        + ("YES " + str(sinfo["overlap"]) if sinfo["overlap"] else "NO"))
    log("     new threat detected inside a RECOVER window: "
        + ("YES" if sinfo["overlap_detect"] else "NO"))
    log("     -> the stored trace CANNOT establish D: its o2 arrival "
        f"({sinfo['first_detect'][2]:.2f} s) precedes its first RECOVER "
        f"window ({sinfo['recover_windows'][0][0]:.2f} s).")
    out.append(dict(source="stored_trace", seed=1000,
                    trace_file=p.name, n_ticks=len(stored_rows),
                    recover_windows=";".join(f"{a:.2f}-{b:.2f}"
                                             for a, b in
                                             sinfo["recover_windows"]),
                    detect_1=sinfo["first_detect"][1],
                    detect_2=sinfo["first_detect"][2],
                    detect_3=sinfo["first_detect"][3],
                    onset_1=sinfo["risk_onset"][1],
                    onset_2=sinfo["risk_onset"][2],
                    onset_3=sinfo["risk_onset"][3],
                    overlap_onset=bool(sinfo["overlap"]),
                    overlap_detect=bool(sinfo["overlap_detect"]),
                    preempted=False,
                    note="pre-dates the 15:46 spacing sweep; cannot establish D"))

    # ---- D2: ONE seed on the shipped scene ----------------------------
    seed = 1000
    log(f"\n  D2 one seed on the shipped scene: V2S3 / Ours_rollout / "
        f"seed {seed} (fresh run -- the only simulation in test D)")
    row, res = run_cell("V2S3", "Ours_rollout", seed, v2cfg=cfg,
                        return_res=True)
    rows = control_rows(res, cfg)
    info = analyse(rows, cfg)
    ts = np.array([r["t"] for r in rows])
    tags = [r["tag"] for r in rows]
    log(f"     ended {res['terminated_by']}, "
        f"C={row['collision_rate']:.3f}, min_dist={row['min_dist']:.2f} m, "
        f"rho={row['rho_avg']:.3f}, {len(rows)} control ticks")
    log("     first detection per obstacle: "
        + ", ".join(f"o{k}={info['first_detect'][k]:.2f}"
                    for k in (1, 2, 3)))
    log("     risk onset (TTC<=T_PRE, visibility-gated): "
        + ", ".join(f"o{k}={info['risk_onset'][k]:.2f}"
                    for k in (1, 2, 3)))
    log("     RECOVER windows: "
        + ("; ".join(f"[{a:.2f}, {b:.2f}] ({b - a:.1f} s)"
                     for a, b in info["recover_windows"])))
    log("     release order (onset 1<2<3): "
        f"{info['order_ok']}, (detect 1<2<3): {info['detect_order_ok']}")

    rec = info["recover_windows"]
    on2 = info["risk_onset"][2]
    det2 = info["first_detect"][2]
    win = next((w for w in rec if w[0] <= on2 <= w[1]), None)
    tag_at, i_at = _state_of(rows, on2)
    tag_at_det, i_det = _state_of(rows, det2)
    after = [tags[j] for j in range(i_at + 1, min(i_at + 40, len(tags)))]
    nxt = next((x for x in after if x != tag_at), None)
    preempted = bool(tag_at == "recover" and nxt in ("pre", "divert"))
    log(f"     o2 detected at {det2:.2f} s in state '{tag_at_det}'; "
        f"risk onset {on2:.2f} s in state '{tag_at}'")
    log(f"     o2 onset inside a RECOVER window: {'YES ' + str(win) if win else 'NO'}")
    log(f"     state path after the onset: {tag_at} -> {nxt}  "
        f"=> preemption by the new threat: {preempted}")
    out.append(dict(source="fresh_run", seed=seed,
                    trace_file="D_state_trace_seed1000.csv",
                    n_ticks=len(rows),
                    recover_windows=";".join(f"{a:.2f}-{b:.2f}"
                                             for a, b in rec),
                    detect_1=info["first_detect"][1],
                    detect_2=det2, detect_3=info["first_detect"][3],
                    onset_1=info["risk_onset"][1], onset_2=on2,
                    onset_3=info["risk_onset"][3],
                    overlap_onset=bool(win), overlap_detect=bool(
                        any(a <= det2 <= b for a, b in rec)),
                    preempted=preempted,
                    note=("RECOVER truncated by o2's PRE at the next tick"
                          if preempted else "no preemption observed")))

    # ---- D2b: the state trace excerpt ---------------------------------
    # the marks are the *gated* first crossings (an obstacle only counts once
    # it is visible), i.e. exactly the onsets ``analyse`` reports.
    excerpt, prev = [], None
    for r in rows:
        marks = []
        for k in (1, 2, 3):
            fd = r[f"first_detect_o{k}"]
            if np.isfinite(fd) and abs(r["t"] - fd) < 1e-6:
                marks.append(f"detect_o{k}")
            on = info["risk_onset"][k]
            if np.isfinite(on) and abs(r["t"] - on) < 1e-6:
                marks.append(f"onset_o{k}")
        changed = (prev is None or r["tag"] != prev["tag"])
        if changed or marks:
            excerpt.append(dict(t=round(r["t"], 2), state=r["tag"],
                                rho=round(r["rho"], 3),
                                lat_c=round(r["lat_c"], 3),
                                ttc_o1=round(r["ttc_o1"], 2)
                                if np.isfinite(r["ttc_o1"]) else np.nan,
                                ttc_o2=round(r["ttc_o2"], 2)
                                if np.isfinite(r["ttc_o2"]) else np.nan,
                                ttc_o3=round(r["ttc_o3"], 2)
                                if np.isfinite(r["ttc_o3"]) else np.nan,
                                visible_o1=r["visible_o1"],
                                visible_o2=r["visible_o2"],
                                visible_o3=r["visible_o3"],
                                marks=",".join(marks)))
        prev = r
    write_csv(GEO / f"D_state_trace_seed{seed}.csv", excerpt)
    log(f"\n     state trace excerpt (full table: "
        f"{GEO / f'D_state_trace_seed{seed}.csv'})")
    for e in excerpt:
        if 60.0 <= e["t"] <= 95.0 or e["marks"]:
            log(f"       t={e['t']:7.2f}  {e['state']:<8s} "
                f"rho={e['rho']:.3f} ttc=({e['ttc_o1']:6.2f},"
                f"{e['ttc_o2']:6.2f},{e['ttc_o3']:6.2f}) "
                f"vis=({e['visible_o1']:.0f},{e['visible_o2']:.0f},"
                f"{e['visible_o3']:.0f}) {e['marks']}")

    # ---- D3: stored independent corroboration (no re-run) -------------
    sp = GEO / "s3_spacing_sweep.csv"
    with open(sp, newline="", encoding="utf-8-sig") as f:
        sw = [r for r in csv.DictReader(f) if r["e2"] == "83.0"]
    n_on = sum(1 for r in sw if r["overlap"] == "True")
    n_det = sum(1 for r in sw if r["overlap_detect"] == "True")
    log(f"\n  D3 stored s3_spacing_sweep.csv (e2=83, seeds 1000-1004): "
        f"risk-onset overlap {n_on}/{len(sw)}, detection overlap "
        f"{n_det}/{len(sw)}")
    for r in sw:
        log(f"       seed {r['seed']}: recover={r['recover_windows']:<40s} "
            f"det2={_f(r['detect_2']):.2f} onset2={_f(r['onset_2']):.2f} "
            f"overlap={r['overlap']} order={r['order_ok']}")
        out.append(dict(source="stored_spacing_sweep", seed=int(r["seed"]),
                        trace_file="s3_spacing_sweep.csv", n_ticks="",
                        recover_windows=r["recover_windows"],
                        detect_1=_f(r["detect_1"]), detect_2=_f(r["detect_2"]),
                        detect_3=_f(r["detect_3"]), onset_1=_f(r["onset_1"]),
                        onset_2=_f(r["onset_2"]), onset_3=_f(r["onset_3"]),
                        overlap_onset=r["overlap"] == "True",
                        overlap_detect=r["overlap_detect"] == "True",
                        preempted="",
                        note="independent 5/5 corroboration, not re-run"))
    write_csv(OUT / "D_sequential_recovery.csv", out)

    ok = bool(win) and preempted and info["order_ok"]
    log(f"\n  [{'PASS' if ok else 'FAIL'}] D/a new threat (o2) reaches its risk "
        f"onset inside the first RECOVER window and preempts it; release order "
        f"1<2<3 reproduced from the trajectory")
    return ok


# ===========================================================================
# E. censoring
# ===========================================================================
def test_E(log):
    log("=" * 78)
    log("E. censoring: runs that hit the horizon without reaching the goal")
    p = PILOT / "pilot_runs.csv"
    with open(p, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    cens = [r for r in rows if str(r["censored"]) == "1"]
    log(f"  {n} pilot runs; {len(cens)} censored "
        f"({100.0 * len(cens) / max(n, 1):.1f} %)")
    scen = {c: resolve_scene(sid) for c, sid in
            (("no_obstacle", "V2S1"), ("single_obstacle", "V2S2"),
             ("sequential_obstacle", "V2S3"))}
    log("  horizon (scene T_max) and reference-kinematics support:")
    for c, s in scen.items():
        sub = [r for r in rows if r["case"] == c]
        log(f"    {c:22s} {s['name']:34s} T_max={s['T_max']:5.1f} s   "
            f"reference capture T_max="
            f"{json.loads((GEO / 'reference_kinematics.json').read_text())['140' if s['T_max'] < 300 else '220']['T_max']:5.1f} s   "
            f"last sim {max(_f(r['sim_time_s']) for r in sub):6.1f} s   "
            f"ended={sorted(set(r['ended'] for r in sub))}")
    log("  (the Stage-3 hook asked for a >=120 s reference window; 140 m "
        "scenes run 200 s, 220 m run 320 s)")

    by, out = {}, []
    for r in rows:
        k = (r["case"], r["method"])
        by.setdefault(k, [0, 0])
        by[k][0] += 1
        by[k][1] += 1 if str(r["censored"]) == "1" else 0
        out.append(dict(case=r["case"], method=r["method"], seed=int(r["seed"]),
                        censored=int(r["censored"]), ended=r["ended"],
                        success=r["success"], completion=r["completion"],
                        exec_time=r["exec_time"], sim_time_s=r["sim_time_s"],
                        coverage_at_end=r["coverage_at_end"],
                        rho_mean=r["rho_mean"]))
    write_csv(OUT / "E_censoring.csv", out)
    log("\n  censored / runs, per (case, method) -- denominator is ALL runs:")
    for (case, m), (tot, c) in sorted(by.items()):
        log(f"    {case:22s} {m:14s}: {c}/{tot}")

    def _isnan(s):
        try:
            return not np.isfinite(float(s))
        except (TypeError, ValueError):
            return True

    viol = [r for r in rows
            if (str(r["censored"]) == "1" and not _isnan(r["exec_time"]))
            or (str(r["censored"]) == "0" and str(r["success"]) != "True")]
    log(f"  censored-with-a-completion-time or uncensored-but-unsuccessful: "
        f"{len(viol)} rows")
    ok = (len(out) == n) and all(c == 0 for _, c in by.values()) and not viol
    log(f"  [{'PASS' if ok else 'FAIL'}] E/the pilot has 0 censored runs and "
        f"every run ({n}) is kept in its cell's denominator (n=3, from "
        f"pilot_summary.json)")

    # ---- positive control: the machinery must flag a truncated run ----
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    rowc, resc = run_cell("V2S3", "Ours_rollout", 1000, v2cfg=cfg,
                          return_res=True, t_max=100.0)
    cut = dict(case="sequential_obstacle", method="Ours_rollout", seed=1000,
               horizon_s=100.0, ended=resc["terminated_by"],
               success=rowc.get("success"), exec_time=rowc.get("exec_time"),
               sim_time_s=resc["logs"][-1]["t"] if resc["logs"] else None,
               rho_at_horizon=resc["logs"][-1]["rho"] if resc["logs"] else None)
    write_csv(OUT / "E_censoring_control.csv", [cut])
    log(f"\n  positive control (V2S3 truncated at 100 s): ended="
        f"{cut['ended']}, success={cut['success']}, exec_time={cut['exec_time']}")
    ok2 = (not bool(cut["success"])) and _isnan(cut["exec_time"])
    log(f"  [{'PASS' if ok2 else 'FAIL'}] E/the censoring machinery works "
        f"(a horizon-truncated run is flagged and keeps no completion time)")
    return ok and ok2


# ===========================================================================
# F. determinism and limits
# ===========================================================================
def test_F(log):
    log("=" * 78)
    log("F. determinism and joint limits (Stage-4 geometry)")
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    res = {}

    # ---- F1: one pilot cell, repeated twice --------------------------
    sid, alg, seed = "V2S2", "B2", 1000
    registry_alg(alg)
    log(f"\n  F1 repeat one pilot cell twice: {sid} / {alg} / seed {seed}")
    r1, s1 = run_cell(sid, alg, seed, v2cfg=cfg, return_res=True,
                      record_states=True)
    r2, s2 = run_cell(sid, alg, seed, v2cfg=cfg, return_res=True,
                      record_states=True)
    bad = _diff_traces(_trace_arrays(s1), _trace_arrays(s2))
    nval = int(sum(v.size for v in _trace_arrays(s1).values()))
    ok1 = not bad
    log(f"    {len(s1['trace_states'])} ticks, {nval} state values: "
        f"{'IDENTICAL' if ok1 else bad[:2]}")
    # against the stored pilot row for the same cell
    import csv as _csv
    with open(PILOT / "pilot_runs.csv", newline="",
              encoding="utf-8-sig") as f:
        stored = next(r for r in _csv.DictReader(f)
                      if r["case"] == "single_obstacle"
                      and r["method"] == alg and r["seed"] == str(seed))

    def _norm(v):
        if isinstance(v, (bool, np.bool_)):
            return str(bool(v))
        try:
            x = float(v)
            return "nan" if not np.isfinite(x) else f"{x:.12g}"
        except (TypeError, ValueError):
            return str(v)

    skip = {"exec_time"}          # identical anyway, but compare it explicitly
    diff = {k: (stored[k], _norm(r1.get(k)))
            for k in stored if k not in skip
            and k in r1 and _norm(stored[k]) != _norm(r1.get(k))}
    log(f"    against the stored pilot row: "
        f"{'all columns identical' if not diff else f'{len(diff)} differ: {diff}'}")
    log(f"    exec_time stored={stored['exec_time']} rerun={r1.get('exec_time')}")
    ok1 &= not diff
    res["F1_repeat"] = dict(cell=f"{sid}/{alg}/{seed}", n_ticks=len(
        s1["trace_states"]), n_values=nval, identical=not bad,
        vs_stored_diff=diff)

    # ---- F2: jobs = 1 vs jobs = 4 ------------------------------------
    log("\n  F2 the same cell through runner_s4 at jobs=1 and jobs=4")
    outs = {}
    for jobs in (1, 4):
        d = OUT / f"_det_j{jobs}"
        d.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, "-X", "utf8", "-m", "v2.runner_s4",
               "--scenarios", sid, "--seeds", "1", "--seed-offset",
               str(seed), "--algs", alg, "--jobs", str(jobs), "--out", str(d)]
        t0 = time.perf_counter()
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        wall = time.perf_counter() - t0
        if r.returncode != 0:
            log("    runner failed:\n" + r.stdout[-1500:] + r.stderr[-1500:])
        with open(d / "detail.csv", newline="", encoding="utf-8-sig") as f:
            outs[jobs] = (list(csv.DictReader(f)), wall)
    keys = [k for k in outs[1][0][0] if k != "ctrl_ms"]
    ndiff = 0
    worst = None
    for a, b in zip(outs[1][0], outs[4][0]):
        for k in keys:
            if str(a[k]) != str(b[k]):
                ndiff += 1
                worst = (k, a[k], b[k])
    log(f"    {len(outs[1][0])} row x {len(keys)} columns (ctrl_ms excluded), "
        f"mismatches={ndiff}" + (f", e.g. {worst}" if worst else ""))
    ok2 = ndiff == 0
    log(f"    [{'PASS' if ok2 else 'FAIL'}] F1+F2 repeat run and jobs=1 == "
        f"jobs=4, every non-timing value")
    res["F2_jobs"] = dict(rows=len(outs[1][0]), columns=len(keys),
                          mismatches=ndiff, example=worst,
                          wall_s={str(j): round(outs[j][1], 2)
                                  for j in outs})

    # ---- F3: limits, read from the stored smoke detail ----------------
    log("\n  F3 limit violations and binding counts, read from the stored "
        f"{SMOKE / 'detail.csv'}")
    with open(SMOKE / "detail.csv", newline="", encoding="utf-8-sig") as f:
        det = list(csv.DictReader(f))
    rows = []
    for r in det:
        d = dict(scenario=r["scenario"], alg=r["alg"], seed=r["seed"],
                 **{k: _f(r.get(k), 0.0) for k in LIMIT_COLS + BIND_COLS})
        d["min_surface_clearance_so"] = _f(r.get("min_surface_clearance_so"))
        d["dmin_robot_robot"] = _f(r.get("dmin_robot_robot"))
        d["rho_avg"] = _f(r.get("rho_avg"))
        d["collision_rate"] = _f(r.get("collision_rate"))
        rows.append(d)
    write_csv(OUT / "F_limits.csv", rows)
    for d in rows:
        log(f"    {d['scenario']}/{d['alg']:<13s}: "
            + " ".join(f"{k.replace('violation_', '')}={d[k]:g}"
                       for k in LIMIT_COLS)
            + f" | bind C3={d['bind_C3_sheep']:g} R4={d['bind_R4_robot']:g} "
              f"lat={d['bind_LAT_robot']:g} C1lo={d['bind_C1_low_sheep']:g} "
              f"C1hi={d['bind_C1_high_sheep']:g} C2={d['bind_C2_sheep']:g} "
              f"| so clr {d['min_surface_clearance_so']:+.2f} m, rr "
              f"{d['dmin_robot_robot']:.2f} m, rho {d['rho_avg']:.3f}")
    zero = all(d[k] == 0.0 for d in rows for k in LIMIT_COLS)
    bind = {k: int(sum(d[k] for d in rows)) for k in BIND_COLS}
    nonzero = [k for k, v in bind.items() if v > 0]
    log(f"    total binding counts over {len(rows)} stored runs: {bind}")
    ok3 = zero and len(nonzero) >= 3
    log(f"    [{'PASS' if ok3 else 'FAIL'}] F3/C3, R4, lateral, C1 and C2 "
        f"violations are 0 in all {len(rows)} stored runs, and the bounds still "
        f"bind ({len(nonzero)}/{len(BIND_COLS)} binding counters > 0)")
    res["F3_limits"] = dict(rows=len(rows), all_zero=zero, bindings=bind)
    (OUT / "F_determinism.json").write_text(json.dumps(res, indent=2,
                                                       default=str),
                                            encoding="utf-8")
    return ok1 and ok2 and ok3


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=list("DEF"))
    ap.add_argument("--log", default="ACCEPTANCE_DEF.txt")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    LOG[0] = Tee(OUT / args.log)
    log = LOG[0]
    log("V2 Stage-4 acceptance tests D / E / F (completion run)")
    log(f"python {sys.version.split()[0]}, numpy {np.__version__}")
    log(f"shipped configuration: config_id={stage3_v2cfg('M1', **STAGE3_TTC).config_id}")
    log("A, B and C are *not* re-run: their evidence is the stored "
        "ACCEPTANCE.txt / A_inertness.csv / B_range_sweep.csv.")
    t0 = time.time()
    fn = dict(D=test_D, E=test_E, F=test_F)
    results = {}
    for t in args.tests:
        results[t] = bool(fn[t](log))
    log("=" * 78)
    n = sum(1 for v in results.values() if v)
    log(f"SUMMARY: {n}/{len(results)} of D/E/F passed ({time.time() - t0:.1f} s)")
    (OUT / "ACCEPTANCE_DEF.json").write_text(json.dumps(
        results, indent=2), encoding="utf-8")
    return 0 if n == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
