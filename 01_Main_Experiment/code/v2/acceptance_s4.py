"""Stage-4 acceptance tests A-F.

    python -X utf8 -m v2.acceptance_s4              # all tests
    python -X utf8 -m v2.acceptance_s4 --tests A B  # subset

A  Inertness -- the *old V1 scene definitions*, pushed through the **Stage-4
   runner** with V1-compatible settings, reproduce the Stage-3 trajectory
   **bit-exactly** on spot cells (both against the archived Stage-3 reference,
   read-only, and against a fresh pre-edit capture).  The V1 and Stage-3 source
   hashes are unchanged.
B  The new geometry exercises sensing -- on the 140 m / 220 m scenes the
   40/70/100 m range sweep produces *different* first-detection times **and**
   *different* trajectories, in contrast with the Stage-3 finding that on the V1
   geometry the 40 m and 70 m trajectories were bit-identical.
C  False-trigger control -- on the no-obstacle 140 m scene (and the bent
   variant) PRE/DIVERT never fire.
D  Sequential-recovery case exists -- in the 220 m S3 a new threat is detected
   (and its RISK ONSET fires) while RECOVER is still active; the state trace is
   reported, together with the held-out-seed hit rate.
E  Censoring -- every pilot run that hits the horizon without reaching the goal
   is counted, kept in the denominator and reported as censored.
F  Determinism and limits -- repeat runs 0 diff, jobs=1 vs jobs=4, C3/R4/
   lateral/C1/C2 violations 0 and still binding.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import STAGE2_TTC, STAGE3_TTC, stage3_v2cfg  # noqa: E402
from v2.metrics_v2 import summarize_v2  # noqa: E402
from v2.runner_s4 import registry_alg, run_cell  # noqa: E402
from v2.scenarios_v2 import W_V2, resolve_scene  # noqa: E402
from v2.seq_trace_s4 import analyse, control_rows  # noqa: E402

_SHIPPED = ROOT.parent / "results" / "results_v2_stage4"
# the archive sits beside ``code/`` in this package, but under ``code/``
# in the original build tree; accept either.
OUT = _SHIPPED if _SHIPPED.is_dir() else ROOT / "results_v2_stage4"
REF = OUT / "reference"
GEO = OUT / "geometry"
PILOT = OUT / "pilot"

TRACE_COLS = ["t", "step", "alive", "spos", "shead", "sspeed",
              "rpos", "rhead", "rspeed", "romega", "opos"]

# V1-equivalent sensing + the Stage-2 tuple == the Stage-3 test-A arm
V1_SENSE = dict(sense_mode="range", sense_r_obs=1e9, sense_r_animal=10.0,
                sense_err=0.0, sense_delay=0.0, sense_loss=0.0, sense_ttl=0.0)
S3_SENSE = dict(sense_mode="range", sense_r_obs=70.0, sense_r_animal=25.0,
                sense_err=0.0, sense_delay=0.0, sense_loss=0.0,
                sense_ttl=0.30)


# ---------------------------------------------------------------------------
class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def __call__(self, *a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        self.f.write(s + "\n")
        self.f.flush()


RESULTS = []
LOG = [None]


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    line = f"  [{'PASS' if ok else 'FAIL'}] {name}" + (
        f"  -- {detail}" if detail else "")
    (LOG[0] or print)(line)
    return bool(ok)


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def trace_arrays(res):
    tr = res["trace_states"]
    out = {c: np.array([row[i] for row in tr]) for i, c in enumerate(
        TRACE_COLS)}
    return {c: np.ascontiguousarray(v) for c, v in out.items()}


def diff_traces(a, b):
    bad = []
    if set(a) != set(b):
        return [("keys", "mismatch")]
    for k in a:
        if a[k].shape != b[k].shape:
            bad.append((k, "shape", a[k].shape, b[k].shape))
        elif not np.array_equal(a[k], b[k]):
            bad.append((k, "value", float(np.max(np.abs(a[k] - b[k])))))
    return bad


def compare_traces(a, b):
    """Tick-level divergence summary between two trace dicts."""
    n = min(len(a["t"]), len(b["t"]))
    first_tick = None
    n_diff_values = 0
    worst = 0.0
    for k in a:
        if k not in b:
            continue
        x, y = a[k][:n], b[k][:n]
        if x.shape != y.shape:
            continue
        d = ~(x == y)
        # collapse per-tick differences across all remaining axes
        per_tick = d.reshape(n, -1).any(axis=1) if d.ndim > 1 else d
        idx = np.where(per_tick)[0]
        if len(idx):
            first_tick = int(idx[0]) if first_tick is None else min(
                first_tick, int(idx[0]))
        n_diff_values += int(d.sum())
        if d.any():
            with np.errstate(invalid="ignore"):
                worst = max(worst, float(np.nanmax(np.abs(
                    x.astype(float) - y.astype(float)))))
    return dict(n_ticks=n, shape_equal=(len(a["t"]) == len(b["t"])),
                n_diff_values=n_diff_values, first_diff_tick=first_tick,
                max_abs_diff=worst,
                identical=bool(n_diff_values == 0 and len(a["t"]) == len(b["t"])))


def first_detect(res, k=1):
    return (res.get("v2") or {}).get(f"sense_first_detect_{k}", np.nan)


# ===========================================================================
# A. inertness
# ===========================================================================
def test_A(log):
    log("=" * 78)
    log("A. inertness: the V1 scene definitions through the Stage-4 runner")
    ok = True
    rows = []

    # ---- A1: V1-equivalent arm == the archived Stage-3 reference ------
    cfg_v1 = stage3_v2cfg("M1", **V1_SENSE, **STAGE2_TTC)
    log(f"  A1 config (V1-equivalent sensing + Stage-2 tuple): "
        f"config_id={cfg_v1.config_id}")
    for name, sid, seed in (("S3_s1000", "S3", 1000),
                            ("S2_s1000", "S2", 1000),
                            ("S1_s1002", "S1", 1002)):
        _PREV = ROOT.parent / "results" / "results_v2_stage3"
        ref_root = _PREV if _PREV.is_dir() else ROOT / "results_v2_stage3"
        ref_path = ref_root / "reference" / f"{name}.npz"
        if not ref_path.exists():
            check(f"A1/{name}: Stage-3 archive reference present", False,
                  str(ref_path))
            continue
        ref = np.load(ref_path)
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg_v1,
                            return_res=True, record_states=True)
        cur = trace_arrays(res)
        bad = diff_traces(cur, {k: ref[k] for k in ref.files})
        n = int(sum(v.size for v in cur.values()))
        ok &= check(f"A1/{name}: V1 scene + Stage-4 runner == Stage-3 archive "
                    f"({n} values)", not bad, f"{bad[:1]}")
        rows.append(dict(arm="A1_v1eq_vs_stage3_archive", cell=name,
                         ticks=len(cur["t"]), values=n, identical=not bad,
                         detail=str(bad[:1])))

    # ---- A2: shipped Stage-3 configuration == fresh pre-edit capture --
    cfg_s3 = stage3_v2cfg("M1", **S3_SENSE, **STAGE3_TTC)
    log(f"  A2 config (shipped Stage-3, 70 m, frozen tuple): "
        f"config_id={cfg_s3.config_id}")
    for name, sid, seed in (("stage3_S3_s1000", "S3", 1000),
                            ("stage3_S2_s1000", "S2", 1000),
                            ("stage3_S1_s1002", "S1", 1002)):
        p = REF / f"{name}.npz"
        if not p.exists():
            check(f"A2/{name}: pre-edit capture present", False, str(p))
            continue
        ref = np.load(p)
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg_s3,
                            return_res=True, record_states=True)
        cur = trace_arrays(res)
        bad = diff_traces(cur, {k: ref[k] for k in ref.files})
        meta = json.loads((REF / f"{name}.json").read_text(encoding="utf-8"))
        import math

        def _norm(v):
            if v is None:
                return "nan"
            if isinstance(v, bool):
                return repr(v)
            if isinstance(v, (int, float, np.floating, np.integer)):
                f = float(v)
                return "nan" if not math.isfinite(f) else repr(f)
            return str(v)
        sdiff = [k for k in meta["summary"]
                 if k not in ("ctrl_ms", "config_id")
                 and _norm(row.get(k)) != _norm(meta["summary"][k])]
        n = int(sum(v.size for v in cur.values()))
        ok &= check(f"A2/{name}: V1 scene + Stage-4 runner == pre-edit Stage-3 "
                    f"capture ({n} values)", (not bad) and (not sdiff),
                    f"trace {bad[:1]} summary {sdiff[:2]}")
        rows.append(dict(arm="A2_shipped_vs_pre_edit_capture", cell=name,
                         ticks=len(cur["t"]), values=n,
                         identical=(not bad) and (not sdiff),
                         detail=str(bad[:1])))
    write_csv(OUT / "A_inertness.csv", rows)

    # ---- A3: no Stage-3 / V1 source file moved ------------------------
    import hashlib
    h = json.loads((OUT / "code_hashes_pre_stage4.json").read_text(
        encoding="utf-8"))
    moved = []
    for group, files in h.items():
        for p, digest in files.items():
            fp = ROOT / p
            if not fp.exists():
                moved.append((p, "missing"))
                continue
            cur = hashlib.sha256(fp.read_bytes()).hexdigest()[:16].upper()
            if cur != digest:
                moved.append((p, f"{digest}->{cur}"))
    ok &= check("A3/no V1 or Stage-3 source file changed since the pre-Stage-4 "
                "hash snapshot", not moved,
                f"{len(moved)} moved" + (f": {moved[:3]}" if moved else ""))
    return ok


# ===========================================================================
# B. the new geometry exercises the sensing range
# ===========================================================================
def _sweep_cell(sid, seed, R, v2cfg_base):
    cfg = replace(v2cfg_base, **{**S3_SENSE, "sense_r_obs": float(R)})
    return run_cell(sid, "Ours_rollout", seed, v2cfg=cfg, return_res=True,
                    record_states=True)


def test_B(log):
    log("=" * 78)
    log("B. the 140/220 m geometry exercises the sensing range "
        "(40/70/100 m sweep)")
    ok = True
    rows = []
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)

    cells = [("V2S2", 1000, (40.0, 70.0, 100.0)),
             ("S3", 1000, (40.0, 70.0)),          # V1 geometry, for contrast
             ("V2S3", 1000, (40.0, 70.0, 100.0))]
    for sid, seed, Rs in cells:
        got = {}
        for R in Rs:
            row, res = _sweep_cell(sid, seed, R, cfg)
            got[R] = (row, res, trace_arrays(res))
            rows.append(dict(scenario=sid, geometry="v2" if sid.startswith("V2")
                             else "v1", seed=seed, R=R,
                             first_detect_1=first_detect(res, 1),
                             first_detect_2=first_detect(res, 2),
                             first_detect_3=first_detect(res, 3),
                             t_pre_on=row.get("t_pre_on"),
                             t_divert_on=row.get("t_divert_on"),
                             contact_fraction=row.get("collision_rate"),
                             min_dist=row.get("min_dist"),
                             rho=row.get("rho_avg"),
                             ticks=len(res["trace_states"])))
            log(f"  {sid} R={R:5.0f}: first detect "
                f"{first_detect(res, 1)}, PRE {row.get('t_pre_on')}, "
                f"DIVERT {row.get('t_divert_on')}, "
                f"C={row.get('collision_rate'):.3f}, "
                f"min_dist={row.get('min_dist')}, "
                f"{len(res['trace_states'])} ticks")
        # pairwise trajectory comparison
        pairs = []
        for a, b in ((40.0, 70.0), (70.0, 100.0), (40.0, 100.0)):
            if a in got and b in got:
                cmp = compare_traces(got[a][2], got[b][2])
                cmp.update(scenario=sid, R_a=a, R_b=b)
                pairs.append(cmp)
                rows[-1][f"cmp_{int(a)}_{int(b)}"] = (
                    "identical" if cmp["identical"]
                    else f"{cmp['n_diff_values']} values differ from tick "
                         f"{cmp['first_diff_tick']}")
                log(f"    {sid} {a:g} vs {b:g} m: "
                    + ("trajectories BIT-IDENTICAL"
                       if cmp["identical"] else
                       f"{cmp['n_diff_values']} differing state values, first "
                       f"difference at tick {cmp['first_diff_tick']} "
                       f"(t={cmp['first_diff_tick'] * 0.05:.2f} s), "
                       f"max |delta| {cmp['max_abs_diff']:.4g}"))
        got["_pairs"] = pairs
        if sid == "V2S2":
            v2pairs = pairs
        if sid == "S3":
            v1pairs = pairs
        if sid == "V2S3":
            v2s3pairs = pairs
    write_csv(OUT / "B_range_sweep.csv", rows)

    # first-detection deltas
    def fd(sid, seed, R):
        for r in rows:
            if r["scenario"] == sid and r["seed"] == seed and r["R"] == R:
                return r["first_detect_1"]
        return np.nan

    d_v2 = fd("V2S2", 1000, 40.0) - fd("V2S2", 1000, 70.0)
    d_v1 = fd("S3", 1000, 40.0) - fd("S3", 1000, 70.0)
    log(f"\n  first-detection delta (40 m - 70 m): V2S2 {d_v2:+.2f} s, "
        f"V1 S3 {d_v1:+.2f} s")
    ok &= check("B/on the new 140 m scene the 40 m run detects strictly later "
                "than the 70 m run", np.isfinite(d_v2) and d_v2 > 1.0,
                f"V2S2 70 m {fd('V2S2', 1000, 70.0):.2f} s -> 40 m "
                f"{fd('V2S2', 1000, 40.0):.2f} s ({d_v2:+.2f} s)")
    ok &= check("B/and the trajectories differ (Stage-3 on the V1 geometry: "
                "bit-identical)", all(not p["identical"] for p in v2pairs),
                "; ".join(f"{p['R_a']:g}/{p['R_b']:g}: "
                          f"{p['n_diff_values']} values" for p in v2pairs))
    ok &= check("B/the contrast holds for the 220 m sequential scene too",
                all(not p["identical"] for p in v2s3pairs),
                "; ".join(f"{p['R_a']:g}/{p['R_b']:g}: "
                          f"{p['n_diff_values']} values" for p in v2s3pairs))
    ok &= check("B/the Stage-3 finding is reproduced on the V1 geometry "
                "(40 m == 70 m, bit-identical), so the contrast is the "
                "geometry, not the code",
                all(p["identical"] for p in v1pairs),
                "; ".join(f"{p['R_a']:g}/{p['R_b']:g}: "
                          f"{p['n_diff_values']} values" for p in v1pairs))
    return ok


# ===========================================================================
# C. false-trigger control
# ===========================================================================
def test_C(log):
    log("=" * 78)
    log("C. false-trigger control on the no-obstacle scenes")
    ok = True
    rows = []
    for sid in ("V2S1", "V2S1N"):
        for alg in ("B1", "B2", "Ours_event", "Ours_rollout"):
            registry_alg(alg)
            row, res = run_cell(sid, alg, 1000, v2cfg=stage3_v2cfg("M1"),
                                return_res=True)
            fired = [tag for tag in ("pre", "divert", "recover")
                     if any(l.get("tag") == tag for l in res["logs"])]
            rows.append(dict(scenario=sid, method=alg,
                             t_pre_on=row.get("t_pre_on"),
                             t_divert_on=row.get("t_divert_on"),
                             states_fired=",".join(fired) or "none",
                             switch_count=row.get("switch_count"),
                             contact_fraction=row.get("collision_rate"),
                             success=row.get("success"),
                             ended=res["terminated_by"],
                             rho=row.get("rho_avg")))
            log(f"  {sid:6s} {alg:14s}: PRE={row.get('t_pre_on')}, "
                f"DIVERT={row.get('t_divert_on')}, "
                f"states fired: {','.join(fired) or 'none'}, "
                f"switches={row.get('switch_count')}, "
                f"C={row.get('collision_rate'):.3f}, {res['terminated_by']}")
    write_csv(OUT / "C_false_trigger.csv", rows)
    bad = [r for r in rows if r["states_fired"] != "none"]
    ok &= check("C/PRE/DIVERT/RECOVER never fire on the 140 m no-obstacle "
                "scenes (8 runs: 2 scenes x 4 methods)", not bad,
                "; ".join(f"{r['scenario']}/{r['method']}: {r['states_fired']}"
                          for r in bad))
    return ok


# ===========================================================================
# D. the sequential-recovery case exists
# ===========================================================================
def test_D(log, jobs=1):
    log("=" * 78)
    log("D. a new threat arrives while RECOVER is still active (220 m S3)")
    ok = True
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    seeds = list(range(1000, 1020))
    rows = []
    hits_onset, hits_detect, traces = 0, 0, {}
    if jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            out = list(ex.map(_d_cell, seeds))
    else:
        out = [_d_cell(s) for s in seeds]
    for seed, row, info in out:
        o_on = bool(info["overlap"])
        o_det = bool(info["overlap_detect"])
        hits_onset += 1 if o_on else 0
        hits_detect += 1 if o_det else 0
        rows.append(dict(
            seed=seed, n_recover=len(info["recover_windows"]),
            recover_windows=";".join(f"{a:.2f}-{b:.2f}"
                                     for a, b in info["recover_windows"]),
            detect_1=info["first_detect"][1], detect_2=info["first_detect"][2],
            detect_3=info["first_detect"][3],
            onset_1=info["risk_onset"][1], onset_2=info["risk_onset"][2],
            onset_3=info["risk_onset"][3],
            order_ok=info["order_ok"],
            detect_order_ok=info["detect_order_ok"],
            overlap_onset=o_on, overlap_detect=o_det,
            overlap_onset_desc=";".join(f"o{x['threat']}@{x['t_onset']:.2f}"
                                        for x in info["overlap"]),
            overlap_detect_desc=";".join(f"o{x['threat']}@{x['t_detect']:.2f}"
                                         for x in info["overlap_detect"]),
            collision_rate=row.get("collision_rate"),
            min_dist=row.get("min_dist"), success=row.get("success"),
            ended=row.get("terminated_by")))
        if o_on and "onset" not in traces:
            traces["onset"] = seed
        if o_det and "detect" not in traces:
            traces["detect"] = seed
        log(f"  seed {seed}: recover={rows[-1]['recover_windows']:32s} "
            f"det2={info['first_detect'][2]:7.2f} "
            f"onset2={info['risk_onset'][2]:7.2f} "
            f"overlap(onset)={o_on} overlap(detect)={o_det} "
            f"order={info['order_ok']}")
    write_csv(OUT / "D_sequential_recovery.csv", rows)

    log(f"\n  held-out check on all 20 calibration seeds: risk-onset overlap "
        f"{hits_onset}/20, detection overlap {hits_detect}/20")
    ok &= check("D/a new threat's RISK ONSET (TTC <= T_PRE) fires inside a "
                "RECOVER window in at least one seed",
                hits_onset >= 1, f"{hits_onset}/20 seeds")
    ok &= check("D/the release order (1 < 2 < 3) is reproduced from the "
                "trajectories, not copied from the design",
                all(r["order_ok"] for r in rows),
                f"{sum(1 for r in rows if r['order_ok'])}/20 seeds ordered")

    # ---- the state trace of one overlapping seed ----------------------
    if traces:
        key = "onset" if "onset" in traces else "detect"
        seed = traces[key]
        row, res = run_cell("V2S3", "Ours_rollout", seed, v2cfg=cfg,
                            return_res=True)
        cr = control_rows(res, cfg)
        rows_out = []
        prev = None
        for r in cr:
            mark = []
            for k in (1, 2, 3):
                fd = r[f"first_detect_o{k}"]
                if np.isfinite(fd) and abs(r["t"] - fd) < 1e-6:
                    mark.append(f"detect_o{k}")
                if np.isfinite(r[f"ttc_o{k}"]) and \
                        r[f"ttc_o{k}"] <= cfg.ttc_pre and \
                        (prev is None or not np.isfinite(prev[f"ttc_o{k}"])
                         or prev[f"ttc_o{k}"] > cfg.ttc_pre):
                    mark.append(f"onset_o{k}")
            changed = (r["tag"] != prev["tag"]) if prev is not None else True
            if changed or mark:
                rows_out.append(dict(t=r["t"], tag=r["tag"], rho=r["rho"],
                                     lat_c=r["lat_c"], ttc_o1=r["ttc_o1"],
                                     ttc_o2=r["ttc_o2"], ttc_o3=r["ttc_o3"],
                                     visible_o1=r["visible_o1"],
                                     visible_o2=r["visible_o2"],
                                     visible_o3=r["visible_o3"],
                                     marks=",".join(mark)))
            prev = r
        write_csv(GEO / f"D_state_trace_seed{seed}.csv", rows_out)
        log(f"\n  state trace (seed {seed}, {key} reading) -- full tick table "
            f"in {GEO / f'D_state_trace_seed{seed}.csv'}:")
        for r in rows_out:
            log(f"    t={r['t']:7.2f}  {r['tag']:<8s} rho={r['rho']:.3f} "
                f"lat={r['lat_c']:+.2f} vis=({r['visible_o1']:.0f},"
                f"{r['visible_o2']:.0f},{r['visible_o3']:.0f}) "
                f"ttc=({r['ttc_o1']:6.2f},{r['ttc_o2']:6.2f},"
                f"{r['ttc_o3']:6.2f}) {r['marks']}")
    return ok


# ===========================================================================
# E. censoring
# ===========================================================================
def test_E(log):
    log("=" * 78)
    log("E. censoring: runs that hit the horizon without reaching the goal")
    p = PILOT / "pilot_runs.csv"
    if not p.exists():
        check("E/pilot results present (run `python -m v2.pilot_s4` first)",
              False, str(p))
        return False
    with open(p, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    cens = [r for r in rows if str(r["censored"]) == "1"]
    log(f"  {n} pilot runs; {len(cens)} censored "
        f"({100.0 * len(cens) / max(n, 1):.1f} %) -- all kept in every "
        f"denominator")
    by = {}
    for r in rows:
        k = (r["case"], r["method"])
        by.setdefault(k, [0, 0])
        by[k][0] += 1
        by[k][1] += 1 if str(r["censored"]) == "1" else 0
    for (case, m), (tot, c) in sorted(by.items()):
        if c:
            log(f"    {case:22s} {m:14s}: {c}/{tot} censored")
    rows_out = [dict(case=r["case"], method=r["method"], seed=r["seed"],
                     censored=int(r["censored"]), ended=r["ended"],
                     success=r["success"], completion=r["completion"],
                     exec_time=r["exec_time"],
                     sim_time_s=r["sim_time_s"],
                     coverage_at_end=r["coverage_at_end"],
                     rho_mean=r["rho_mean"])
                for r in rows]
    write_csv(OUT / "E_censoring.csv", rows_out)

    def _isnan(s):
        try:
            return not np.isfinite(float(s))
        except (TypeError, ValueError):
            return True

    # a run that never reached the goal cannot have a completion time, and a
    # finite completion time cannot belong to a censored run.  (The converse is
    # not an invariant: a run can reach the goal and still miss the joint
    # completion criterion -- that category is counted separately.)
    viol = [r for r in rows
            if (str(r["censored"]) == "1" and not _isnan(r["exec_time"]))
            or (not _isnan(r["exec_time"]) and str(r["censored"]) == "0"
                and str(r["success"]) != "True")]
    goal_but_no_joint = [r for r in rows
                         if str(r["censored"]) == "0"
                         and _isnan(r["exec_time"])]
    log(f"  goal reached but joint-completion criterion never met: "
        f"{len(goal_but_no_joint)} runs (kept, reported as right-censored "
        f"completion time)")
    ok = check("E/censored runs are reported and not dropped",
               len(rows_out) == n, f"{n} runs, {len(cens)} censored")
    ok &= check("E/a censored run never carries a completion time (and no "
                "completion time belongs to a censored run)", not viol,
                f"{len(viol)} inconsistent rows")

    # ---- positive control: truncate the horizon and check censorship --
    #   The pilot itself has no censored runs (every method reaches the goal on
    #   140/220 m), so the machinery is demonstrated on a deliberately
    #   truncated horizon: V2S3 cut at 100 s (the third threat has not even been
    #   reached).  The run must be reported censored, kept in the denominator
    #   and must not acquire a completion time.
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    rowc, resc = run_cell("V2S3", "Ours_rollout", 1000, v2cfg=cfg,
                          return_res=True, t_max=100.0)
    cut = dict(case="sequential_obstacle", method="Ours_rollout", seed=1000,
               horizon_s=100.0, ended=resc["terminated_by"],
               success=rowc.get("success"),
               exec_time=rowc.get("exec_time"),
               rho_at_horizon=resc["logs"][-1]["rho"] if resc["logs"] else None,
               sim_time_s=resc["logs"][-1]["t"] if resc["logs"] else None)
    write_csv(OUT / "E_censoring_control.csv", [cut])
    log(f"  positive control (V2S3 truncated at 100 s): ended="
        f"{cut['ended']}, success={cut['success']}, "
        f"exec_time={cut['exec_time']}, "
        f"rho at horizon={cut['rho_at_horizon']}")
    ok &= check("E/the censoring machinery works (a horizon-truncated run is "
                "flagged censored and keeps no completion time)",
                (not bool(cut["success"]))
                and str(cut["ended"]) != "goal"
                and _isnan(cut["exec_time"]),
                f"ended={cut['ended']} success={cut['success']}")
    return ok


# ===========================================================================
# F. determinism and limits
# ===========================================================================
def test_F(log):
    log("=" * 78)
    log("F. determinism and joint limits on the Stage-4 geometry")
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    ok = True
    for sid, seed in (("V2S3", 1000), ("V2S2", 1001)):
        _, r1 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        _, r2 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        bad = diff_traces(trace_arrays(r1), trace_arrays(r2))
        ok &= check(f"F/repeat run identical ({sid}, seed {seed}, "
                    f"{len(r1['trace_states'])} ticks)", not bad, f"{bad[:2]}")

    # ---- jobs = 1 vs 4 ------------------------------------------------
    import subprocess
    outs = {}
    for jobs in (1, 4):
        d = OUT / f"_det_j{jobs}"
        d.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, "-X", "utf8", "-m", "v2.runner_s4",
               "--scenarios", "V2S2", "--seeds", "2", "--seed-offset", "1000",
               "--algs", "Ours_rollout", "--jobs", str(jobs), "--out", str(d)]
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        if r.returncode != 0:
            log("  runner failed:\n" + r.stdout[-1500:] + r.stderr[-1500:])
        with open(d / "detail.csv", newline="", encoding="utf-8-sig") as f:
            outs[jobs] = list(csv.DictReader(f))
    keys = [k for k in outs[1][0] if k not in ("ctrl_ms",)]
    ndiff = 0
    worst = None
    for a, b in zip(outs[1], outs[4]):
        for k in keys:
            if str(a[k]) != str(b[k]):
                ndiff += 1
                worst = (k, a[k], b[k])
    log(f"  jobs=1 vs jobs=4: {len(outs[1])} rows x {len(keys)} columns "
        f"(ctrl_ms excluded), mismatches={ndiff}")
    ok &= check("F/jobs=1 == jobs=4, every non-timing column", ndiff == 0,
                f"{ndiff} mismatches" + (f", e.g. {worst}" if worst else ""))

    # ---- limit violations ---------------------------------------------
    rows = []
    for sid, seed in (("V2S2", 1000), ("V2S3", 1000), ("V2S3", 1001)):
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                            return_res=True)
        m = summarize_v2(res, cfg)
        r = dict(scenario=sid, seed=seed,
                 **{k: m.get(k) for k in (
                     "violation_C3", "violation_R4", "violation_lat",
                     "violation_C1_low", "violation_C1_high", "violation_C2",
                     "violation_R1", "violation_R3", "LAT_UNRESOLVED",
                     "bind_C3_sheep", "bind_R4_robot", "bind_LAT_robot",
                     "n_contact_events_so", "min_surface_clearance_so",
                     "dmin_robot_robot", "success", "rho_avg", "max_dev")})
        rows.append(r)
        log(f"  {sid}/{seed}: C3={r['violation_C3']} R4={r['violation_R4']} "
            f"lat={r['violation_lat']} C1lo={r['violation_C1_low']} "
            f"C1hi={r['violation_C1_high']} | bind C3={r['bind_C3_sheep']} "
            f"R4={r['bind_R4_robot']} lat={r['bind_LAT_robot']} | "
            f"so clr {r['min_surface_clearance_so']:.2f} m, rr "
            f"{r['dmin_robot_robot']:.2f} m | rho {r['rho_avg']:.3f}")
    write_csv(OUT / "F_limits.csv", rows)
    zero = all(float(r[k] or 0) == 0.0 for r in rows for k in
               ("violation_C3", "violation_R4", "violation_lat",
                "violation_C1_low", "violation_C1_high", "violation_C2",
                "violation_R1", "violation_R3", "LAT_UNRESOLVED"))
    ok &= check("F/C3, R4, lateral, C1 and C2 violations are 0", zero)
    ok &= check("F/the bounds still bind (non-vacuous)",
                all(float(r["bind_C3_sheep"] or 0) > 0 for r in rows)
                and all(float(r["bind_R4_robot"] or 0) > 0 for r in rows)
                and all(float(r["bind_LAT_robot"] or 0) > 0 for r in rows))
    return ok


# ---------------------------------------------------------------------------
def _d_cell(seed):
    """One S3 seed for test D (module level so a process pool can pickle it)."""
    cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    row, res = run_cell("V2S3", "Ours_rollout", seed, v2cfg=cfg,
                        return_res=True)
    info = analyse(control_rows(res, cfg), cfg)
    row = dict(row)
    row["terminated_by"] = res["terminated_by"]
    return seed, row, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=list("ABCDEF"))
    ap.add_argument("--jobs", type=int, default=1)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    GEO.mkdir(parents=True, exist_ok=True)
    log = Tee(OUT / "ACCEPTANCE.txt")
    LOG[0] = log
    log("V2 Stage-4 acceptance tests")
    log(f"python {sys.version.split()[0]}, numpy {np.__version__}")
    log(f"shipped Stage-3 configuration (reused unchanged): "
        f"config_id={stage3_v2cfg('M1', **STAGE3_TTC).config_id}")
    log(f"geometry: W={W_V2:g} m half-width (16 m total), 140 m / 220 m, "
        f"R_obs=70 m")
    t0 = time.time()
    fn = dict(A=test_A, B=test_B, C=test_C, D=test_D, E=test_E, F=test_F)
    for t in args.tests:
        if t == "D":
            fn[t](log, jobs=args.jobs)
        else:
            fn[t](log)
    log("=" * 78)
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    log(f"SUMMARY: {npass}/{len(RESULTS)} checks passed "
        f"({time.time() - t0:.1f} s)")
    for name, ok, detail in RESULTS:
        if not ok:
            log(f"  FAILED: {name}  {detail}")
    (OUT / "ACCEPTANCE.json").write_text(json.dumps(
        [dict(name=n, ok=ok, detail=d) for n, ok, d in RESULTS], indent=2),
        encoding="utf-8")
    return 0 if npass == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
