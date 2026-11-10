"""Stage-2 acceptance tests A-F.

    python -X utf8 -m v2.acceptance_s2                # all tests
    python -X utf8 -m v2.acceptance_s2 --tests A D    # subset

A  M0 inertness  -- M0 reproduces the Stage-1 V2 trajectory (and the V1 archive)
   **bit-exactly**, whole trajectory, not just a summary row.
B  M1 actually avoids -- controlled corridor, no robots, M0 vs M1 at 3/6/9 m:
   contact rate, mean minimum surface clearance, monotone in range, and no
   oscillation/explosion (speed + heading-rate distributions).
C  Delay and heterogeneity are real -- avoidance-onset latency vs reaction
   delay; per-animal gain distribution; removing heterogeneity changes the
   outcome distribution.
D  Predictor/environment decoupling -- matched vs mismatched predictor with an
   identical environment trajectory and different predictions; the NumPy engine
   agrees with the V1 DLL on M0.
E  Determinism -- repeat run 0 diff; jobs=1 vs jobs=4.
F  Joint limits in M1/M2 -- C3/R4/lateral violations stay 0 and still bind.

Every test prints a transcript and a PASS/FAIL line; the session is also written
to ``results_v2_stage2/ACCEPTANCE.txt`` and each table to a CSV beside it.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from config import default_cfg  # noqa: E402
from v2.acceptance_v2 import archived_row, parse_cell  # noqa: E402
from v2.config_v2 import (V2Cfg, compat_v1cfg, default_v2cfg,  # noqa: E402
                          sim_overrides, stage2_v2cfg)
from v2.corridor_v2 import corridor_scene, run_corridor  # noqa: E402
from v2.metrics_v2 import summarize_v2  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402
from v2.sim_v2 import run_scene_v2  # noqa: E402

_SHIPPED = ROOT.parent / "results" / "results_v2_stage2"
# the archive sits beside ``code/`` in this package, but under ``code/``
# in the original build tree; accept either.
OUT = _SHIPPED if _SHIPPED.is_dir() else ROOT / "results_v2_stage2"
REF = OUT / "reference"
BEH = OUT / "behaviour"
CAL_SEEDS = list(range(1000, 1020))

# CALIBRATED VALUES (filled from results_v2_stage2/calibration_summary.json;
# the fallbacks are the shipped Stage-2 defaults and are overwritten by load())
SHIPPED = {
    "m1_sense_range": 6.0,
    "m1_gain": 3.0,
    "m1_decay": "linear",
    "m1_decay_len": 2.0,
    "m2_react_delay": 0.15,
    "m2_gain_sigma": 0.20,
    "m_v_max": 2.0,
    "m2_slow_gain": 1.0,
}


def load_shipped():
    p = OUT / "calibration_summary.json"
    if not p.exists():
        return
    s = json.loads(p.read_text(encoding="utf-8"))
    SHIPPED["m1_gain"] = float(s["M1"]["gain"])
    SHIPPED["m1_decay"] = "exp" if str(s["M1"]["decay"]).startswith("exp") \
        else str(s["M1"]["decay"])
    SHIPPED["m1_decay_len"] = (1.0 if s["M1"]["decay"] == "exp1"
                               else 3.0) if SHIPPED["m1_decay"] == "exp" else 2.0
    m2 = s["M2"]
    for k in ("m2_react_delay", "m2_gain_sigma", "m_v_max", "m2_slow_gain"):
        SHIPPED[k] = float(m2[k])


def shipped_cfg(model="M2", **over):
    base = dict(SHIPPED)
    base.update(over)
    return stage2_v2cfg(model, **base)


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
    line = (f"  [{'PASS' if ok else 'FAIL'}] {name}"
            + (f"  -- {detail}" if detail else ""))
    if LOG[0] is not None:          # into ACCEPTANCE.txt (the Tee also prints)
        LOG[0](line)
    else:
        print(line, flush=True)
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


TRACE_COLS = ["t", "step", "alive", "spos", "shead", "sspeed",
              "rpos", "rhead", "rspeed", "romega", "opos"]


def trace_arrays(res):
    tr = res["trace_states"]
    out = {c: np.array([row[i] for row in tr]) for i, c in enumerate(TRACE_COLS)}
    for c in TRACE_COLS:
        out[c] = np.ascontiguousarray(out[c])
    return out


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


# ===========================================================================
# A. M0 inertness
# ===========================================================================
def test_A(log):
    log("=" * 78)
    log("A. M0 inertness: whole-trajectory bit-exactness against the Stage-1 code")
    cells = [("S3_s200_v2", "S3", 200, "v2", "joint"),
             ("S3_s200_v1compat", "S3", 200, "compat", "joint"),
             ("S1_s201_v2", "S1", 201, "v2", "joint"),
             ("S3_s200_v2_seq", "S3", 200, "v2", "sequential")]
    allok = True
    for name, sid, seed, mode, proj in cells:
        ref = np.load(REF / f"{name}.npz")
        v2cfg = compat_v1cfg() if mode == "compat" else default_v2cfg()
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=v2cfg,
                            compat=(mode == "compat"), projection=proj,
                            return_res=True, record_states=True)
        cur = trace_arrays(res)
        bad = diff_traces(cur, {k: ref[k] for k in ref.files})
        n_floats = int(sum(v.size for v in cur.values()))
        ok = not bad
        allok &= ok
        check(f"A/{name} M0 == Stage-1 (whole trajectory, {n_floats} values)",
              ok, "" if ok else f"first diffs: {bad[:3]}")
        if mode == "compat":
            # ... and the archived V1 summary row, column by column
            arch = archived_row(sid, "Ours_rollout", seed)
            bad2 = []
            n_cmp = 0
            for k, v in row.items():
                if k not in arch or k in ("ctrl_ms", "seed", "config_id"):
                    continue
                n_cmp += 1
                a = parse_cell(arch[k])
                if isinstance(v, bool) or isinstance(a, bool):
                    if bool(v) != bool(a):
                        bad2.append((k, v, a))
                elif isinstance(a, float) and isinstance(v, (int, float)):
                    if not ((math.isnan(a) and math.isnan(float(v)))
                            or float(v) == a):
                        bad2.append((k, v, a))
                elif str(v) != str(a):
                    bad2.append((k, v, a))
            check(f"A/{name} V1 archive row reproduced ({n_cmp} of "
                  f"{len(arch)} archived columns; ctrl_ms/seed excluded)",
                  not bad2, "" if not bad2 else f"{bad2[:3]}")

    # ---- layer inertness: M1 with the obstacle term off == M0, and
    #      M2 with heterogeneity/delay off and no speed-up == M1 ----
    #   NB the comparison configs keep the Stage-1 controller layer
    #   (speed_policy="off"), otherwise the *controller* would differ too and
    #   the test would no longer isolate the behaviour model.
    log("  layer inertness (the added terms are inactive => exactly the layer below)")
    sid, seed = "S3", 200
    base = default_v2cfg()
    _, r0 = run_cell(sid, "Ours_rollout", seed, v2cfg=base, return_res=True,
                     record_states=True)
    m1off = base.with_(animal_model="M1", m1_gain=0.0, m_s_min=1.0,
                       m_v_max=1.5)
    _, r1 = run_cell(sid, "Ours_rollout", seed, v2cfg=m1off, return_res=True,
                     record_states=True)
    bad = diff_traces(trace_arrays(r0), trace_arrays(r1))
    check("A/M1(m1_gain=0) == M0, bit-exact", not bad, "" if not bad
          else f"{bad[:3]}")
    m1on = base.with_(animal_model="M1", m1_gain=SHIPPED["m1_gain"],
                      m1_decay=SHIPPED["m1_decay"])
    m2off = base.with_(animal_model="M2", m1_gain=SHIPPED["m1_gain"],
                       m1_decay=SHIPPED["m1_decay"], m2_gain_sigma=0.0,
                       m2_react_delay=0.0, m2_speed_map=False)
    _, ra = run_cell(sid, "Ours_rollout", seed, v2cfg=m1on, return_res=True,
                     record_states=True)
    _, rb = run_cell(sid, "Ours_rollout", seed, v2cfg=m2off, return_res=True,
                     record_states=True)
    bad = diff_traces(trace_arrays(ra), trace_arrays(rb))
    check("A/M2(sigma=0, delay=0, no speed-up) == M1, bit-exact", not bad,
          "" if not bad else f"{bad[:3]}")
    return allok


# ===========================================================================
# B. M1 actually avoids
# ===========================================================================
B_KEYS = ["animal_contact_fraction", "n_contact_events_so",
          "mean_min_clearance", "worst_min_clearance", "speed_mean",
          "speed_p95", "speed_max", "frac_stopped", "omega_mean", "omega_p95",
          "omega_max", "frac_omega_at_cap", "flip_rate_mean", "flip_rate_p95",
          "path_efficiency_mean", "inside_frac", "max_abs_lateral",
          "radius95_max", "n_exploded", "onset_latency", "sep_guard",
          "body_priority"]


def _agg(rows):
    out = {}
    for k in B_KEYS:
        v = [r.get(k, np.nan) for r in rows]
        v = np.array([x for x in v if x is not None], dtype=float)
        out[k] = float(np.nanmean(v)) if v.size else np.nan
        out[k + "_sd"] = float(np.nanstd(v, ddof=1)) if v.size > 1 else 0.0
    out["finite_state"] = bool(all(bool(r.get("finite_state", True))
                                   for r in rows))
    return out


def test_B(log):
    log("=" * 78)
    log("B. M1 actually avoids: corridor crossing, NO robots, seeds "
        f"{CAL_SEEDS[0]}-{CAL_SEEDS[-1]}")
    settings = [("M0", "M0", {})]
    for R in (3.0, 6.0, 9.0):
        settings.append((f"M1_R{int(R)}", "M1",
                         dict(SHIPPED, m1_sense_range=R)))
    table = []
    detail = {}
    for label, model, over in settings:
        cfg = stage2_v2cfg(model, **over)
        rows = []
        for s in CAL_SEEDS:
            m, _ = run_corridor(s, cfg)
            rows.append(m)
        a = _agg(rows)
        a.update(setting=label, animal_model=model,
                 sense_range=cfg.m1_sense_range, gain=cfg.m1_gain,
                 decay=cfg.m1_decay)
        table.append(a)
        detail[label] = rows
        log(f"  {label:8s} contact={a['animal_contact_fraction']:.3f}"
            f"+-{a['animal_contact_fraction_sd']:.3f}  "
            f"episodes={a['n_contact_events_so']:.1f}  "
            f"mean_clr={a['mean_min_clearance']:6.2f}  "
            f"worst={a['worst_min_clearance']:6.2f}  "
            f"speed[p95]={a['speed_p95']:.2f}/{a['speed_max']:.2f}  "
            f"omega[p95]={a['omega_p95']:.2f}/{a['omega_max']:.2f}  "
            f"flips={a['flip_rate_mean']:.2f}/s  eff={a['path_efficiency_mean']:.2f}")
    write_csv(BEH / "B_corridor.csv", table)
    write_csv(BEH / "B_corridor_per_seed.csv",
              [dict(setting=k, seed=s, **{kk: dd.get(kk) for kk in B_KEYS})
               for k, rows in detail.items() for s, dd in zip(CAL_SEEDS, rows)])

    m0 = table[0]
    c0 = m0["animal_contact_fraction"]
    cl0 = m0["mean_min_clearance"]
    seq = table[1:]
    better = [t["animal_contact_fraction"] <= c0 + 1e-9 for t in seq]
    mono_c = all(seq[i]["animal_contact_fraction"] >=
                 seq[i + 1]["animal_contact_fraction"] - 1e-9
                 for i in range(len(seq) - 1))
    mono_cl = all(seq[i]["mean_min_clearance"] <=
                  seq[i + 1]["mean_min_clearance"] + 1e-9
                  for i in range(len(seq) - 1))
    check("B/M1 beats M0 at every sensing range", all(better),
          f"contact {c0:.3f} -> " + ", ".join(
              f"{t['animal_contact_fraction']:.3f}" for t in seq))
    check("B/contact rate monotone non-increasing in range", mono_c,
          ", ".join(f"R{t['sense_range']:.0f}:{t['animal_contact_fraction']:.3f}"
                    for t in seq))
    check("B/mean clearance monotone non-decreasing in range", mono_cl,
          ", ".join(f"R{t['sense_range']:.0f}:{t['mean_min_clearance']:.2f}"
                    for t in seq))
    check("B/M1 mean clearance beats M0 at the widest range",
          seq[-1]["mean_min_clearance"] > cl0,
          f"{cl0:.2f} -> {seq[-1]['mean_min_clearance']:.2f}")
    # no oscillation / explosion
    ok_osc = all(t["omega_max"] <= 3.0 * (1 + 1e-6) for t in table)
    ok_spd = all(t["speed_max"] <= 2.0 * (1 + 1e-6) for t in table)
    ok_exp = all(t["finite_state"] for t in table) and \
        all(t["radius95_max"] < 25.0 for t in table)
    ok_flip = all(t["flip_rate_mean"] < 12.0 for t in table)
    check("B/no heading-rate explosion (|dtheta|/dt <= omega_s)", ok_osc,
          f"worst omega {max(t['omega_max'] for t in table):.3f}")
    check("B/no speed explosion (s <= m_v_max)", ok_spd,
          f"worst speed {max(t['speed_max'] for t in table):.3f}")
    check("B/no spatial explosion (finite states, group p95 radius < 25 m)",
          ok_exp, "p95 radius max "
          + ", ".join(f"{t['setting']}:{t['radius95_max']:.1f}" for t in table))
    check("B/oscillation bounded (heading-rate sign flips < 12 /s)", ok_flip,
          ", ".join(f"{t['setting']}:{t['flip_rate_mean']:.1f}" for t in table))
    return True


# ===========================================================================
# C. delay and heterogeneity
# ===========================================================================
def test_C(log):
    log("=" * 78)
    log("C. reaction delay and gain heterogeneity are real (corridor)")
    # The delay/heterogeneity comparisons use the *short* sensing range
    # (R = 3 m): at R = 6 m the avoidance saturates (contact = 0 for every
    # setting, see test B) and no outcome difference could be measured.
    base3 = dict(SHIPPED, m1_sense_range=3.0)
    seeds = CAL_SEEDS
    from v2.corridor_v2 import onset_latency

    # ---- C1: onset latency vs reaction delay ----
    #   Two measures, both from the same runs:
    #     (i)  "20% threshold" latency (frac=0.2): how long until the animal
    #          has received a substantial stimulus -- used for the trend;
    #     (ii) "first response" latency (frac=0): the first tick with a nonzero
    #          obstacle term.  Before that instant the animal's trajectory is
    #          identical in every setting, so the *animal-paired* difference is
    #          the delay itself -- an exact causal check that the delay reaches
    #          the response instead of being absorbed.
    delays = (0.0, 0.15, 0.30, 0.45)
    lat = {}
    con = {}
    clr = {}
    per_animal = {}
    firsts = {}
    for d in delays:
        cfg = stage2_v2cfg("M2", **dict(base3, m2_react_delay=d))
        L, Cc, Cl, PA, PF = [], [], [], [], []
        for s in seeds:
            m, res = run_corridor(s, cfg)
            L.append(m["onset_latency"])
            Cc.append(m["animal_contact_fraction"])
            Cl.append(m["mean_min_clearance"])
            PA.append(onset_latency(res, cfg, frac=0.2, per_animal=True))
            PF.append(onset_latency(res, cfg, frac=0.0, per_animal=True))
        lat[d] = np.array(L, dtype=float)
        con[d] = np.array(Cc)
        clr[d] = np.array(Cl)
        per_animal[d] = np.array(PA)                 # (seeds, n_animals)
        firsts[d] = np.array(PF)
    rows = []
    for d in delays:
        if d > 0:
            pa, p0 = per_animal[d], per_animal[0.0]
            ok = np.isfinite(pa) & np.isfinite(p0)
            paired = float(np.mean((pa - p0)[ok])) if ok.any() else float("nan")
            npair = int(ok.sum())
            fa, f0 = firsts[d], firsts[0.0]
            ok2 = np.isfinite(fa) & np.isfinite(f0)
            paired_first = float(np.mean((fa - f0)[ok2])) if ok2.any() \
                else float("nan")
            npair2 = int(ok2.sum())
        else:
            paired = paired_first = 0.0
            npair = int(np.isfinite(per_animal[0.0]).sum())
            npair2 = int(np.isfinite(firsts[0.0]).sum())
        rows.append(dict(react_delay=d, latency=float(np.nanmean(lat[d])),
                         latency_sd=float(np.nanstd(lat[d], ddof=1)),
                         paired_latency=paired, paired_n=npair,
                         paired_first_response=paired_first,
                         paired_first_n=npair2,
                         contact=float(np.mean(con[d])),
                         contact_sd=float(np.std(con[d], ddof=1)),
                         clearance=float(np.mean(clr[d])),
                         clearance_sd=float(np.std(clr[d], ddof=1)),
                         sense_range=3.0))
        log(f"  delay={d:4.2f} s  latency={rows[-1]['latency']:.3f}"
            f"+-{rows[-1]['latency_sd']:.3f}  first-response paired +"
            f"{rows[-1]['paired_first_response']:.3f} ({npair2} animals)  "
            f"contact={rows[-1]['contact']:.4f}+-{rows[-1]['contact_sd']:.4f}  "
            f"clearance={rows[-1]['clearance']:.3f}")
    write_csv(BEH / "C_delay.csv", rows)
    gains = [rows[i]["paired_first_response"] / (delays[i] - delays[0])
             for i in range(1, len(delays))]
    log(f"  animal-paired first-response gain per unit delay: "
        f"{[round(g, 3) for g in gains]}")
    #   The measurement quantum is the integration step (dt = 0.05 s), and the
    #   smallest delay step is 3 ticks, so the check is an *absolute* one:
    #   the animal-paired shift must equal the delay to within 2 ticks.
    dev = [rows[i]["paired_first_response"] - (delays[i] - delays[0])
           for i in range(1, len(delays))]
    log(f"  animal-paired first-response shift minus the delay [s]: "
        f"{[round(x, 3) for x in dev]} (2 ticks = 0.10 s)")
    check("C/animal-paired first response is shifted by the reaction delay "
          "(within 2 ticks)", all(abs(x) <= 0.10 for x in dev),
          f"shift-minus-delay={[round(x, 3) for x in dev]} s, "
          f"gains={[round(g, 3) for g in gains]}")
    check("C/latency monotone non-decreasing in delay",
          all(np.nanmean(lat[delays[i]]) <= np.nanmean(lat[delays[i + 1]]) + 1e-9
              for i in range(len(delays) - 1)),
          f"{[round(float(np.nanmean(lat[d])), 3) for d in delays]}")
    # dynamic consequence: with a real reaction delay the avoidance degrades
    worse = np.mean(con[delays[-1]]) >= np.mean(con[0.0]) - 1e-9
    log(f"  dynamic consequence at R=3: contact {np.mean(con[0.0]):.4f} "
        f"(no delay) -> {np.mean(con[delays[-1]]):.4f} (0.45 s delay)")
    check("C/delay degrades the avoidance outcome (0.45 s vs 0 s)", worse,
          f"contacts {np.mean(con[0.0]):.4f} -> {np.mean(con[delays[-1]]):.4f}")

    # ---- C2: per-animal gain distribution ----
    cfg_h = stage2_v2cfg("M2", **SHIPPED)
    from v2.animal_models import make_animal_model
    models = [make_animal_model(cfg_h, default_cfg(n_sheep=36, n_robots=0), s)
              for s in CAL_SEEDS[:8]]
    gains = np.concatenate([m.gain for m in models])
    q = np.percentile(gains, [5, 25, 50, 75, 95]).tolist()
    log(f"  gains: mean={gains.mean():.4f} sd={gains.std(ddof=1):.4f} "
        f"min={gains.min():.4f} max={gains.max():.4f} p5/25/50/75/95="
        f"{[round(v, 3) for v in q]}")
    write_csv(BEH / "C_gains.csv",
              [dict(seed=s, gain=float(m.gain[i]))
               for s, m in zip(CAL_SEEDS[:8], models) for i in range(len(m.gain))])
    check("C/gain spread is applied (sd > 0, min < 1 < max)",
          gains.std(ddof=1) > 1e-3 and gains.min() < 1.0 < gains.max(),
          f"sd={gains.std(ddof=1):.4f} range=[{gains.min():.3f},"
          f"{gains.max():.3f}]")
    check("C/gain mean is 1 (same total gain as M1)",
          abs(gains.mean() - 1.0) < 1e-9, f"mean={gains.mean():.6f}")
    check("C/gains are seed-dependent",
          not np.allclose(models[0].gain, models[1].gain),
          f"first two animals seed {CAL_SEEDS[0]}: "
          f"{models[0].gain[:2].round(4).tolist()} vs "
          f"{models[1].gain[:2].round(4).tolist()}")

    # ---- C3: removing heterogeneity changes the outcome distribution ----
    het_rows = []
    for label, sig in (("hetero", SHIPPED["m2_gain_sigma"]), ("homo", 0.0)):
        cfg = stage2_v2cfg("M2", **dict(base3, m2_gain_sigma=sig))
        Cc, Cl = [], []
        for s in seeds:
            m, _ = run_corridor(s, cfg)
            Cc.append(m["animal_contact_fraction"])
            Cl.append(m["mean_min_clearance"])
        het_rows.append(dict(setting=label, sigma=sig, sense_range=3.0,
                             contact=float(np.mean(Cc)),
                             contact_sd=float(np.std(Cc, ddof=1)),
                             clearance=float(np.mean(Cl)),
                             clearance_sd=float(np.std(Cl, ddof=1)),
                             per_seed_contact=Cc, per_seed_clear=Cl))
        log(f"  {label:6s} sigma={sig:.2f}  contact={np.mean(Cc):.4f}"
            f"+-{np.std(Cc, ddof=1):.4f}  clearance={np.mean(Cl):.3f}"
            f"+-{np.std(Cl, ddof=1):.3f}")
    a = np.array(het_rows[0]["per_seed_clear"])
    b = np.array(het_rows[1]["per_seed_clear"])
    ca = np.array(het_rows[0]["per_seed_contact"])
    cb = np.array(het_rows[1]["per_seed_contact"])
    d = a - b
    # exact sign-flip permutation test on the paired clearance difference
    obs = abs(d.mean())
    rng = np.random.default_rng(0)
    cnt = 0
    for _ in range(20000):
        sg = rng.choice([-1.0, 1.0], len(d))
        if abs((d * sg).mean()) >= obs - 1e-15:
            cnt += 1
    p = (cnt + 1) / 20001
    rows_out = [{k: v for k, v in r.items()
                 if not k.startswith("per_seed")} for r in het_rows]
    for r in rows_out:
        r["paired_mean_delta_clearance"] = float(d.mean())
        r["paired_sd_delta_clearance"] = float(np.std(d, ddof=1))
        r["perm_p"] = p
    write_csv(BEH / "C_heterogeneity.csv", rows_out)
    log(f"  paired clearance difference hetero-homo = {d.mean():+.4f} m "
        f"(sd {np.std(d, ddof=1):.4f}, {int((np.abs(d) > 1e-9).sum())}/"
        f"{len(d)} seeds differ), permutation p={p:.4f}")
    check("C/removing heterogeneity changes the outcome distribution",
          (np.abs(d) > 1e-9).any() and np.std(d, ddof=1) > 1e-9,
          f"paired clearance diff {d.mean():+.4f} m, "
          f"sd {np.std(d, ddof=1):.4f}, p={p:.4f}; contact "
          f"{ca.mean():.4f} vs {cb.mean():.4f}")
    return True


# ===========================================================================
# D. predictor / environment decoupling
# ===========================================================================
class _FakeObs:
    def __init__(self, r):
        self.radius = float(r)


class _FakeView:
    def __init__(self, path, spos, svel, rpos):
        self.path = path
        self.sheep_pos = spos
        self.sheep_vel = svel
        self.robot_pos = rpos
        self.obs_pos = np.array([[30.0, 6.0]])
        self.obs_vel = np.array([[-0.6, -0.4]])
        self.obstacles_active = [_FakeObs(1.2)]
        self.t = 12.0

    @property
    def centroid(self):
        return self.sheep_pos.mean(axis=0)


def test_D(log):
    log("=" * 78)
    log("D. predictor / environment decoupling")
    # ---- D1: the NumPy engine reproduces the V1 DLL on M0 ----
    from v2.controller_v2 import OursRolloutV2
    path = corridor_scene()["path"]
    rng = np.random.default_rng(5)
    gx, gy = np.meshgrid(np.arange(6) - 2.5, np.arange(6) - 2.5)
    spos = np.stack([16 + gx.ravel() * 1.55, gy.ravel() * 1.1], axis=1)
    spos += rng.uniform(-0.1, 0.1, spos.shape)
    svel = np.stack([np.ones(36), np.zeros(36)], axis=1)
    ang = 2 * np.pi * np.arange(12) / 12 + np.pi / 6
    rpos = np.stack([16 + 3.0 * np.cos(ang), 3.0 * np.sin(ang)], axis=1)
    view = _FakeView(path, spos, svel, rpos)
    cfg_sim = replace(default_cfg(n_sheep=36, n_robots=12),
                      **sim_overrides(default_v2cfg()))
    offs = np.array([0.0, -1.35, 1.35, -2.4, 2.4, -3.6, 3.6])
    # speed policy off on both sides: the v2 engine *models* the controller rule
    # when predictor_speed_policy is on, which is a deliberate behavioural
    # difference, not a porting error; the port is judged with it off.
    c_dll = OursRolloutV2(v2cfg=default_v2cfg().with_(
        predictor_speed_policy=False))
    c_np = OursRolloutV2(v2cfg=default_v2cfg().with_(
        predictor_engine="v2", predictor_speed_policy=False))
    s0, _, _ = path.closest(view.centroid)
    d0 = path.tangent_at(s0)
    for c in (c_dll, c_np):
        c._heading = np.full(12, np.arctan2(d0[1], d0[0]))
        c._previous_pos = rpos.copy()
        c._previous_t = view.t - 0.1

    def _both(steps):
        hz = c_dll.horizon, c_np.horizon
        c_dll.horizon = c_np.horizon = steps * c_dll.prediction_dt
        a = c_dll._predict(view, cfg_sim, offs)
        b = c_np._predict(view, cfg_sim, offs)
        c_dll.horizon, c_np.horizon = hz
        return a, b

    o_dll, o_np = _both(8)
    denom = np.maximum(np.abs(o_dll), 1e-12)
    rel = float(np.max(np.abs(o_np - o_dll) / denom))
    absd = float(np.max(np.abs(o_np - o_dll)))
    log(f"  M0 engine agreement, 8 prediction steps (DLL vs NumPy): "
        f"max rel={rel:.3e}  max abs={absd:.3e}")
    o_dll_long, o_np_long = _both(80)
    same_long = int(np.sum(np.all(
        np.abs(o_np_long - o_dll_long) <= 1e-12
        * np.maximum(np.abs(o_dll_long), 1.0), axis=1)))
    log(f"  full 80-step horizon: {same_long}/{len(offs)} candidate offsets "
        f"agree to 1e-12; the rest diverge through a discrete slot-spacing / "
        f"contact test flipping (chaotic amplification of a 1e-16 summation-"
        f"order difference), max abs={float(np.max(np.abs(o_np_long - o_dll_long))):.3e}")
    write_csv(BEH / "D_engine.csv",
              [dict(engine_a="dll", engine_b="v2", model="M0",
                    steps=8, max_rel=rel, max_abs=absd,
                    long_horizon_rows_identical=same_long,
                    long_horizon_max_abs=float(np.max(np.abs(o_np_long
                                                              - o_dll_long))),
                    dll_short=o_dll.tolist(), v2_short=o_np.tolist())])
    check("D/NumPy engine reproduces the V1 DLL on M0 (8 steps, <=1e-8 rel)",
          rel <= 1e-8, f"max rel={rel:.3e}")

    # ---- D2: shadow predictor -- same environment, different predictions ----
    env = stage2_v2cfg("M2", **dict(SHIPPED, predictor_shadow=True))
    runs = {}
    for model in ("M0", "M2"):
        cfg = env.with_(predictor_model=model)
        _, res = run_cell("S2", "Ours_rollout", 1000, v2cfg=cfg,
                          return_res=True, record_states=True)
        runs[model] = res
    ta, tb = trace_arrays(runs["M0"]), trace_arrays(runs["M2"])
    bad = diff_traces(ta, tb)
    log(f"  shadow runs: {len(ta['t'])} ticks, "
        f"environment trajectory identical = {not bad}")
    la = runs["M0"]["shadow_log"]
    lb = runs["M2"]["shadow_log"]
    same_len = len(la) == len(lb) and len(la) > 0
    maxabs = 0.0
    n_diff = 0
    col_abs = np.zeros(7)
    if same_len:
        for e1, e2 in zip(la, lb):
            a = np.array(e1["outcomes"])
            b = np.array(e2["outcomes"])
            dd = np.abs(a - b)
            maxabs = max(maxabs, float(dd.max()))
            col_abs = np.maximum(col_abs, dd.max(axis=0))
            if dd.max() > 1e-9:
                n_diff += 1
    maxrel = 0.0
    if same_len:
        for e1, e2 in zip(la, lb):
            a = np.array(e1["outcomes"])
            b = np.array(e2["outcomes"])
            big = np.abs(a) > 1e-6
            if big.any():
                maxrel = max(maxrel, float((np.abs(a - b)[big]
                                            / np.abs(a)[big]).max()))
    log(f"  predictions: {len(la)} planning calls, {n_diff} differ; "
        f"max|dpred|={maxabs:.3e}, max rel (|a|>1e-6)={maxrel:.3e}")
    log(f"  per-column max |difference| (contacted, pressure, outside, centre,"
        f" spread, progress, worst): {np.round(col_abs, 4).tolist()}")
    cols = {}
    if same_len:
        k = 6
        cols[("M0", "worst")] = [float(np.array(e["outcomes"])[k, 6])
                                 for e in la]
        cols[("M2", "worst")] = [float(np.array(e["outcomes"])[k, 6])
                                 for e in lb]
        log(f"  predicted worst margin at offset +3.6 m: "
            f"M0 first/last={cols[('M0','worst')][0]:.3f}/"
            f"{cols[('M0','worst')][-1]:.3f}  "
            f"M2 first/last={cols[('M2','worst')][0]:.3f}/"
            f"{cols[('M2','worst')][-1]:.3f}")
    write_csv(BEH / "D_decoupling.csv",
              [dict(env_model="M2", predictor_a="M0", predictor_b="M2",
                    ticks=len(ta["t"]), env_identical=not bad,
                    planning_calls=len(la), calls_that_differ=n_diff,
                    max_abs_diff=maxabs, max_rel_diff_gt1e6=maxrel,
                    **{f"col{i}_maxdiff": float(col_abs[i])
                       for i in range(7)},
                    worst_M0_first=(cols[("M0", "worst")][0] if cols else None),
                    worst_M2_first=(cols[("M2", "worst")][0] if cols else None))])
    check("D/shadow: environment trajectory identical (bit-exact)", not bad,
          f"{bad[:2]}")
    check("D/shadow: predictions differ between predictor models",
          same_len and n_diff > 0, f"{n_diff}/{len(la)} calls differ")

    # ---- D3: the predictor actually drives decisions (live mode) ----
    live = {}
    for model in ("M0", "M2"):
        cfg = stage2_v2cfg("M0", predictor_engine="v2", predictor_model=model)
        row, res = run_cell("S2", "Ours_rollout", 1000, v2cfg=cfg,
                            return_res=True)
        live[model] = (row, res)
    ctrl = live["M2"][1]["controller_obj"]
    from v2.controller_v2 import unpack
    core, pol, shadow = unpack(ctrl)
    off_a = [p["offset"] for p in live["M0"][1]["controller_obj"].plans] \
        if hasattr(live["M0"][1]["controller_obj"], "plans") else []
    core_a, _, _ = unpack(live["M0"][1]["controller_obj"])
    off_a = [p["offset"] for p in core_a.plans]
    off_b = [p["offset"] for p in core.plans]
    log(f"  live mode: {len(off_a)} vs {len(off_b)} plans; "
        f"offsets M0-predictor {off_a[:8]} ...")
    log(f"             offsets M2-predictor {off_b[:8]} ...")
    write_csv(BEH / "D_live.csv",
              [dict(predictor=a, n_plans=len(o), offsets=o, lost=live[a][1]["lost"],
                    terminated=live[a][1]["terminated_by"])
               for a, o in (("M0", off_a), ("M2", off_b))])
    check("D/live: a different predictor model changes the actuated plan",
          off_a != off_b, f"{len(off_a)} vs {len(off_b)} plans")
    return True


# ===========================================================================
# E. determinism
# ===========================================================================
def _csv_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def test_E(log):
    log("=" * 78)
    log("E. determinism")
    # ---- E1: repeat run, whole trajectory ----
    cfg = stage2_v2cfg("M2", **SHIPPED)
    ok = True
    for sid, seed in (("S3", 1000), ("S2", 1001)):
        _, r1 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        _, r2 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        bad = diff_traces(trace_arrays(r1), trace_arrays(r2))
        ok &= not bad
        check(f"E/repeat run identical ({sid}, seed {seed}, "
              f"{len(r1['trace_states'])} ticks)", not bad, f"{bad[:2]}")
    # ---- E2: jobs=1 vs jobs=4 ----
    from v2 import runner_v2
    import subprocess
    outs = {}
    for jobs in (1, 4):
        d = OUT / f"_det_j{jobs}"
        d.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, "-X", "utf8", "-m", "v2.runner_v2",
               "--scenarios", "S3", "--seeds", "2", "--seed-offset", "1000",
               "--algs", "Ours_rollout", "--jobs", str(jobs), "--out", str(d),
               "--set"] + [f"{k}={v}" for k, v in SHIPPED.items()] + \
              ["animal_model=M2", "speed_policy=manoeuvre"]
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        if r.returncode != 0:
            log("  runner failed:\n" + r.stdout[-2000:] + r.stderr[-2000:])
        outs[jobs] = _csv_rows(d / "detail.csv")
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
    write_csv(BEH / "E_jobs.csv",
              [dict(rows=len(outs[1]), columns=len(keys), mismatches=ndiff,
                    example=str(worst))])
    check("E/jobs=1 == jobs=4, every non-timing column", ndiff == 0,
          f"{ndiff} mismatches" + (f", e.g. {worst}" if worst else ""))
    return ok


# ===========================================================================
# F. joint limits in M1 / M2
# ===========================================================================
F_KEYS = ["violation_C1_low", "violation_C1_high", "violation_C2",
          "violation_C3", "violation_R1", "violation_R2", "violation_R3",
          "violation_R4", "violation_lat", "bind_C3_sheep", "bind_C2_sheep",
          "bind_R4_robot", "bind_LAT_robot", "bind_R3_robot",
          "opp_sheep_steps", "opp_robot_steps", "LAT_UNRESOLVED",
          "plant_infeasible", "n_contact_events_so", "collision_rate",
          "dmin_robot_robot", "min_surface_clearance_rr",
          "min_surface_clearance_ro", "min_surface_clearance_rs",
          "min_surface_clearance_so", "sp_n_bounded_ticks",
          "sp_r_req_mean", "sp_v_allow_mean", "sp_v_allow_min",
          "speed_mean", "speed_max", "success", "terminated_by",
          "rho_avg", "max_dev"]


def test_F(log):
    log("=" * 78)
    log("F. joint limits still hold (and bind) in M1/M2")
    rows = []
    ok = True
    for model in ("M1", "M2"):
        for seed in (1000, 1001):
            cfg = stage2_v2cfg(model, **SHIPPED)
            row, res = run_cell("S3", "Ours_rollout", seed, v2cfg=cfg,
                                return_res=True, record_states=True)
            m = summarize_v2(res, cfg)
            tr = res["trace_states"]
            spd = np.array([r[5] for r in tr])
            m["speed_mean"] = float(spd.mean())
            m["speed_max"] = float(spd.max())
            r = dict(model=model, seed=seed,
                     **{k: m.get(k) for k in F_KEYS})
            rows.append(r)
            log(f"  {model} seed {seed}: C3={r['violation_C3']} "
                f"R4={r['violation_R4']} lat={r['violation_lat']} "
                f"C1low={r['violation_C1_low']} C1high={r['violation_C1_high']} "
                f"| bind C3={r['bind_C3_sheep']} R4={r['bind_R4_robot']} "
                f"lat={r['bind_LAT_robot']} | speed {r['speed_mean']:.3f}"
                f"/{r['speed_max']:.3f} | v_allow {r['sp_v_allow_mean']} "
                f"(min {r['sp_v_allow_min']}) | animal contacts "
                f"{r['n_contact_events_so']} | task success {r['success']} "
                f"({r['terminated_by']}) rho {r['rho_avg']}")
            ok &= all(float(r[k] or 0) == 0.0 for k in
                      ("violation_C3", "violation_R4", "violation_lat",
                       "violation_C1_low", "violation_C1_high",
                       "violation_C2", "LAT_UNRESOLVED"))
            ok &= float(r["bind_C3_sheep"] or 0) > 0
            ok &= float(r["bind_R4_robot"] or 0) > 0
            ok &= float(r["bind_LAT_robot"] or 0) > 0
    write_csv(BEH / "F_limits.csv", rows)
    check("F/C3, R4, lateral and C1-band violations are 0 in M1 and M2", ok)
    check("F/C3 and R4 and the lateral cap still bind (non-vacuous)",
          all(float(r["bind_C3_sheep"] or 0) > 0
              and float(r["bind_R4_robot"] or 0) > 0
              and float(r["bind_LAT_robot"] or 0) > 0 for r in rows))
    # the speed policy must actually reduce the commanded speed in DIVERT
    bounded = [r for r in rows if float(r["sp_n_bounded_ticks"] or 0) > 0]
    check("F/speed policy applied during pre/divert",
          len(bounded) == len(rows),
          f"{len(bounded)}/{len(rows)} runs with bounded ticks")
    return ok


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=list("ABCDEF"))
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    BEH.mkdir(parents=True, exist_ok=True)
    load_shipped()
    log = Tee(OUT / "ACCEPTANCE.txt")
    LOG[0] = log
    log("V2 Stage-2 acceptance tests")
    log(f"shipped parameters: {json.dumps(SHIPPED, sort_keys=True)}")
    log(f"python {sys.version.split()[0]}, numpy {np.__version__}")
    t0 = time.time()
    fn = dict(A=test_A, B=test_B, C=test_C, D=test_D, E=test_E, F=test_F)
    for t in args.tests:
        fn[t](log)
    log("=" * 78)
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    log(f"SUMMARY: {npass}/{len(RESULTS)} checks passed "
        f"({time.time() - t0:.1f} s)")
    for name, ok, detail in RESULTS:
        if not ok:
            log(f"  FAILED: {name}  {detail}")
    (OUT / "ACCEPTANCE.json").write_text(json.dumps(
        [dict(name=n, ok=ok, detail=d) for n, ok, d in RESULTS],
        indent=2), encoding="utf-8")
    return 0 if npass == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
