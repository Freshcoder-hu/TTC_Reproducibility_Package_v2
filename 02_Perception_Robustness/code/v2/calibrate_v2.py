"""Stage-2 calibration (seeds 1000-1019 ONLY; evaluation seeds 2000-2029 are
never touched here).

    python -X utf8 -m v2.calibrate_v2 --stage all

WHAT IS CALIBRATED
------------------
M1 : obstacle-avoidance gain ``m1_gain`` and distance-decay shape ``m1_decay``
     (linear / quadratic / exponential), at the mid sensing range R = 6 m.
M2 : reaction delay ``m2_react_delay``, gain spread ``m2_gain_sigma``,
     speed-target mapping (``m_v_max``, ``m2_slow_gain``) on top of the chosen
     M1 values.  The heterogeneity *magnitude* is searched with sigma > 0 only:
     heterogeneity itself is part of the model definition (author decision),
     not a calibration option.
Then a one-factor-at-a-time sensitivity table around the chosen values, and the
sensing-range sweep 3/6/9 m (which is also acceptance-test-B data).

OBJECTIVE (documented in the report)
------------------------------------
    J = 100 * contact_fraction          (avoidance success; primary)
      + 10 * max(0, -worst_clearance)   (never let the obstacle penetrate)
      +  5 * max(0, 0.5 - path_eff)     (animals must still move as a group)
      +  2 * max(0, latency - 2.0)      (no implausibly slow response)
      +  2 * max(0, 0.3 - latency)      (no instantaneous reaction)

``contact_fraction`` is the fraction of animals whose minimum surface gap to the
obstacle ever drops to 0 (or below); ``worst_clearance`` its minimum.  Every
component is written to the CSV, so the choice can be re-judged from the table
instead of from J alone.

Corridor keeping (``inside_frac``) is **reported but not in J**: with no robots
present no model can hold the corridor, so the term would penalise animals *for
avoiding* (dodging leaves the corridor) instead of measuring the avoidance.

TIE-BREAK (parsimony, fixed before the run)
-------------------------------------------
Cells within ``TIE_TOL`` of the best J are treated as equivalent, and the
simplest of them is shipped: ``linear`` before ``quadratic`` before ``exp``,
and the smaller gain.  This avoids shipping an over-avoidant parameter set when
the data cannot distinguish it.

BUDGET
------
The script counts runs and wall-clock seconds and writes them to
``calibration_summary.json``; the report quotes exactly those numbers.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from v2.config_v2 import stage2_v2cfg  # noqa: E402
from v2.corridor_v2 import run_corridor  # noqa: E402

OUT = ROOT / "results_v2_stage2"
CAL_SEEDS = list(range(1000, 1020))     # 1000-1019 ONLY
N_CAL = 5                              # seeds per cell during the search

BUDGET = dict(runs=0, seconds=0.0)


def evaluate(**mods):
    """Run one configuration on the calibration seed subset."""
    v2cfg = stage2_v2cfg(mods.pop("animal_model", "M2"), **mods)
    rows = []
    for s in CAL_SEEDS[:N_CAL]:
        t0 = time.time()
        m, _ = run_corridor(s, v2cfg)
        BUDGET["runs"] += 1
        BUDGET["seconds"] += time.time() - t0
        rows.append(m)
    agg = {k: float(np.nanmean([r[k] for r in rows]))
           for k in ("animal_contact_fraction", "mean_min_clearance",
                     "worst_min_clearance", "path_efficiency_mean",
                     "inside_frac", "onset_latency", "speed_mean",
                     "speed_max", "flip_rate_mean", "omega_max",
                     "n_contact_events_so")}
    agg["J"] = objective(agg)
    return agg


def objective(a):
    return (100.0 * a["animal_contact_fraction"]
            + 10.0 * max(0.0, -a["worst_min_clearance"])
            + 5.0 * max(0.0, 0.5 - a["path_efficiency_mean"])
            + 2.0 * max(0.0, (a["onset_latency"] if np.isfinite(
                a["onset_latency"]) else 0.0) - 2.0)
            + 2.0 * max(0.0, 0.3 - (a["onset_latency"] if np.isfinite(
                a["onset_latency"]) else 0.3)))


TIE_TOL = 0.5
_DECAY_ORDER = {"linear": 0, "quadratic": 1, "exp1": 2, "exp3": 3}


def select_best(rows, tag="decay"):
    """Parsimony tie-break: best J within TIE_TOL, then simplest shape/gain."""
    best = min(r["J"] for r in rows)
    cand = [r for r in rows if r["J"] <= best + TIE_TOL]
    cand.sort(key=lambda r: (_DECAY_ORDER.get(r.get(tag, ""), 9), r["gain"]))
    return cand[0], best, len(cand)


# ---------------------------------------------------------------------------
def stage_m1(log):
    log("=" * 78)
    log("M1 calibration: gain x decay shape, R = 6 m, seeds "
        + ",".join(str(s) for s in CAL_SEEDS[:N_CAL]))
    grid = []
    for decay in ("linear", "quadratic", "exp1", "exp3"):
        for gain in (0.5, 1.0, 2.0, 3.0):
            grid.append((decay, gain))
    rows = []
    for decay, gain in grid:
        d = {"m1_decay": "exp" if decay.startswith("exp") else decay,
             "m1_gain": gain, "m1_sense_range": 6.0}
        if decay == "exp1":
            d["m1_decay_len"] = 1.0
        if decay == "exp3":
            d["m1_decay_len"] = 3.0
        a = evaluate(animal_model="M1", **d)
        a.update(decay=decay, gain=gain, sense_range=6.0, model="M1")
        rows.append(a)
        log(f"  {decay:9s} k={gain:3.1f}  J={a['J']:7.3f}  "
            f"contact={a['animal_contact_fraction']:.3f}  "
            f"clear_mean={a['mean_min_clearance']:6.2f}  "
            f"worst={a['worst_min_clearance']:6.2f}  "
            f"lat={a['onset_latency']:.3f}  eff={a['path_efficiency_mean']:.2f}")
    best, bestJ, nties = select_best(rows)
    log(f"  -> best J={bestJ:.3f}; {nties} cell(s) within {TIE_TOL}; "
        f"shipped by parsimony: decay={best['decay']} gain={best['gain']} "
        f"(J={best['J']:.3f})")
    return rows, best


def stage_m2(log, m1):
    log("=" * 78)
    log("M2 calibration (coordinate search on the chosen M1 values)")
    base = dict(animal_model="M2", m1_decay=m1["decay"], m1_gain=m1["gain"],
                m1_sense_range=6.0)
    if m1["decay"].startswith("exp"):
        base["m1_decay_len"] = 1.0 if m1["decay"] == "exp1" else 3.0
    chosen = dict(m2_react_delay=0.0, m2_gain_sigma=0.2, m_v_max=2.0,
                  m2_slow_gain=1.0)
    rows = []
    # -- 1. reaction delay -------------------------------------------------
    #   A positive reaction delay is part of the M2 definition (author
    #   decision), so the search grid is > 0; 0.0 appears only in the
    #   sensitivity table as the "no delay" arm.
    for d in (0.15, 0.30, 0.45):
        a = evaluate(**base, m2_react_delay=d, **{k: v for k, v in chosen.items()
                                                 if k != "m2_react_delay"})
        a.update(param="m2_react_delay", value=d, **{k: chosen[k] for k in
                                                     chosen if k != "m2_react_delay"})
        rows.append(a)
        log(f"  delay={d:4.2f}  J={a['J']:7.3f} contact={a['animal_contact_fraction']:.3f} "
            f"lat={a['onset_latency']:.3f} worst={a['worst_min_clearance']:6.2f}")
    best = min([r for r in rows if r["param"] == "m2_react_delay"],
               key=lambda r: r["J"])
    chosen["m2_react_delay"] = best["value"]
    # -- 2. gain spread (sigma > 0 only: heterogeneity is part of the model) --
    for sg in (0.10, 0.20, 0.30):
        a = evaluate(**base, m2_react_delay=chosen["m2_react_delay"],
                     m2_gain_sigma=sg, m_v_max=chosen["m_v_max"],
                     m2_slow_gain=chosen["m2_slow_gain"])
        a.update(param="m2_gain_sigma", value=sg,
                 m2_react_delay=chosen["m2_react_delay"])
        rows.append(a)
        log(f"  sigma={sg:4.2f}  J={a['J']:7.3f} contact={a['animal_contact_fraction']:.3f} "
            f"worst={a['worst_min_clearance']:6.2f}")
    best = min(rows, key=lambda r: r["J"] if r["param"] == "m2_gain_sigma" else 1e9)
    chosen["m2_gain_sigma"] = best["value"]
    # -- 3. speed-target upper reference -----------------------------------
    for vm in (1.5, 1.8, 2.0):
        a = evaluate(**base, m2_react_delay=chosen["m2_react_delay"],
                     m2_gain_sigma=chosen["m2_gain_sigma"], m_v_max=vm,
                     m2_slow_gain=chosen["m2_slow_gain"])
        a.update(param="m_v_max", value=vm, m2_react_delay=chosen["m2_react_delay"],
                 m2_gain_sigma=chosen["m2_gain_sigma"])
        rows.append(a)
        log(f"  v_max={vm:4.2f}  J={a['J']:7.3f} contact={a['animal_contact_fraction']:.3f} "
            f"speed_mean={a['speed_mean']:.3f} max={a['speed_max']:.2f}")
    best = min([r for r in rows if r["param"] == "m_v_max"], key=lambda r: r["J"])
    chosen["m_v_max"] = best["value"]
    # -- 4. turn-demand slowdown -------------------------------------------
    for sg in (1.0, 0.5, 0.0):
        a = evaluate(**base, m2_react_delay=chosen["m2_react_delay"],
                     m2_gain_sigma=chosen["m2_gain_sigma"],
                     m_v_max=chosen["m_v_max"], m2_slow_gain=sg)
        a.update(param="m2_slow_gain", value=sg)
        rows.append(a)
        log(f"  slow={sg:4.2f}  J={a['J']:7.3f} contact={a['animal_contact_fraction']:.3f} "
            f"speed_min={a['speed_mean']:.3f} speed_max={a['speed_max']:.2f}")
    best = min([r for r in rows if r["param"] == "m2_slow_gain"],
               key=lambda r: r["J"])
    chosen["m2_slow_gain"] = best["value"]
    log(f"  -> chosen: {chosen}")
    return rows, chosen


def stage_sensitivity(log, m1, m2):
    log("=" * 78)
    log("Sensitivity around the chosen values (one factor at a time)")
    base = dict(m1_decay=m1["decay"], m1_gain=m1["gain"], m1_sense_range=6.0)
    if m1["decay"].startswith("exp"):
        base["m1_decay_len"] = 1.0 if m1["decay"] == "exp1" else 3.0
    rows = []
    # M1 gain +- one grid step, decay shapes, sensing range 3/6/9
    for g in (m1["gain"] / 2.0, m1["gain"], m1["gain"] * 1.5):
        a = evaluate(animal_model="M1", **{**base, "m1_gain": g})
        a.update(factor="m1_gain", value=g, model="M1")
        rows.append(a)
    for dec in ("linear", "quadratic", "exp1", "exp3"):
        d = dict(base)
        d["m1_decay"] = "exp" if dec.startswith("exp") else dec
        if dec.startswith("exp"):
            d["m1_decay_len"] = 1.0 if dec == "exp1" else 3.0
        a = evaluate(animal_model="M1", **d)
        a.update(factor="m1_decay", value=dec, model="M1")
        rows.append(a)
    for R in (3.0, 6.0, 9.0):
        a = evaluate(animal_model="M1", **{**base, "m1_sense_range": R})
        a.update(factor="m1_sense_range", value=R, model="M1")
        rows.append(a)
        log(f"  M1 R={R:3.1f}  J={a['J']:7.3f} contact={a['animal_contact_fraction']:.3f} "
            f"mean_clr={a['mean_min_clearance']:6.2f} worst={a['worst_min_clearance']:6.2f}")
    # M2: delay, sigma, speed mapping
    for d in (0.0, m2["m2_react_delay"], m2["m2_react_delay"] * 2):
        a = evaluate(animal_model="M2", **base, m2_gain_sigma=m2["m2_gain_sigma"],
                     m_v_max=m2["m_v_max"], m2_slow_gain=m2["m2_slow_gain"],
                     m2_react_delay=d)
        a.update(factor="m2_react_delay", value=d, model="M2")
        rows.append(a)
    for sg in (0.0, m2["m2_gain_sigma"], m2["m2_gain_sigma"] * 2):
        a = evaluate(animal_model="M2", **base, m2_react_delay=m2["m2_react_delay"],
                     m_v_max=m2["m_v_max"], m2_slow_gain=m2["m2_slow_gain"],
                     m2_gain_sigma=sg)
        a.update(factor="m2_gain_sigma", value=sg, model="M2")
        rows.append(a)
    for vm in (1.5, m2["m_v_max"]):
        a = evaluate(animal_model="M2", **base, m2_react_delay=m2["m2_react_delay"],
                     m2_gain_sigma=m2["m2_gain_sigma"], m_v_max=vm,
                     m2_slow_gain=m2["m2_slow_gain"])
        a.update(factor="m_v_max", value=vm, model="M2")
        rows.append(a)
    for r in rows:
        log(f"  {r['model']} {r['factor']:16s}={r['value']!s:>6}  J={r['J']:7.3f} "
            f"contact={r['animal_contact_fraction']:.3f}")
    return rows


# ---------------------------------------------------------------------------
# SELECTION RULE (fixed after the search, applied to the CSVs so the choice is
# reproducible without re-running anything)
# ---------------------------------------------------------------------------
#   1. M1: best J, then the parsimony tie-break above.
#   2. M2: if the objective is flat over a parameter (every searched value
#      within TIE_TOL of the best), the outcome does not identify it and the
#      value is set by the design rule below; otherwise the objective decides.
#   3. One documented design override: m2_slow_gain.  The author's decision
#      requires the admissible lower speed (0) to be *reachable by the model*;
#      with m2_slow_gain = 1.0 the speed target never falls below the base
#      value, so the mechanism would only exist on paper.  0.5 halves the target
#      at a 90-degree turn demand and costs 0.55 J (path efficiency) -- the cost
#      is reported instead of hidden.
DESIGN_M2 = {
    "m2_react_delay": (0.30, "flat objective -> middle of the searched "
                             "physiological band 0.15/0.30/0.45 s"),
    "m2_gain_sigma": (0.20, "flat objective -> middle of the searched spread "
                            "0.10/0.20/0.30 (20% coefficient of variation)"),
    "m_v_max": (2.0, "flat objective -> the author's exploratory upper "
                     "reference (1.8-2.0 m/s)"),
    "m2_slow_gain": (0.5, "design override: makes the admissible lower speed "
                          "reachable (J cost 0.55, reported)"),
}


def stage_select():
    """Re-derive the shipped values from the sweep CSVs (no new runs)."""
    def read(p):
        with open(p, newline="", encoding="utf-8-sig") as f:
            return list(csv.DictReader(f))

    rows1 = read(OUT / "calibration_m1.csv")
    for r in rows1:
        r["J"] = float(r["J"])
        r["gain"] = float(r["gain"])
    best, bestJ, nties = select_best(rows1)
    rows2 = read(OUT / "calibration_m2.csv")

    def flat(param):
        vals = [float(r["J"]) for r in rows2 if r["param"] == param]
        return vals, (max(vals) - min(vals)) <= TIE_TOL and len(vals) > 1

    chosen = {}
    notes = {}
    for param, (val, why) in DESIGN_M2.items():
        vals, isflat = flat(param)
        if isflat or param == "m2_slow_gain":
            chosen[param] = val
            notes[param] = why + (f" (J range {min(vals):.3f}-{max(vals):.3f})"
                                  if vals else "")
        else:
            cand = [r for r in rows2 if r["param"] == param]
            b = min(cand, key=lambda r: float(r["J"]))
            chosen[param] = float(b["value"])
            notes[param] = (f"objective decided (J range "
                            f"{min(vals):.3f}-{max(vals):.3f})")
    summary = dict(
        M1=dict(decay=best["decay"], gain=best["gain"], sense_range=6.0,
                J=best["J"], best_J=bestJ, n_within_tie=nties,
                rule="best J, then parsimony (linear<quadratic<exp, then "
                     "smaller gain) within TIE_TOL"),
        M2=dict(chosen), M2_notes=notes,
        objective="J = 100*contact + 10*max(0,-worst_clearance) + "
                  "5*max(0,0.5-path_eff) + 2*max(0,latency-2) + "
                  "2*max(0,0.3-latency)",
        budget=json.loads((OUT / "calibration_summary.json").read_text(
            encoding="utf-8"))["budget"]
        if (OUT / "calibration_summary.json").exists() else None,
    )
    (OUT / "calibration_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


# ---------------------------------------------------------------------------
HEAD = ["model", "decay", "gain", "sense_range", "param", "value", "factor",
        "m2_react_delay", "m2_gain_sigma", "m_v_max", "m2_slow_gain",
        "J", "animal_contact_fraction", "mean_min_clearance",
        "worst_min_clearance", "path_efficiency_mean", "inside_frac",
        "onset_latency", "speed_mean", "speed_max", "flip_rate_mean",
        "omega_max", "n_contact_events_so"]


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    keys = [k for k in HEAD if k in keys] + [k for k in keys if k not in HEAD]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="all",
                    choices=["m1", "m2", "sens", "select", "all"])
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.stage == "select":
        stage_select()
        return
    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    t0 = time.time()
    m1rows, m1best = stage_m1(log) if args.stage in ("m1", "all") else (None, None)
    if args.stage == "all":
        write_csv(OUT / "calibration_m1.csv", m1rows)
        m2rows, m2best = stage_m2(log, m1best)
        write_csv(OUT / "calibration_m2.csv", m2rows)
        srows = stage_sensitivity(log, m1best, m2best)
        write_csv(OUT / "calibration_sensitivity.csv", srows)
        summary = dict(
            M1=dict(decay=m1best["decay"], gain=m1best["gain"],
                    sense_range=6.0, J=m1best["J"]),
            M2=dict(m2best),
            budget=dict(runs=BUDGET["runs"], core_seconds=BUDGET["seconds"],
                        wall_seconds=time.time() - t0,
                        seeds=CAL_SEEDS[:N_CAL]),
        )
        (OUT / "calibration_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        log(f"budget: {BUDGET['runs']} corridor runs, "
            f"{BUDGET['seconds']:.1f} core-s, {time.time() - t0:.1f} s wall")
    (OUT / "calibration.log").write_text("\n".join(lines) + "\n",
                                         encoding="utf-8")


if __name__ == "__main__":
    main()
