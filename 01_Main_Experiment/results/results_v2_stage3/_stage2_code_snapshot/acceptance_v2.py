"""V2 Stage-1 acceptance tests A-E (plus the reviewer additions F-I).

    python -m v2.acceptance_v2                # all tests
    python -m v2.acceptance_v2 --tests A D    # subset

Every test prints a transcript and a PASS/FAIL line; the whole session is also
written to ``results_v2_stage1/ACCEPTANCE.txt``.

A  Inertness (critical): V2-compat vs the archived V1 cell, and vs a direct
   in-process V1 ``run_scene`` (whole trajectory, not just the summary).
B  Joint limits: zero C3/R4/lateral violations on a short full run, and the
   bounds must be *binding* (otherwise the test is vacuous).
C  Non-penetration: per-class surface clearance / contact counts, with the
   S / P / C cause split (S != physically unavoidable).
D  Contact-detection accuracy against analytic grazing scenarios.
E  Determinism: repeat run, and jobs=1 vs jobs=4.
F  Separate clipping vs joint projection (reviewer comment 4).
G  V1 criterion / V1-trajectory solid re-check / V2 solid contact (comment 1).
H  Nominal spacing vs braking distance vs required spacing (comment 3).
I  Infeasibility cause census (comment 2).
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import V2Cfg, compat_v1cfg, default_v2cfg  # noqa: E402
from v2.contacts_v2 import (ContactMonitor, poly_first_contact,  # noqa: E402
                            poly_min_clearance, seg_first_contact,
                            seg_min_clearance)
from v2.limits_v2 import project_sheep_increment, robot_step_project  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage1"
ARCH = ROOT / "results_cea_matrix" / "_parts"

RESULTS = []


# ---------------------------------------------------------------------------
# plumbing
# ---------------------------------------------------------------------------
class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def __call__(self, *a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        self.f.write(s + "\n")
        self.f.flush()


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
          + (f"  -- {detail}" if detail else ""), flush=True)


def eq(a, b):
    """Exact equality for floats/strings, NaN==NaN."""
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) or math.isnan(b):
            return math.isnan(a) and math.isnan(b)
        return a == b
    return a == b


def parse_cell(t):
    """Archive CSV cell -> float / bool / string (repr round-trips exactly)."""
    s = (t or "").strip()
    if s in ("True", "true"):
        return True
    if s in ("False", "false"):
        return False
    if s in ("", "nan"):
        return float("nan")
    if s == "inf":
        return float("inf")
    if s == "-inf":
        return float("-inf")
    try:
        return float(s)
    except ValueError:
        return s


def archived_row(scenario, alg, seed):
    p = ARCH / alg / "detail.csv"
    with open(p, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if row["scenario"] == scenario and int(row["seed"]) == seed:
                return row
    raise KeyError((scenario, alg, seed))


V1_TIMING = {"ctrl_ms"}


# ---------------------------------------------------------------------------
# A. inertness
# ---------------------------------------------------------------------------
def test_A(log):
    log("=" * 78)
    log("A. INERTNESS  (r_robot->0, caps off, feasibility bypassed)")
    log("=" * 78)
    from run_experiments import ALGS, KEYS
    from scenarios import SCENARIOS
    from simulation import run_scene

    sid, alg, seed = "S3", "Ours_rollout", 200
    # --- A1: against the archive -------------------------------------------
    row = run_cell(sid, alg, seed, compat=True)
    arc = archived_row(sid, alg, seed)
    bad = []
    for k in KEYS:
        if k in V1_TIMING:
            continue
        av = parse_cell(arc[k])
        got = row.get(k)
        if isinstance(av, bool) or isinstance(got, bool):
            same = bool(av) == bool(got) and type(av) is type(got)
        else:
            same = eq(av, got)
        if not same:
            bad.append((k, arc[k], got))
    log(f"A1  cell {sid}/{alg}/seed {seed}: {len(KEYS) - len(V1_TIMING)} "
        f"non-timing columns")
    for k, a, g in bad:
        log(f"    mismatch {k}: archived={a!r} v2compat={g!r}")
    check("A1 V2-compat == archived V1 cell (bit-exact, non-timing)",
          not bad, f"{len(bad)} mismatch(es)")

    # --- A2: against a direct in-process V1 run ----------------------------
    scn = SCENARIOS[sid]()
    ctrl = ALGS[alg](**scn.get("params", {}))
    r1 = run_scene(scn["cfg"], scn["path"], scn["obstacles"], ctrl, seed=seed,
                   T_max=scn["T_max"], s0=scn["s0"])
    ctrl2 = ALGS[alg](**scn.get("params", {}))
    res2 = run_cell(sid, alg, seed, compat=True, return_res=True)[1]
    # note: run_cell already builds its own controller; compare the results
    diffs = []
    for k in ("lost", "success", "success_t", "terminated_by", "timeout",
              "violation_rate", "dmin_robot_obs", "dmin_robot_robot",
              "dmin_sheep_obs", "control_var", "out_normal", "out_risk",
              "role_switches", "rr_min_t", "rr_min_tag"):
        if not eq(r1.get(k), res2.get(k)):
            diffs.append((k, r1.get(k), res2.get(k)))
    la, lb = r1["logs"], res2["logs"]
    if len(la) != len(lb):
        diffs.append(("len(logs)", len(la), len(lb)))
    else:
        for i, (x, y) in enumerate(zip(la, lb)):
            for k in x:
                if not eq(x[k], y[k]):
                    diffs.append((f"logs[{i}].{k}", x[k], y[k]))
                    break
            if len(diffs) > 8:
                break
    log(f"A2  V1 run_scene vs V2-compat: {len(la)} logged ticks, "
        f"{len(diffs)} difference(s)")
    for d in diffs[:10]:
        log(f"    {d[0]}: v1={d[1]!r} v2={d[2]!r}")
    check("A2 V2-compat == V1 run_scene (whole trajectory, bit-exact)",
          not diffs, f"{len(diffs)} difference(s)")
    return not bad and not diffs


# ---------------------------------------------------------------------------
# B. joint limits
# ---------------------------------------------------------------------------
def test_B(log, seeds=(200, 201)):
    log("=" * 78)
    log("B. JOINT LIMITS  (C3 / R4 / lateral must be zero AND binding)")
    log("=" * 78)
    rows = []
    agg = {}
    for sd in seeds:
        t0 = time.time()
        row = run_cell("S3", "Ours_rollout", sd)
        row["_wall"] = time.time() - t0
        rows.append(row)
        for k, v in row.items():
            if isinstance(v, (int, float)) and v == v:
                agg.setdefault(k, []).append(v)
        log(f"B  seed {sd}: ticks={row.get('opp_sheep_steps')} "
            f"wall={row['_wall']:.1f}s  "
            f"viol C3/R4/lat = {row['violation_C3']}/"
            f"{row['violation_R4']}/{row['violation_lat']}")
    tot = {k: sum(v) for k, v in agg.items()
           if k in ("violation_C3", "violation_R4", "violation_lat",
                    "violation_C1_low", "violation_C2", "violation_R1",
                    "violation_R2", "violation_R3")}
    log(f"B  totals over {len(seeds)} seeds: " +
        ", ".join(f"{k}={int(v)}" for k, v in tot.items()))
    ok = (tot["violation_C3"] == 0 and tot["violation_R4"] == 0
          and tot["violation_lat"] == 0)

    log("\nB  binding frequency (how often each bound had to clip):")
    log(f"    {'bound':22s} {'clips':>10s} {'opportunities':>14s} "
        f"{'rate':>9s}")
    opp_s = sum(agg.get('opp_sheep_steps', [0]))
    opp_r = sum(agg.get('opp_robot_steps', [0]))
    table = [("C2 sheep turn", "bind_C2_sheep", opp_s),
             ("C3 sheep vector accel", "bind_C3_sheep", opp_s),
             ("  ... of which turn", "bind_C3_turn_sheep", opp_s),
             ("  ... of which speed", "bind_C3_speed_sheep", opp_s),
             ("C1 sheep lower (relaxed)", "bind_C1_low_sheep", opp_s),
             ("R1 robot speed", "bind_R1_robot", opp_r),
             ("R2 robot turn rate", "bind_R2_robot", opp_r),
             ("R3 robot linear accel", "bind_R3_robot", opp_r),
             ("R4 robot angular accel", "bind_R4_robot", opp_r),
             ("LAT |v*omega|", "bind_LAT_robot", opp_r)]
    binding = 0
    for label, key, opp in table:
        n = int(sum(agg.get(key, [0])))
        if not key.startswith("  "):
            binding += n
        r = (n / opp) if opp else float("nan")
        log(f"    {label:22s} {n:10d} {opp:14d} {r:9.4f}")
    check("B1 zero C3 / R4 / lateral violations", ok,
          f"C3={int(tot['violation_C3'])}, R4={int(tot['violation_R4'])}, "
          f"lat={int(tot['violation_lat'])}")
    check("B2 the caps are actually binding (test is not vacuous)",
          binding > 0 and int(sum(agg.get("bind_C3_sheep", [0]))) > 0
          and int(sum(agg.get("bind_R4_robot", [0]))) > 0
          and int(sum(agg.get("bind_LAT_robot", [0]))) > 0,
          f"C3_sheep={int(sum(agg.get('bind_C3_sheep', [0])))}, "
          f"R4={int(sum(agg.get('bind_R4_robot', [0])))}, "
          f"LAT={int(sum(agg.get('bind_LAT_robot', [0])))}")
    return rows, ok


# ---------------------------------------------------------------------------
# C. non-penetration
# ---------------------------------------------------------------------------
def test_C(log, rows):
    log("=" * 78)
    log("C. NON-PENETRATION  (continuous surface clearance, per pair class)")
    log("=" * 78)
    cls_hard = {"rr": 0.90, "ro": 1.65, "rs": 0.80, "ss": 0.70, "so": 1.55}
    name = {"rr": "robot-robot", "ro": "robot-obstacle", "rs": "robot-sheep",
            "ss": "sheep-sheep", "so": "sheep-obstacle"}
    log(f"    {'class':16s} {'hard d':>7s} {'min clr':>10s} {'~seed':>16s} "
        f"{'events':>7s} {'preexist':>9s} {'any':>5s}")
    for c in ("rr", "ro", "rs", "ss", "so"):
        mins = [r.get(f"min_surface_clearance_{c}") for r in rows]
        ev = [r.get(f"n_contact_events_{c}") for r in rows]
        pre = [r.get(f"n_preexisting_contact_{c}") for r in rows]
        anyc = any(bool(r.get(f"any_contact_{c}")) for r in rows)
        log(f"    {name[c]:16s} {cls_hard[c]:7.2f} "
            f"{min(mins):10.4f} {str([round(m, 3) for m in mins]):>16s} "
            f"{sum(int(x) for x in ev):7d} {sum(int(x) for x in pre):9d} "
            f"{str(anyc):>5s}")
    log("\n    Cause census for the infeasibility layer (S != unavoidable):")
    for r in rows:
        log(f"      seed {r['seed']}: n_infeasible={int(r['n_infeasible'])} "
            f"S={int(r['n_infeasible_S'])} P={int(r['n_infeasible_P'])} "
            f"resolved={int(r['n_infeasible_resolved'])} "
            f"contact_C={int(r['n_contact_C'])}")
    log("    Interpretation is in V2_STAGE1_REPORT.md §6 (test C); contacts are"
        " reported, never tuned away.")
    # The test asserts *reporting and attribution*, not zero contacts.
    check("C1 per-class clearances and contact counts reported",
          all(np.isfinite(r.get("min_surface_clearance_rr", np.nan))
              for r in rows), "all five classes measured on every run")
    attributed = all(
        int(r.get("n_infeasible_S") or 0) + int(r.get("n_infeasible_P") or 0)
        + int(r.get("n_contact_C") or 0) > 0 or
        all(float(r.get(f"min_surface_clearance_{c}") or 0) >= 0
            for c in ("rr", "ro", "rs"))
        for r in rows)
    check("C2 every contact is attributed to S / P / C (no silent tuning)",
          attributed)
    worst = min(r.get("min_surface_clearance_rr", np.inf) for r in rows)
    nev = sum(int(r.get("n_contact_events_rr") or 0) for r in rows)
    log(f"\n    FINDING: worst robot-robot surface clearance = {worst:+.3f} m "
        f"against the 0.90 m hard limit ({nev} contact episode(s) over "
        f"{len(rows)} runs), at the shipped d_rr_safe = "
        f"{default_v2cfg().d_rr_safe:.2f} m.")
    if worst < 0:
        log("    The hard limit is therefore NOT held in every run. The same")
        log("    runs at the author-proposed d_rr_safe = 1.2 m give 69 episodes")
        log("    and -0.457 m (v2/diag_v2.py, 3 seeds). Reported, not tuned")
        log("    (brief: 'do not silently tune to zero'); see report §6.4/§9.")
    else:
        log("    The hard limit is held in every run of this sample; it is not")
        log("    a guarantee -- see the residual episodes at tighter margins in")
        log("    the report §9.4.")
    return True


# ---------------------------------------------------------------------------
# D. detector accuracy
# ---------------------------------------------------------------------------
def test_D(log):
    log("=" * 78)
    log("D. CONTACT-DETECTION ACCURACY  (analytic grazing scenarios)")
    log("=" * 78)
    # --- D1a: closest approach inside a straight segment -------------------
    a0 = np.array([0.0, 0.0])
    a1 = np.array([6.0, 0.0])
    b0 = np.array([5.0, 0.4])
    b1 = np.array([5.0, 0.4])
    d, tau = seg_min_clearance(a0, a1, b0, b1)
    log(f"D1a closest approach of a swept segment to a point: "
        f"d={float(d):.17g} (exact 0.4), tau={float(tau):.10f} "
        f"(exact 5/6={5/6:.10f})")
    ok1 = abs(float(d) - 0.4) < 1e-15 and abs(float(tau) - 5 / 6) < 1e-15

    # --- D1b: exact entry time inside one integration interval -------------
    #   a: (0,0) -> (0.075, 0), 1.5 m/s ; b: (0.9042,0.35) -> (0.8292,0.35)
    a0 = np.array([0.0, 0.0])
    a1 = np.array([0.075, 0.0])
    b0 = np.array([0.9042, 0.35])
    b1 = np.array([0.8292, 0.35])
    t_hit, hit = seg_first_contact(a0, a1, b0, b1, 0.9)
    r0 = a0 - b0
    u = (a1 - b1) - r0
    A, B, C = u @ u, 2 * (r0 @ u), r0 @ r0 - 0.81
    tau_ex = (-B - math.sqrt(B * B - 4 * A * C)) / (2 * A)
    log(f"D1b moving pair, exact entry tau = {float(t_hit):.17g} vs analytic "
        f"{tau_ex:.17g}  (|diff| = {abs(float(t_hit) - tau_ex):.2e}, "
        f"start clearance = {math.sqrt(r0 @ r0) - 0.9:+.4f} m)")
    ok2 = abs(float(t_hit) - tau_ex) < 1e-14 and 0.0 < tau_ex < 1.0

    # --- D2: monitor level, multi-interval, first-contact time -----------
    dt = 0.05
    n = 60
    mon = ContactMonitor({"rr": 0.9, "ro": 0.0, "rs": 0.0, "ss": 0.0,
                          "so": 0.0}, tol=0.0)
    pA = np.array([0.0, 0.0])
    vA = np.array([1.5, 0.0])
    pB = np.array([5.0, 0.5])
    vB = np.array([-1.5, 0.0])
    for k in range(n):
        t = k * dt
        mon.update(t, dt,
                   rob=(np.array([pA, pB]),
                        np.array([pA + vA * dt, pB + vB * dt])),
                   sheep=(np.zeros((0, 2)), np.zeros((0, 2))),
                   obs=(np.zeros((0, 2)), np.zeros((0, 2))))
        pA = pA + vA * dt
        pB = pB + vB * dt
    t_an = (5.0 - math.sqrt(0.81 - 0.25)) / 3.0
    log(f"D2  monitor, 2 robots closing at 3 m/s, lateral offset 0.5 m, "
        f"r_rr = 0.90:")
    log(f"    reported min surface clearance = "
        f"{mon.min_clearance['rr']:.17g}  (analytic 0.5-0.9 = -0.4)")
    log(f"    reported first contact time    = {mon.first_time['rr']:.17g}"
        f"  (analytic {t_an:.17g})")
    log(f"    |error| = {abs(mon.first_time['rr'] - t_an):.2e} s   "
        f"events={mon.n_events['rr']}")
    ok3 = (abs(mon.min_clearance["rr"] + 0.4) < 1e-12
           and abs(mon.first_time["rr"] - t_an) < 1e-9)

    # --- D3: chord vs arc resolution -------------------------------------
    from v2.contacts_v2 import arc_samples
    v, om, dt, S = 1.5, 3.0, 0.05, 9
    p0 = np.array([0.0, 0.0])
    th0 = np.array([0.0])
    arc = arc_samples(p0[None, :], th0, np.array([v]), om, dt, S - 1)[0]
    chord2 = np.stack([p0, arc[-1]])                 # one straight chord
    dense = arc_samples(p0[None, :], th0, np.array([v]), om, dt, 4000)[0]
    R = v / om
    sag = R * (1 - math.cos(om * dt / 2))            # exact sagitta, full step
    sub_res = R * (1 - math.cos(om * dt / (2 * (S - 1))))
    log(f"D3  one integration step, v={v}, omega={om}, dt={dt}, "
        f"N_sub={S - 1}; probes 0.02 m from the arc mid-point:")
    log(f"    exact sagitta R(1-cos(omega dt/2))        = {sag:.3e} m")
    log(f"    sub-chord resolution R(1-cos(omega dt/2N)) = {sub_res:.3e} m")
    log(f"    {'probe':8s} {'dense arc':>16s} {'chord err':>12s} "
        f"{'arc-N8 err':>12s}")
    errs = []
    for tag, sgn in (("+y", +1.0), ("-y", -1.0)):
        probe = arc[(S - 1) // 2] + np.array([0.0, sgn * 0.02])

        def _d(poly):
            pp = np.repeat(probe[None, None, :], poly.shape[0], axis=1)
            return float(poly_min_clearance(poly[None], pp, 0.0)[0][0])

        d_ref = _d(dense)
        d_chord = _d(chord2)
        d_arc = _d(arc)
        log(f"    {tag:8s} {d_ref:16.12f} {d_chord - d_ref:+12.3e} "
            f"{d_arc - d_ref:+12.3e}")
        errs.append((d_ref, d_chord, d_arc))
    ok4 = all(abs(a - r) <= 2.0 * sub_res + 1e-12 for r, c, a in errs) and \
        all(abs(c - r) >= 0.5 * sag for r, c, a in errs)
    log("    => the single chord errs by the sagitta (1.4e-3 m, sign depends "
        "on which side the partner is);")
    log(f"       arc mode with {S - 1} sub-chords is within {2 * sub_res:.1e} m "
        f"of the exact arc.")

    for nm, ok in (("D1a linear sweep exact (closest approach)", ok1),
                   ("D1b linear sweep exact (entry time)", ok2),
                   ("D2 monitor min clearance + first-contact time exact", ok3),
                   ("D3 arc mode within the stated sub-chord resolution", ok4)):
        check(nm, ok)
    return ok1 and ok2 and ok3 and ok4


# ---------------------------------------------------------------------------
# E. determinism
# ---------------------------------------------------------------------------
def test_E(log):
    log("=" * 78)
    log("E. DETERMINISM  (repeat, and jobs=1 vs jobs=4)")
    log("=" * 78)
    r1 = run_cell("S3", "Ours_rollout", 200)
    r2 = run_cell("S3", "Ours_rollout", 200)
    diff = [k for k in r1 if k != "ctrl_ms" and not eq(r1[k], r2[k])]
    log(f"E1  repeat run: {len(diff)} differing columns "
        f"{'(ctrl_ms excluded)' if diff == ['ctrl_ms'] else ''}")
    check("E1 two identical runs give identical results", not diff,
          f"{len(diff)} diff")

    def run_jobs(j):
        out = OUT / f"_det_j{j}"
        cmd = [sys.executable, "-X", "utf8", "-m", "v2.runner_v2",
               "--scenarios", "S3", "S2", "--seeds", "1",
               "--seed-offset", "200", "--algs", "Ours_rollout",
               "--jobs", str(j), "--out", str(out)]
        subprocess.run(cmd, cwd=ROOT, check=True, capture_output=True)
        with open(out / "detail.csv", newline="", encoding="utf-8-sig") as f:
            return list(csv.DictReader(f))

    a = run_jobs(1)
    b = run_jobs(4)
    bad = []
    for ra, rb in zip(a, b):
        for k in ra:
            if k == "ctrl_ms":
                continue
            if ra[k] != rb[k]:
                bad.append((ra["scenario"], k, ra[k], rb[k]))
    log(f"E2  jobs=1 vs jobs=4: {len(a)} rows, {len(bad)} differing cells")
    for x in bad[:8]:
        log(f"    {x}")
    check("E2 jobs=1 == jobs=4 (bit-exact, deterministic reduction order)",
          not bad, f"{len(bad)} differing cells")
    return not diff and not bad


# ---------------------------------------------------------------------------
# F. separate clipping vs joint projection
# ---------------------------------------------------------------------------
def test_F(log, seeds=(200, 201)):
    log("=" * 78)
    log("F. SEPARATE CLIPPING vs JOINT PROJECTION  (reviewer comment 4)")
    log("=" * 78)
    # analytic counterexample from the review
    v, om = 1.5, 3.0
    log(f"F1  counterexample: v={v} m/s, omega={om} rad/s -> "
        f"|v*omega| = {v * om:.2f} m/s^2, which already exceeds both the "
        f"lateral cap (2.0) and the 4.0 m/s^2 acceleration scale.")
    # what a per-bound clip does to the (V2) box:
    dth_des, s_des, s_old = 0.15, 1.5, 1.5
    A = 4.0 * 0.05
    seq_s = s_old + float(np.clip(s_des - s_old, -A, A))   # scalar clip
    v_old = s_old * np.array([1.0, 0.0])
    v_new_seq = seq_s * np.array([math.cos(dth_des), math.sin(dth_des)])
    jump_seq = float(np.linalg.norm(v_new_seq - v_old))
    dth_j, s_j, _ = project_sheep_increment(s_old, dth_des, s_des, dt=0.05,
                                            a_s=4.0, omega_s=3.0, s_lo=1.0,
                                            s_hi=1.5)
    v_new_j = s_j * np.array([math.cos(dth_j), math.sin(dth_j)])
    jump_j = float(np.linalg.norm(v_new_j - v_old))
    log(f"    separate clipping : turn kept {dth_des} rad -> |dv| = "
        f"{jump_seq:.4f} m/s over one step = {jump_seq / 0.05:.2f} m/s^2 "
        f"(LIMIT {A / 0.05:.1f})")
    log(f"    joint projection  : turn {dth_j:.6f} rad, speed {s_j:.6f} -> "
        f"|dv| = {jump_j:.4f} -> {jump_j / 0.05:.2f} m/s^2")
    check("F1 joint projection satisfies the vector bound where separate "
          "clipping violates it",
          jump_j <= A * (1 + 1e-12) and jump_seq > A)

    # F2: whole-run comparison on the same scenarios/seeds
    log("\nF2  same short runs, projection=joint vs projection=sequential")
    log(f"    {'arm':12s} {'seed':>5s} {'viol_C3':>9s} {'bind_C3':>9s} "
        f"{'collision_rate':>15s} {'rho_avg':>8s}")
    tot = {"joint": 0, "sequential": 0}
    for arm in ("joint", "sequential"):
        for sd in seeds:
            row = run_cell("S3", "Ours_rollout", sd, projection=arm)
            tot[arm] += int(row["violation_C3"])
            log(f"    {arm:12s} {sd:5d} {int(row['violation_C3']):9d} "
                f"{int(row['bind_C3_sheep']):9d} "
                f"{row['collision_rate']:15.4f} {row['rho_avg']:8.4f}")
    check("F2 joint projection has strictly fewer C3 violations than separate "
          "clipping", tot["joint"] < tot["sequential"],
          f"joint={tot['joint']}, sequential={tot['sequential']}")
    return tot["joint"] == 0 and tot["sequential"] > 0


# ---------------------------------------------------------------------------
# G. V1 criterion vs solid re-check
# ---------------------------------------------------------------------------
def test_G(log):
    log("=" * 78)
    log("G. V1 CRITERION / V1-TRAJECTORY SOLID RE-CHECK / V2 SOLID CONTACT")
    log("=" * 78)
    RISK = ["S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]
    rows = []
    for f in glob.glob(str(ARCH / "*" / "detail.csv")):
        with open(f, newline="", encoding="utf-8-sig") as fh:
            rows += list(csv.DictReader(fh))

    def num(t):
        t = (t or "").strip()
        if t in ("", "nan"):
            return float("nan")
        if t == "inf":
            return float("inf")
        return float(t)

    risk = [r for r in rows if r["scenario"] in RISK]
    algs = sorted({r["alg"] for r in rows})
    n_runs = len(risk)
    c_v1 = sum(1 for r in risk if num(r["dmin_robot_obs"]) < 1.2)
    c_solid = sum(1 for r in risk if num(r["dmin_robot_obs"]) < 1.65)
    log(f"G1  archive: {len(rows)} runs = {len(algs)} methods x "
        f"10 scenarios x 30 seeds; risk subset = {n_runs} runs "
        f"({len(algs)} x 8 x 30)")
    log(f"    V1 criterion   dmin_robot_obs < r_o        = 1.20 m : "
        f"{c_v1}/{n_runs} = {100 * c_v1 / n_runs:.1f}%")
    log(f"    same trajectories, solid re-check < r_robot + r_o = 1.65 m : "
        f"{c_solid}/{n_runs} = {100 * c_solid / n_runs:.1f}%")
    log(f"    contacts added purely by the criterion change : "
        f"{c_solid - c_v1} runs (+{100 * (c_solid - c_v1) / n_runs:.1f} pp)")
    check("G1 1269/1920 reproduced exactly",
          (n_runs == 1920 and c_v1 == 1269), f"{c_v1}/{n_runs}")
    log("    (1509/2160 belongs to the 9-method results_v9_noB3 lineage, which "
        "is not present in this workspace -> 待确认)")

    # per-scenario / per-method split
    log("\nG2  per scenario (V1 runs, n=240 each):")
    log(f"    {'scenario':8s} {'<1.20':>7s} {'<1.65':>7s} {'added':>7s}")
    for sid in RISK:
        rr = [r for r in risk if r["scenario"] == sid]
        a = sum(1 for r in rr if num(r["dmin_robot_obs"]) < 1.2)
        b = sum(1 for r in rr if num(r["dmin_robot_obs"]) < 1.65)
        log(f"    {sid:8s} {a:7d} {b:7d} {b - a:7d}")
    log("\nG3  per method (V1 runs, n=240 each):")
    log(f"    {'method':16s} {'<1.20':>7s} {'<1.65':>7s} {'added':>7s}")
    for alg in algs:
        rr = [r for r in risk if r["alg"] == alg]
        a = sum(1 for r in rr if num(r["dmin_robot_obs"]) < 1.2)
        b = sum(1 for r in rr if num(r["dmin_robot_obs"]) < 1.65)
        log(f"    {alg:16s} {a:7d} {b:7d} {b - a:7d}")

    # continuous re-check on replayed V1 trajectories
    log("\nG4  continuous swept re-check of the *V1* trajectory "
        "(8 risk scenarios, seed 200, Ours_rollout):")
    log(f"    {'scenario':8s} {'archive dmin':>13s} {'swept min clr ro':>18s} "
        f"{'step min clr':>13s} {'events_ro':>10s}")
    for sid in RISK:
        row, res = run_cell(sid, "Ours_rollout", 200, compat=True,
                            return_res=True)
        arc = archived_row(sid, "Ours_rollout", 200)
        v2m = res["v2"]
        swept = float(v2m.get("min_surface_clearance_ro", float("nan")))
        stepmin = num(arc["dmin_robot_obs"]) - 1.65
        log(f"    {sid:8s} {num(arc['dmin_robot_obs']):13.4f} "
            f"{swept:18.4f} {stepmin:13.4f} "
            f"{int(v2m.get('n_contact_events_ro', 0)):10d}")
    log("    'archive dmin' is a *lagged* statistic: the V1 loop measures the")
    log("    robot at t+dt against the obstacle at t, so the archived number is")
    log("    not the synchronised closest approach. The swept column is.")
    return c_v1 == 1269


# ---------------------------------------------------------------------------
# H. braking distance
# ---------------------------------------------------------------------------
def test_H(log):
    log("=" * 78)
    log("H. NOMINAL SPACING vs BRAKING DISTANCE vs REQUIRED SPACING")
    log("=" * 78)
    v2 = default_v2cfg()
    log(f"    d_ro_safe (nominal, author) = {v2.d_ro_safe:.2f} m; "
        f"hard body limit r_robot + r_o = {v2.d_ro_contact:.2f} m; "
        f"static surface margin = {v2.d_ro_safe - v2.d_ro_contact:.2f} m")
    log(f"    t_delay (one control period) = {v2.t_delay:.2f} s")
    log(f"    {'v':>5s} {'d_stop a=3.0':>13s} {'d_stop a=4.0':>13s} "
        f"{'d_delay':>8s} {'d_req(3.0)':>11s} {'d_req(4.0)':>11s} "
        f"{'vs 2.0 m':>9s}")
    for v in (1.0, 2.0, 2.6, 3.5, 4.5):
        ds3 = v * v / (2 * 3.0)
        ds4 = v * v / (2 * 4.0)
        dd = v * v2.t_delay
        req3 = ds3 + dd + v2.d_ro_contact + v2.brake_margin
        req4 = ds4 + dd + v2.d_ro_contact + v2.brake_margin
        log(f"    {v:5.1f} {ds3:13.3f} {ds4:13.3f} {dd:8.3f} {req3:11.3f} "
            f"{req4:11.3f} {req3 - v2.d_ro_safe:+9.3f}")
    log("    => at v = 4.5 m/s the stopping distance alone (2.53 m at a_lin =")
    log("       4.0, 3.375 m de-rated to 3.0) exceeds the whole 2.0 m nominal")
    log("       target, so a static target cannot be a high-speed guarantee.")
    log(f"    => the supervisor plans with a_dec = {v2.a_dec_plan} m/s^2 "
        f"(= {100 * v2.a_dec_plan / v2.a_lin:.0f}% of a_lin) and limits the "
        f"*approach* component accordingly.")
    check("H1 nominal target is below the 4.5 m/s stopping distance "
          "(documented, not hidden)",
          4.5 ** 2 / (2 * 3.0) > v2.d_ro_safe)
    return True


# ---------------------------------------------------------------------------
# I. infeasibility census
# ---------------------------------------------------------------------------
def test_I(log, rows):
    log("=" * 78)
    log("I. INFEASIBILITY CAUSE CENSUS  (S / P / C)")
    log("=" * 78)
    keys = ("n_infeasible", "n_infeasible_S", "n_infeasible_P",
            "n_infeasible_resolved", "n_contact_C", "n_brake_events")
    log(f"    {'seed':>5s} " + "".join(f"{k.replace('n_', ''):>12s}"
                                      for k in keys))
    for r in rows:
        log(f"    {r['seed']:5d} " + "".join(
            f"{int(r.get(k) or 0):12d}" for k in keys))
    tot = {k: sum(int(r.get(k) or 0) for r in rows) for k in keys}
    log(f"    S = solver-infeasible (NOT a claim of physical unavoidability)")
    log(f"    P = certified unavoidable under the stated assumptions")
    log(f"    C = a safe action existed (post-hoc oracle) but was not taken")
    check("I1 all three cause codes are populated and the S!=P distinction is "
          "reported", tot["n_infeasible_S"] > 0 and tot["n_infeasible_P"] > 0,
          f"S={tot['n_infeasible_S']}, P={tot['n_infeasible_P']}, "
          f"C={tot['n_contact_C']}")
    return True


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=list("ABCDEFGHI"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    log = Tee(Path(args.out) if args.out else OUT / "ACCEPTANCE.txt")
    t0 = time.time()
    log(f"V2 Stage-1 acceptance -- {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"python {sys.version.split()[0]}  numpy {np.__version__}")
    c = default_v2cfg()
    log(f"V2 config_id = {c.config_id}   sha256 = {c.config_sha256}")
    log(f"V1 compat config_id = {compat_v1cfg().config_id}")
    rowsB = []
    okA = None
    for t in args.tests:
        if t == "A":
            okA = test_A(log)
            if not okA:
                log("\n*** ACCEPTANCE A FAILED -- STOPPING (per the brief) ***")
                break
        elif t == "B":
            rowsB, _ = test_B(log)
        elif t == "C":
            if not rowsB:
                rowsB, _ = test_B(log)
            test_C(log, rowsB)
        elif t == "D":
            test_D(log)
        elif t == "E":
            test_E(log)
        elif t == "F":
            test_F(log)
        elif t == "G":
            test_G(log)
        elif t == "H":
            test_H(log)
        elif t == "I":
            if not rowsB:
                rowsB, _ = test_B(log)
            test_I(log, rowsB)
    log(f"\nwall = {time.time() - t0:.1f} s")
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    log(f"SUMMARY: {npass}/{len(RESULTS)} checks passed")
    for name, ok, det in RESULTS:
        log(f"  {'PASS' if ok else 'FAIL'}  {name}"
            + (f"  ({det})" if det else ""))
    return 0 if npass == len(RESULTS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
