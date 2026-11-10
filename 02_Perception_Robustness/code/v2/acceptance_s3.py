"""Stage-3 acceptance tests A-F.

    python -X utf8 -m v2.acceptance_s3              # all tests
    python -X utf8 -m v2.acceptance_s3 --tests A B  # subset

A  Inertness -- the sensing interface switched on but configured to the
   V1-equivalent (full visibility, exact, zero delay, no loss) plus the Stage-2
   TTC tuple reproduces the Stage-2 trajectory **bit-exactly**, whole trajectory.
B  Gating is real -- the controller's first obstacle-visible tick equals the
   geometric detection tick (70 m) within one control period, is later at 40 m,
   stays invisible while the obstacle is out of range, a scripted visibility
   flip changes the controller input, and no ground truth reaches the rollout
   predictor.  Assertion-based: the interface's own per-tick assertions run on a
   whole S3 run.
C  Observable-time table (speed x geometry), with the implied achievable T_PRE
   and the unachievable cells identified.
D  Threshold freezing -- the derived tuple, its measured margin, the gates, and
   a behavioural 40/70/100 m sensitivity run.
E  Honest-sensing pilot -- Stage-2 (full visibility) vs Stage-3 (70 m +
   realistic quality) vs Stage-3 + the frozen tuple, full method and B2, on S2
   and S3, seeds 1000/1001.
F  Determinism and joint limits -- repeat runs 0 diff, jobs=1 vs jobs=4,
   C3/R4/lateral violations 0 and still binding.
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

from config import default_cfg  # noqa: E402
from obstacle import Obstacle  # noqa: E402
from path import straight_path  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402

from v2.config_v2 import (STAGE2_TTC, STAGE3_TTC,  # noqa: E402
                          compat_v1cfg, sim_overrides, stage3_v2cfg)
from v2.metrics_v2 import summarize_v2  # noqa: E402
from v2.observability_v2 import load_reference, measure  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402
from v2.sensing_v2 import Sensorium  # noqa: E402
from v2.sim_v2 import run_scene_v2  # noqa: E402

OUT = ROOT / "results_v2_stage3"
REF = OUT / "reference"
OBS = OUT / "observability"
CAL_SEEDS = list(range(1000, 1020))

TRACE_COLS = ["t", "step", "alive", "spos", "shead", "sspeed",
              "rpos", "rhead", "rspeed", "romega", "opos"]

# V1-equivalent sensing: everything the interface offers, switched off.
V1_SENSE = dict(sense_mode="range", sense_r_obs=1e9, sense_r_animal=10.0,
                sense_err=0.0, sense_delay=0.0, sense_loss=0.0, sense_ttl=0.0)

# Stage-3 shipped interface, initial configuration (exact quality).
S3_SENSE = dict(sense_mode="range", sense_r_obs=70.0, sense_r_animal=25.0,
                sense_err=0.0, sense_delay=0.0, sense_loss=0.0, sense_ttl=0.30)

# "realistic" observation quality for the honest-sensing pilot (mid levels).
REAL_Q = dict(sense_err=0.2, sense_delay=0.2, sense_loss=0.05)


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


def _norm(v):
    """Normalise a summary value so None/nan/float repr compare consistently."""
    import math
    if v is None:
        return "nan"
    if isinstance(v, bool):
        return repr(v)
    if isinstance(v, (int, float, np.floating, np.integer)):
        f = float(v)
        return "nan" if not math.isfinite(f) else repr(f)
    return str(v)


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
# A. inertness
# ===========================================================================
def test_A(log):
    log("=" * 78)
    log("A. inertness: V1-equivalent sensing + Stage-2 tuple == Stage-2, "
        "bit-exactly")
    cfg = stage3_v2cfg("M1", **V1_SENSE, **STAGE2_TTC)
    log(f"  config_id={cfg.config_id}  "
        f"(sense_mode={cfg.sense_mode}, R_obs={cfg.sense_r_obs:g}, "
        f"R_animal={cfg.sense_r_animal:g}, ttl={cfg.sense_ttl})")
    allok = True
    rows = []
    for name, sid, seed in (("S2_s1000", "S2", 1000), ("S3_s1000", "S3", 1000),
                            ("S3_s1001", "S3", 1001), ("S1_s1002", "S1", 1002)):
        ref = np.load(REF / f"{name}.npz")
        meta = json.loads((REF / f"{name}.json").read_text(encoding="utf-8"))
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                            return_res=True, record_states=True)
        cur = trace_arrays(res)
        bad = diff_traces(cur, {k: ref[k] for k in ref.files})
        n = int(sum(v.size for v in cur.values()))
        # the summary row must also be identical (config_id is expected to
        # change, because the configuration record gained Stage-3 fields; the
        # timing column ctrl_ms is excluded as in Stage 2)
        sdiff = [k for k in meta["summary"]
                 if k not in ("ctrl_ms", "config_id")
                 and _norm(row.get(k)) != _norm(meta["summary"][k])]
        ok = (not bad) and (not sdiff)
        allok &= ok
        rows.append(dict(cell=name, ticks=len(res["trace_states"]),
                         values=n, trace_ok=not bad,
                         summary_mismatch=len(sdiff),
                         first_bad=str(bad[:1]) if bad else ""))
        check(f"A/{name}: M0..M1 + V1-equivalent sensing == Stage-2 "
              f"({n} values)", ok,
              "" if ok else f"trace {bad[:1]} summary {sdiff[:3]}")
    write_csv(OUT / "A_inertness.csv", rows)

    # ---- informational: a positive track TTL is *memory* V1 does not have ----
    cfg2 = stage3_v2cfg("M1", **{**V1_SENSE, "sense_ttl": 0.30, **STAGE2_TTC})
    ref = np.load(REF / "S3_s1000.npz")
    r2 = run_cell("S3", "Ours_rollout", 1000, v2cfg=cfg2, return_res=True,
                  record_states=True)
    bad2 = diff_traces(trace_arrays(r2[1]), {k: ref[k] for k in ref.files})
    log(f"  [info] the same arm with sense_ttl=0.30 s (a tracker that coasts a "
        f"vanished obstacle): {'identical on this cell' if not bad2 else 'diverges at ' + str(bad2[0])}")
    log("         The shipped inert arm uses sense_ttl=0.0 because V1's "
        "obstacle list is memoryless.  A positive TTL is tracker *memory*; "
        "where it coincides here it is because the coasted track sits far "
        "behind the flock and changes no command -- we do not rely on that.")
    return allok


# ===========================================================================
# B. gating is real
# ===========================================================================
def _first_tick(mask, t, t_min=0.0):
    idx = np.where(mask & (t >= t_min - 1e-9))[0]
    return float(t[idx[0]]) if len(idx) else np.nan


def _visible_ticks(res, k):
    return [r["t"] for r in res["sense_log"] if k in r["obs_index"]]


def test_B(log):
    log("=" * 78)
    log("B. gating is real: first visible tick vs geometric detection")
    ok = True
    rows = []

    # ---- B0: assertion-instrumented run ------------------------------
    cfg = stage3_v2cfg("M1", **{**S3_SENSE, "sense_assert": True, **REAL_Q})
    try:
        _, res = run_cell("S3", "Ours_rollout", 1000, v2cfg=cfg,
                          return_res=True, record_states=True)
        err = None
    except AssertionError as e:            # pragma: no cover
        res, err = None, str(e)
    ok &= check("B/interface assertions hold on a whole S3 run "
                "(sense_assert=True, 70 m + err/delay/loss)", err is None,
                str(err))
    if res is not None:
        log(f"  asserted run: {len(res['logs'])} control ticks, "
            f"{res['v2']['sense_publish_steps']} published ticks, "
            f"max track age {res['v2']['sense_track_age_max']:.2f} s")

    # ---- B1: 70 m vs 40 m vs out-of-range on the same scene ----------
    #   A synthetic S2-like scene whose obstacle starts ~129 m away and is
    #   aimed at where the flock will be at t = 40 s, so it is *outside* both
    #   ranges when released and becomes a genuine threat.
    path = straight_path(0, 70, halfwidth=6.0)
    out_obs = [Obstacle([128.0, 48.0], [-2.0, -1.2], radius=1.2,
                        life=200.0, t_spawn=0.0)]
    for R in (70.0, 40.0):
        v2 = stage3_v2cfg("M1", **{**S3_SENSE, "sense_r_obs": R,
                                   "sense_assert": True})
        cfg1 = replace(default_cfg(n_sheep=36, n_robots=12),
                       **sim_overrides(v2))
        from v2.controller_v2 import make_controller
        from run_experiments import ALGS
        ctrl = make_controller("Ours_rollout", ALGS, v2, {"divert_shift": 3.0})
        r = run_scene_v2(cfg1, path, out_obs, ctrl, seed=1000, T_max=70.0,
                         s0=8.0, v2cfg=v2, record_states=True)
        tr = trace_arrays(r)
        t = tr["t"]
        dmin = np.linalg.norm(tr["opos"][:, 0, :][:, None, :]
                              - tr["rpos"], axis=2).min(axis=1)
        geo = _first_tick(dmin <= R, t, t_min=cfg1.dt)
        vis = _visible_ticks(r, 0)
        first_vis = vis[0] if vis else np.nan
        lag = first_vis - geo if (np.isfinite(first_vis)
                                  and np.isfinite(geo)) else np.nan
        rows.append(dict(R=R, seed=1000, tau_geom=geo, tau_visible=first_vis,
                         lag=lag, n_visible_ticks=len(vis),
                         dmin_at_release=float(dmin[0]),
                         t_pre_on=next((l["t"] for l in r["logs"]
                                        if l["tag"] == "pre"), np.nan),
                         t_divert_on=next((l["t"] for l in r["logs"]
                                           if l["tag"] == "divert"), np.nan),
                         min_dist=float(r["dmin_sheep_obs"]),
                         collision_rate=float(r["lost"] / r["n0"])))
        tol = cfg1.ctrl_period * cfg1.dt + 1e-9
        good = np.isfinite(geo) and np.isfinite(first_vis) and abs(lag) <= tol
        ok &= check(f"B/R={R:g} m: first visible tick matches the geometric "
                    f"detection tick within one control period "
                    f"(geo {geo:.2f} s, visible {first_vis:.2f} s)", good,
                    f"lag {lag:+.2f} s")
        log(f"    obstacle starts {dmin[0]:.1f} m away -> invisible until "
            f"t={first_vis:.2f} s; controller saw it on {len(vis)} ticks; "
            f"PRE on {rows[-1]['t_pre_on']}, DIVERT on "
            f"{rows[-1]['t_divert_on']}, min_dist "
            f"{rows[-1]['min_dist']:.2f} m")
    # 40 m must be strictly later than 70 m, and the delay must reach the
    # controller's own state machine (not just the view)
    ok &= check("B/40 m detects strictly later than 70 m",
                rows[1]["tau_visible"] > rows[0]["tau_visible"] + 5.0,
                f"{rows[0]['tau_visible']:.2f} s -> "
                f"{rows[1]['tau_visible']:.2f} s")
    ok &= check("B/the gating reaches the controller: the 40 m run's PRE onset "
                "is later than the 70 m run's (or absent)",
                (not np.isfinite(rows[1]["t_pre_on"]))
                or (rows[1]["t_pre_on"] > rows[0]["t_pre_on"] + 1.0),
                f"PRE {rows[0]['t_pre_on']} s (70 m) vs "
                f"{rows[1]['t_pre_on']} s (40 m)")
    write_csv(OUT / "B_first_visible.csv", rows)

    # ---- B2: scripted visibility flip (unit level, tick exact) -------
    v2 = stage3_v2cfg("M1", **S3_SENSE)
    cfg1 = replace(default_cfg(n_sheep=36, n_robots=12), **sim_overrides(v2))
    sen = Sensorium(v2, cfg1, seed=1000)
    robot = np.array([[0.0, 0.0], [2.0, 0.0]])
    sheep = np.array([[5.0, 0.0], [30.0, 0.0], [-40.0, 0.0]])
    svel = np.zeros_like(sheep)

    class _O:
        def __init__(self, p):
            self.pos = np.array(p, dtype=float)
            self.vel = np.array([0.0, 0.0])
            self.radius = 1.2

        def is_alive(self):
            return True

    class _V:
        sheep_pos_global = sheep
        sheep_vel_global = svel
        robot_pos = robot
        obs_pos = np.zeros((0, 2))

    flip = []
    t = 0.0
    # obstacle walks in from 100 m to 10 m, then walks away again
    for i in range(110):
        x = 100.0 - 2.0 * i if i < 55 else 10.0 + 2.0 * (i - 55)
        o = _O([x, 0.0])
        sen.update(t, i, True, robot, [o], sheep, svel)
        v = sen.publish(_V(), t)
        flip.append(dict(t=t, x=x, n_obs=v.obs_active_n,
                         pos=(v.obs_pos[0].tolist() if v.obs_active_n else None),
                         n_animals=len(v.sheep_pos)))
        t += cfg1.dt
    tbl = [r for r in flip]
    for r in tbl:
        r["dmin"] = float(np.min(np.linalg.norm(
            np.array([r["x"], 0.0])[None, :] - robot, axis=1)))
    entry = next((r for r in tbl if r["n_obs"] > 0), None)
    exit_ = next((r for r in tbl if r["n_obs"] == 0 and entry is not None
                  and r["t"] > entry["t"]), None)
    det_geo = next((r for r in tbl if r["dmin"] <= 70.0), None)
    ok &= check("B/flip: obstacle invisible beyond R, visible from the tick it "
                "enters R, invisible again once it leaves",
                entry is not None and exit_ is not None
                and abs(entry["t"] - det_geo["t"]) < 1e-9,
                f"entered at t={entry['t']:.2f} s (d={entry['dmin']:.2f} m), "
                f"left at t={exit_['t']:.2f} s (d={exit_['dmin']:.2f} m)")
    # an animal 30 m or 40 m away is outside the 25 m radius of every robot ->
    # absent; only the 5 m one is published.  The 70 m obstacle channel does
    # not change that (the two channels are independent).
    ok &= check("B/channel independence: only the animal within 25 m of a "
                "robot is published (the 70 m obstacle range does not widen "
                "the animal radius)",
                tbl[-1]["n_animals"] == 1,
                f"{tbl[-1]['n_animals']}/3 animals published "
                f"(1 is within 25 m of a robot)")
    write_csv(OUT / "B_flip.csv", tbl)

    # ---- B3: no ground truth in the predictor path --------------------
    from v2.controller_v2 import OursRolloutV2, SpeedPolicyController, unpack

    seen = {}

    class Spy(OursRolloutV2):
        def _predict(self, view, cfg, offsets):
            seen["op"] = np.array(view.obs_pos, dtype=float)
            seen["ov"] = np.array(view.obs_vel, dtype=float)
            seen["n_act"] = int(view.obs_active_n)
            seen["t"] = float(view.t)
            return super()._predict(view, cfg, offsets)

    delay = 0.3
    v2d = stage3_v2cfg("M1", **{**S3_SENSE, "sense_delay": delay,
                                "sense_assert": True})
    scn = SCENARIOS["S2"]()
    cfgd = replace(scn["cfg"], **sim_overrides(v2d))
    core = Spy(v2cfg=v2d, t_pre=v2d.ttc_pre, t_divert=v2d.ttc_divert,
               t_rec=v2d.ttc_rec, hyst=v2d.ttc_hyst, divert_shift=3.0)
    ctrl = SpeedPolicyController(core, v2d)
    res = run_scene_v2(cfgd, scn["path"], scn["obstacles"], ctrl, seed=1000,
                       T_max=scn["T_max"], s0=scn["s0"], v2cfg=v2d,
                       record_states=True)
    # truth obstacle position on the recorded grid
    tr = trace_arrays(res)
    tgrid = tr["t"]
    opos = tr["opos"][:, 0, :]
    t_cap = seen["t"]
    i_now = int(np.argmin(np.abs(tgrid - t_cap)))
    i_old = int(np.argmin(np.abs(tgrid - max(t_cap - delay, 0.0))))
    d_now = float(np.linalg.norm(seen["op"][0] - opos[i_now]))
    d_old = float(np.linalg.norm(seen["op"][0] - opos[i_old]))
    ok &= check("B/predictor sees the *stale* track, not the current truth",
                d_old < 1e-9 and d_now > 1e-6,
                f"|est - truth(t)| = {d_now:.4f} m, "
                f"|est - truth(t-delay)| = {d_old:.2e} m")
    log(f"  live predictor: {len(core.plans)} plans; captured at t={t_cap:.2f} "
        f"s with {seen['n_act']} published obstacle(s); truth moved "
        f"{d_now:.3f} m in {delay:.1f} s")
    # the published set can never exceed the number of alive obstacles
    max_tracks = max(r["n_obs"] for r in res["sense_log"]) if \
        res["sense_log"] else 0
    ok &= check("B/at most one obstacle is published for a one-obstacle scene",
                max_tracks <= 1, f"max published = {max_tracks}")
    return ok


# ===========================================================================
# C. observable-time table
# ===========================================================================
def test_C(log):
    log("=" * 78)
    log("C. observable warning time per speed x geometry (R = 70 m, exact)")
    p = OBS / "observable_table_R70.csv"
    if not p.exists():
        raise SystemExit("run `python -m v2.observability_v2` first")
    with open(p, newline="", encoding="utf-8-sig") as f:
        agg = list(csv.DictReader(f))
    cal = json.loads((OUT / "ttc_calibration.json").read_text(encoding="utf-8"))
    t_pre = cal["frozen"]["ttc_pre"]
    hdr = ("| geometry | v [m/s] | first detect [s] | corridor [s] | "
           "flock contact [s] | T_obs mean [s] | p10 [s] | max achievable "
           "T_PRE [s] |")
    log("  " + hdr)
    log("  " + "|---|---|---|---|---|---|---|---|")
    rows = []
    for r in agg:
        g = r["geometry"]
        v = float(r["speed"])
        row = dict(geometry=g, speed=v,
                   tau_det=float(r["tau_det_mean"]),
                   tau_corr=float(r["tau_corr_mean"]),
                   tau_hit=float(r["tau_hit_mean"]),
                   T_obs_mean=float(r["T_obs_mean"]),
                   T_obs_p10=float(r["T_obs_p10"]),
                   T_PRE_implied=float(r["T_obs_p10"]),
                   passes_frozen=bool(float(r["T_obs_p10"]) >= t_pre),
                   passes_16=bool(float(r["T_obs_p10"]) >= 16.0))
        rows.append(row)
        log(f"  | {g} | {v:g} | {row['tau_det']:.2f} | {row['tau_corr']:.2f} | "
            f"{row['tau_hit']:.2f} | {row['T_obs_mean']:.2f} | "
            f"{row['T_obs_p10']:.2f} | {row['T_PRE_implied']:.2f} "
            f"{'' if row['passes_16'] else '<- 16 s FAILS'} |")
    write_csv(OUT / "C_observable_table.csv", rows)
    env = cal["measured"]["p10_design_envelope"]
    fails16 = [r for r in rows if not r["passes_16"]]
    check("C/the 3/4/6 m/s rows are all present",
          {r["speed"] for r in rows} == {3.0, 4.0, 6.0},
          f"{len(rows)} cells")
    check("C/frozen T_PRE is below the design-envelope 10th percentile",
          t_pre <= env, f"{t_pre:.1f} <= {env:.2f} s")
    check("C/(16,8) is unachievable in the identified cells",
          len(fails16) > 0,
          ", ".join(f"{r['geometry']}@{r['speed']:g}" for r in fails16))
    log("  reading: a cell's 10th percentile is the largest T_PRE that cell "
        "alone could support.")
    return True


# ===========================================================================
# D. threshold freezing + sensitivity
# ===========================================================================
def test_D(log):
    log("=" * 78)
    log("D. threshold freezing and the 40/70/100 m sensitivity")
    cal = json.loads((OUT / "ttc_calibration.json").read_text(encoding="utf-8"))
    fr = cal["frozen"]
    log(f"  frozen tuple (T_PRE, T_DIVERT, T_REC, h) = "
        f"({fr['ttc_pre']:.1f}, {fr['ttc_divert']:.1f}, {fr['ttc_rec']:.1f}, "
        f"{fr['ttc_hyst']:.1f}) s   [config_v2.STAGE3_TTC]")
    log(f"  rules: guard_PRE={cal['rules']['GUARD_PRE']}, "
        f"t_react={cal['rules']['t_react']:.2f} s, "
        f"t_lat={cal['rules']['t_lat']:.3f} s, "
        f"B_DIV={cal['rules']['B_DIV']:.2f} s, "
        f"median RECOVER duration="
        f"{cal['rules']['recover_duration_median']:.2f} s")
    m = cal["margins"]
    log(f"  measured margins: T_PRE vs p10 {m['T_PRE_vs_p10']:+.2f} s; "
        f"T_DIVERT vs min(T_obs) {m['T_DIVERT_vs_min_all']:+.2f} s; "
        f"T_PRE vs the 16 s proposal {m['T_PRE_vs_16']:+.2f} s")
    log(f"  required range: 12 s needs <= "
        f"{cal['required_range']['env_12']:.1f} m (3-4 m/s), 16 s needs <= "
        f"{cal['required_range']['env_16']:.1f} m (3-4 m/s) / "
        f"{cal['required_range']['stress_16']:.1f} m (6 m/s)")
    ok = all(g["ok"] for g in cal["gates"])
    ok &= check("D/all calibration gates pass", ok,
                "; ".join(f"{g['name']}={'ok' if g['ok'] else 'FAIL'}"
                          for g in cal["gates"]))
    ok &= check("D/config_v2.STAGE3_TTC == the frozen JSON",
                all(abs(float(STAGE3_TTC[k]) - float(fr[k])) < 1e-9
                    for k in fr), str(STAGE3_TTC))

    # ---- behavioural sensitivity at 40/70/100 m ----------------------
    rows = []
    for R in (40.0, 70.0, 100.0):
        for sid, seed in (("S2", 1000), ("S3", 1000)):
            cfg = stage3_v2cfg("M1", **{**S3_SENSE, "sense_r_obs": R})
            row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                                return_res=True)
            v = res["v2"]
            rows.append(dict(R=R, scenario=sid, seed=seed,
                             first_detect_1=v.get("sense_first_detect_1"),
                             t_pre_on=row["t_pre_on"],
                             t_divert_on=row["t_divert_on"],
                             anticipation_gap=row["obs_anticipation_gap"],
                             collision_rate=row["collision_rate"],
                             min_dist=row["min_dist"], rho=row["rho_avg"],
                             max_dev=row["max_dev"],
                             success=row["success"],
                             ended=res["terminated_by"]))
            log(f"  R={R:5.0f} m {sid}: first detect "
                f"{v.get('sense_first_detect_1')}, PRE {row['t_pre_on']}, "
                f"DIVERT {row['t_divert_on']}, contacts {row['collision_rate']:.3f}"
                f", min_dist {row['min_dist']:.2f} m, rho {row['rho_avg']:.3f}"
                f", max_dev {row['max_dev']:.2f} m, {res['terminated_by']}")
    write_csv(OUT / "D_range_sensitivity.csv", rows)
    # the 40 m arm must detect later, and the warning band must shrink with R
    def _fd(R, sid, key):
        for r in rows:
            if r["R"] == R and r["scenario"] == sid:
                return r[key]
        return np.nan
    ok &= check("D/range binds: first detection is later at 40 m than at 70 m",
                _fd(40.0, "S2", "first_detect_1")
                > _fd(70.0, "S2", "first_detect_1") + 1.0,
                f"S2 70 m: {_fd(70.0, 'S2', 'first_detect_1')}, "
                f"40 m: {_fd(40.0, 'S2', 'first_detect_1')}")
    ok &= check("D/the anticipation gap (PRE onset - first detection) grows "
                "with the range",
                _fd(100.0, "S2", "anticipation_gap")
                > _fd(40.0, "S2", "anticipation_gap"),
                f"40/70/100 m: "
                f"{_fd(40.0, 'S2', 'anticipation_gap'):.2f} / "
                f"{_fd(70.0, 'S2', 'anticipation_gap'):.2f} / "
                f"{_fd(100.0, 'S2', 'anticipation_gap'):.2f} s")
    return ok


# ===========================================================================
# E. honest-sensing pilot
# ===========================================================================
def _pilot_row(arm, sid, seed, alg, v2cfg):
    row, res = run_cell(sid, alg, seed, v2cfg=v2cfg, return_res=True)
    return dict(arm=arm, scenario=sid, seed=seed, alg=alg,
                collision_rate=row["collision_rate"],
                n_contact_so=res["v2"].get("n_contact_events_so"),
                min_dist=row["min_dist"],
                min_clearance_so=res["v2"].get("min_surface_clearance_so"),
                rho=row["rho_avg"], max_dev=row["max_dev"],
                success=row["success"], ended=res["terminated_by"],
                t_pre_on=row["t_pre_on"], t_divert_on=row["t_divert_on"],
                first_detect=res["v2"].get("sense_first_detect_1"),
                anticipation_gap=row["obs_anticipation_gap"],
                sense_active=res["v2"].get("sense_active"))


def test_E(log):
    log("=" * 78)
    log("E. honest-sensing pilot: does the anticipation benefit survive "
        "gating?")
    arms = {
        "S2_fullvis": stage3_v2cfg("M1", sense_mode="exact", **STAGE2_TTC),
        "S3_gated_exactQ": stage3_v2cfg("M1", **S3_SENSE, **STAGE2_TTC),
        "S3_gated_realQ": stage3_v2cfg("M1", **{**S3_SENSE, **REAL_Q,
                                                **STAGE2_TTC}),
        "S3_gated_frozen": stage3_v2cfg("M1", **S3_SENSE),
    }
    rows = []
    for arm, cfg in arms.items():
        for sid in ("S2", "S3"):
            for seed in (1000, 1001):
                for alg in ("Ours_rollout", "B2"):
                    rows.append(_pilot_row(arm, sid, seed, alg, cfg))
    write_csv(OUT / "E_pilot.csv", rows)
    # ---- paired aggregation ------------------------------------------
    aggs = {}
    for r in rows:
        a, s, g = r["arm"], r["scenario"], r["alg"]
        aggs.setdefault((a, g), []).append(r)
        aggs.setdefault(("__all__", g), []).append(r)
    log(f"  {'arm':<18}{'scenario':<10}{'alg':<13}{'C':>7}{'min_dist':>10}"
        f"{'rho':>8}{'max_dev':>9}{'PRE on':>8}{'detect':>8}")
    table = []
    for (arm, alg), rs in sorted(aggs.items()):
        if arm == "__all__":
            continue
        for sid in ("S2", "S3"):
            sub = [r for r in rs if r["scenario"] == sid]
            if not sub:
                continue
            C = float(np.mean([r["collision_rate"] for r in sub]))
            md = float(np.nanmean([r["min_dist"] for r in sub]))
            rho = float(np.mean([r["rho"] for r in sub]))
            dev = float(np.mean([r["max_dev"] for r in sub]))
            pre = [r["t_pre_on"] for r in sub if r["t_pre_on"] is not None]
            det = [r["first_detect"] for r in sub
                   if r["first_detect"] is not None]
            table.append(dict(arm=arm, scenario=sid, alg=alg, C=C, min_dist=md,
                              rho=rho, max_dev=dev,
                              t_pre_on=float(np.mean(pre)) if pre else None,
                              first_detect=float(np.mean(det)) if det else None))
            log(f"  {arm:<18}{sid:<10}{alg:<13}{C:>7.3f}{md:>10.2f}{rho:>8.3f}"
                f"{dev:>9.2f}{(np.mean(pre) if pre else float('nan')):>8.2f}"
                f"{(np.mean(det) if det else float('nan')):>8.2f}")
    write_csv(OUT / "E_pilot_summary.csv", table)

    def get(arm, alg, sid, key):
        for t in table:
            if t["arm"] == arm and t["alg"] == alg and t["scenario"] == sid:
                return t[key]
        return np.nan

    # the scientific statement: the Ours-vs-B2 margin in each arm
    log("\n  Ours - B2 margin (negative = Ours has fewer contacts):")
    benefit = {}
    for arm in arms:
        for sid in ("S2", "S3"):
            dC = get(arm, "Ours_rollout", sid, "C") - get(arm, "B2", sid, "C")
            benefit[(arm, sid)] = dC
            log(f"    {arm:<18}{sid}  dC = {dC:+.3f}")
    surv = [benefit[("S3_gated_realQ", sid)] for sid in ("S2", "S3")]
    base = [benefit[("S2_fullvis", sid)] for sid in ("S2", "S3")]
    ok = check("E/report: the Ours-vs-B2 contact advantage is preserved "
               "(not reversed) under 70 m + realistic quality",
               all(x <= 0 for x in surv),
               f"gated dC={['%+.3f' % x for x in surv]} vs full-visibility "
               f"dC={['%+.3f' % x for x in base]}")
    log("  NOTE: n = 2 seeds X 2 scenarios per cell.  This is a *pilot*; it "
        "cannot establish significance (see §9 of the report).")
    return ok


# ===========================================================================
# F. determinism and joint limits
# ===========================================================================
def test_F(log):
    log("=" * 78)
    log("F. determinism and joint limits under the Stage-3 interface")
    cfg = stage3_v2cfg("M1", **{**S3_SENSE, **REAL_Q})
    ok = True
    for sid, seed in (("S3", 1000), ("S2", 1001)):
        _, r1 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        _, r2 = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                         return_res=True, record_states=True)
        bad = diff_traces(trace_arrays(r1), trace_arrays(r2))
        ok &= check(f"F/repeat run identical ({sid}, seed {seed}, "
                    f"{len(r1['trace_states'])} ticks)", not bad, f"{bad[:2]}")

    # ---- jobs = 1 vs 4 ----
    import subprocess
    outs = {}
    sets = [f"{k}={v}" for k, v in {**S3_SENSE, **REAL_Q}.items()]
    sets += [f"ttc_{k[4:]}={v}" for k, v in STAGE3_TTC.items()
             if k.startswith("ttc_")] + ["animal_model=M1",
                                         "speed_policy=manoeuvre"]
    for jobs in (1, 4):
        d = OUT / f"_det_j{jobs}"
        d.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, "-X", "utf8", "-m", "v2.runner_v2",
               "--scenarios", "S3", "--seeds", "2", "--seed-offset", "1000",
               "--algs", "Ours_rollout", "--jobs", str(jobs), "--out", str(d),
               "--set"] + sets
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

    # ---- limit violations ----
    rows = []
    for sid, seed in (("S2", 1000), ("S3", 1000), ("S3", 1001)):
        row, res = run_cell(sid, "Ours_rollout", seed, v2cfg=cfg,
                            return_res=True)
        m = summarize_v2(res, cfg)
        r = dict(scenario=sid, seed=seed,
                 **{k: m.get(k) for k in (
                     "violation_C3", "violation_R4", "violation_lat",
                     "violation_C1_low", "violation_C1_high", "violation_C2",
                     "violation_R1", "violation_R3", "LAT_UNRESOLVED",
                     "bind_C3_sheep", "bind_R4_robot", "bind_LAT_robot",
                     "n_contact_events_so", "min_surface_clearance_ro",
                     "min_surface_clearance_so", "dmin_robot_robot",
                     "success", "rho_avg", "max_dev", "terminated_by")})
        rows.append(r)
        log(f"  {sid}/{seed}: C3={r['violation_C3']} R4={r['violation_R4']} "
            f"lat={r['violation_lat']} C1lo={r['violation_C1_low']} "
            f"C1hi={r['violation_C1_high']} | bind C3={r['bind_C3_sheep']} "
            f"R4={r['bind_R4_robot']} lat={r['bind_LAT_robot']} | "
            f"ro clr {r['min_surface_clearance_ro']:.2f} m, so clr "
            f"{r['min_surface_clearance_so']:.2f} m, rr "
            f"{r['dmin_robot_robot']:.2f} m | rho {r['rho_avg']:.3f} | "
            f"{r['terminated_by']}")
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
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", nargs="+", default=list("ABCDEF"))
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    log = Tee(OUT / "ACCEPTANCE.txt")
    LOG[0] = log
    log("V2 Stage-3 acceptance tests")
    log(f"python {sys.version.split()[0]}, numpy {np.__version__}")
    log(f"shipped sensing: {json.dumps(S3_SENSE)}")
    log(f"realistic quality: {json.dumps(REAL_Q)}")
    log(f"frozen tuple: {json.dumps(STAGE3_TTC)}   "
        f"stage2 tuple: {json.dumps(STAGE2_TTC)}")
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
        [dict(name=n, ok=ok, detail=d) for n, ok, d in RESULTS], indent=2),
        encoding="utf-8")
    return 0 if npass == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
