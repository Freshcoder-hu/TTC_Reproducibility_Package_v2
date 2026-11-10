"""Stage-4 scenario geometry: the manuscript Table-14 V2 family.

WHY A SEPARATE MODULE
---------------------
``scenarios.py`` is V1 and must stay bit-reproducible, so — exactly as Stage 2
added ``v2/corridor_v2.py`` instead of editing it — the Stage-4 geometry lives
here.  Nothing in this module is imported by ``scenarios.py`` or by any V1 code
path; a Stage-4 run resolves its scene through :func:`resolve_scene`.

THE FAMILY (Table 14, with the two author decisions of this stage)
------------------------------------------------------------------
| family | V1 (length / total width) | Stage 4 |
|---|---|---|
| S1   no obstacle | 70 m / 12 m | **140 m / 16 m** |
| S1N  bent, no obstacle | 50 m / 8 m | **140 m / 16 m**, bend at s=60 m, 75 deg |
| S2   single obstacle | 70 m / 12 m | **140 m / 16 m** |
| S2R3/R2/R1 risk levels | 70 m / 12 m | **140 m / 16 m** |
| S3   sequential | 110 m / 12 m | **220 m / 16 m**, three releases |
| S4S/M/L fixed-width scale | width varied with scale | **140 m / 16 m** at every scale |

``W`` is the *half*-width everywhere (manuscript: "W always denotes half-width"),
so 16 m total width = ``halfwidth=8.0``.  Deviations from Table 14 are exactly
two, both author decisions recorded in ``V2_STAGE4_REPORT.md`` §2:
S1N is built at 140 m / 16 m (Table 14 offers 100 m / 12 m) so that corridor
width is not confounded across the family, and the S4 *adapted-width* family is
not built (it is a separate Table-14 family; only the fixed-width one is in
scope here).

THE RELEASE RULE (Stage-3 §10 hook 1)
-------------------------------------
    E      = point on the path at arc length ``e_s``, offset ``lateral`` along
             the path normal (the *intended encounter point*)
    u      = obstacle approach direction, fixed in the path frame
    start  = E - u * v * T_enc
    T_enc  >= 2 R / v        with R = 70 m  (the Stage-3 obstacle range)

so the obstacle is at least ``2R`` from the intended encounter point when it
starts, which is what makes "initially outside the 70 m sensing range" true by
construction rather than by luck (the V1 geometry failed exactly this: its
obstacle was already 53 m from the flock at release, so 40/70/100 m produced
*bit-identical* trajectories).

Two release conventions are used, both inside the rule:

* **single-obstacle family (S2/S2R\\*/S4\\*)**: released at ``t=0`` with
  ``T_enc = t_arr(e_s)`` (the no-intervention flock's arrival time at E, taken
  from ``results_v2_stage4/geometry/reference_kinematics.json``).  The obstacle
  is therefore already on the road when the run starts — the realistic case —
  and its initial distance grows with its speed (a faster obstacle simply
  starts farther away, so the encounter point and the flock timing are
  unchanged).
* **S3 (sequential)**: released at ``t_spawn = t_arr(e_k) - 2R/v_k``, i.e. at
  the *smallest* lead time the rule allows, so that only one obstacle is
  developing at a time and the release *order* is a designed quantity.

Everything is a pure function of the scenario id; no cached file is read at run
time (the arrival table below is the measured probe output, pasted as
literals so a scene can never silently depend on a missing cache).

MEASURED CONSTANTS (``python -m v2.probe_geometry_s4``, seeds 1000-1004)
-----------------------------------------------------------------------
Passive (B1) controller, **no obstacles**, 36:12, M1, 16 m corridor:

* mean flock speed after the start-up transient: **0.96-0.99 m/s**
* flock lateral extent |lat| about the centroid: p50 1.54, p90 3.74,
  p95 4.33, p99 5.22 m  (this is what fixes the "graze" offset: an obstacle
  crossing at |lat| = 5.0 m clips the outer ~2 % of the flock if nothing
  avoids it, and a 3 m divert clears it)
* the goal (0.92 L) is reached at 126.9 s on the 140 m path and 206.1 s on the
  220 m path, so the Stage-4 horizon is set to 200 s / 320 s (>= 120 s
  reference, §5 of the report).
"""
from __future__ import annotations

import numpy as np

import corrections

corrections.install(environment=True, contact=True)

from config import default_cfg                       # noqa: E402
from obstacle import Obstacle                        # noqa: E402
from path import Path, bent_path, straight_path      # noqa: E402

# ---------------------------------------------------------------------------
# geometry constants
# ---------------------------------------------------------------------------
W_V2 = 8.0                 # half-width [m]  (16 m total width everywhere)
L_MAIN = 140.0             # S1/S1N/S2/S2R*/S4* path length [m]
L_S3 = 220.0               # S3 path length [m]
S0 = 8.0                   # flock/robot start arc length [m] (V1 value)
R_OBS = 1.2                # obstacle radius [m] (unchanged)
R_SENSE = 70.0             # Stage-3 frozen obstacle range [m]
V_OBS_DESIGN = 3.0         # nominal obstacle speed [m/s] (Table 13: 3/4 m/s)
GOAL_FRAC = 0.92           # config.SimCfg.goal_frac (fixed at Stage 1)
T_ENC_MIN_FACTOR = 2.0     # T_enc >= 2 R / v  (Stage-3 rule)

# T_max per family.  >= 120 s reference horizon; the no-intervention goal is
# reached at 127 s (140 m) / 206 s (220 m), so there is margin for the
# avoidance detour of the risk scenarios.
T_MAX_MAIN = 200.0
T_MAX_S3 = 320.0

# ---------------------------------------------------------------------------
# no-intervention arrival table: t_arr(s) measured on the 16 m corridor
# (v2.probe_geometry_s4, seeds 1000-1004, B1 passive, no obstacles)
# ---------------------------------------------------------------------------
_GRID_140 = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85,
             90, 95, 100, 105, 110, 115, 120, 125]
_T_140 = [3.35, 9.68, 14.59, 19.83, 25.06, 30.28, 35.46, 40.55, 45.73, 51.03,
          56.16, 61.45, 66.65, 71.86, 76.94, 82.10, 87.45, 92.76, 98.02,
          103.23, 108.33, 113.51, 118.58, 123.84]
_GRID_220 = list(range(10, 205, 5))          # 10 .. 200 m, 39 points
_T_220 = [3.35, 9.68, 14.58, 19.73, 24.96, 30.03, 35.26, 40.45, 45.71, 50.92,
          56.41, 61.74, 66.94, 72.26, 77.59, 82.76, 87.88, 93.22, 98.42,
          103.54, 108.69, 113.87, 118.95, 124.15, 129.40, 134.51, 139.76,
          144.94, 150.10, 155.32, 160.53, 165.76, 170.84, 176.16, 181.31,
          186.62, 191.88, 197.18, 202.48]

# approach directions in the PATH FRAME (x = along the path, y = path normal,
# i.e. "left" of the direction of travel).  These are the V1 S2/S3 direction
# (-0.8, -0.45) and its mirror, normalised.
U_BACK_LEFT = np.array([-0.8, -0.45])
U_BACK_LEFT = U_BACK_LEFT / np.linalg.norm(U_BACK_LEFT)
U_BACK_RIGHT = np.array([-0.8, 0.45])
U_BACK_RIGHT = U_BACK_RIGHT / np.linalg.norm(U_BACK_RIGHT)


def t_arr(s, length=L_MAIN):
    """No-intervention flock arrival time at arc length ``s`` [s]."""
    grid = np.array(_GRID_220 if length > L_MAIN else _GRID_140, dtype=float)
    tab = np.array(_T_220 if length > L_MAIN else _T_140, dtype=float)
    return float(np.interp(float(s), grid, tab))


def encounter_angle_deg(u):
    """Angle between the obstacle heading and the *reverse* path tangent.

    0 deg = head-on (the obstacle meets the flock head-on), 90 deg = the
    obstacle crosses the path perpendicular to it.
    """
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    return float(np.degrees(np.arccos(float(np.clip(-u[0], -1.0, 1.0)))))


# ---------------------------------------------------------------------------
# release construction
# ---------------------------------------------------------------------------
def encounter_point(path, e_s, lateral=0.0):
    """Point at arc length ``e_s`` offset ``lateral`` along the path normal."""
    p = np.asarray(path.point_at(float(e_s)), dtype=float)
    tang = np.asarray(path.tangent_at(float(e_s)), dtype=float)
    nrm = np.array([-tang[1], tang[0]])
    return p + nrm * float(lateral)


def release(path, e_s, lateral, u, speed, t_enc, t_spawn=0.0, life_slack=8.0,
            name=""):
    """One obstacle built with the Stage-3 release rule.

    ``start = E - u_hat * speed * T_enc``; ``E`` is the intended encounter
    point (arc length ``e_s``, normal offset ``lateral``).  Returns the
    ``Obstacle`` plus the design record used by the report table.
    """
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    E = encounter_point(path, e_s, lateral)
    vel = u * float(speed)
    start = E - vel * float(t_enc)
    life = float(t_enc) + float(life_slack)
    o = Obstacle(list(start), list(vel), radius=R_OBS, life=life,
                 t_spawn=float(t_spawn), a_max=0.0)
    rec = dict(obstacle=name or "o", e_s=float(e_s), lateral=float(lateral),
               u=[float(u[0]), float(u[1])], speed=float(speed),
               t_enc=float(t_enc), t_spawn=float(t_spawn),
               E=[float(E[0]), float(E[1])],
               start=[float(start[0]), float(start[1])],
               travel=float(np.linalg.norm(E - start)),
               angle_deg=encounter_angle_deg(u),
               life=life,
               t_enc_min=float(T_ENC_MIN_FACTOR * R_SENSE / float(speed)),
               rule_ok=bool(float(t_enc) >= T_ENC_MIN_FACTOR * R_SENSE
                            / float(speed) - 1e-9))
    return o, rec


# ---------------------------------------------------------------------------
# the scenario definitions
# ---------------------------------------------------------------------------
# graze offsets: |lat| of the intended encounter point.  5.0 m is just outside
# the flock's 95th-percentile lateral extent (4.33 m), so an obstacle crossing
# there clips the outer 1-3 % of the flock if nothing reacts, and a 3 m divert
# (the controller's own divert_shift) clears it.  §4 of the report verifies both
# halves of that sentence with a passive run and a full-method run.
GRAZE = 5.0

# S3 release plan: (e_s, lateral sign, speed, u, tag).
#   The first two obstacles are the **threat-arrival-during-recovery** spacing:
#   obstacle 2 is detected (enters the 70 m horizon) 0.3-1.4 s after obstacle 1's
#   DIVERT hands over to RECOVER, and its risk onset (TTC <= T_PRE) falls inside
#   that RECOVER window (measured 5/5 on calibration seeds 1000-1004, see
#   results_v2_stage4/geometry/s3_spacing_sweep.csv).  Obstacle 3 is the
#   **recovery-complete** spacing: its risk onset is ~50 s after the second
#   RECOVER has closed.  Before this stage, the V1 110 m geometry could not
#   produce either regime: every pair of threats merged into one continuous
#   DIVERT because the next obstacle was already inside the sensing range when
#   the previous one cleared.
_S3_PLAN = [
    dict(e_s=70.0, lateral=+GRAZE, speed=3.0, u=U_BACK_LEFT,
         tag="threat1_crossing"),
    dict(e_s=83.0, lateral=-GRAZE, speed=6.0, u=U_BACK_RIGHT,
         tag="threat_during_recovery"),
    dict(e_s=145.0, lateral=+GRAZE, speed=3.0, u=U_BACK_LEFT,
         tag="recovery_complete"),
]

# S2 family: same encounter point and angle, obstacle speed is the only
# variable (3 / 4 / 5 / 6 m/s).  V2S2 is the nominal single-obstacle case.
_S2_SPEEDS = {"V2S2": 4.0, "V2S2R3": 3.0, "V2S2R2": 5.0, "V2S2R1": 6.0}

_S4_GROUPS = {"V2S4S": (18, 6), "V2S4M": (36, 12), "V2S4L": (60, 20)}


def _base_cfg(n_sheep=36, n_robots=12):
    """SimCfg for the scene; the caller mirrors its V2Cfg over it."""
    return default_cfg(n_sheep=int(n_sheep), n_robots=int(n_robots),
                       path_halfwidth=W_V2)


def _single_obstacle_scene(sid, speed, n_sheep=36, n_robots=12):
    path = straight_path(0.0, L_MAIN, halfwidth=W_V2)
    t_enc = t_arr(70.0, L_MAIN)                      # flock arrival at E
    o, rec = release(path, 70.0, GRAZE, U_BACK_LEFT, speed, t_enc,
                     t_spawn=0.0, name="o1")
    return dict(name=f"{sid}_single_obstacle", path=path, obstacles=[o],
                cfg=_base_cfg(n_sheep, n_robots), T_max=T_MAX_MAIN, s0=S0,
                params=dict(divert_shift=3.0), design=[rec])


def scenario_s1():
    """140 m, 16 m wide, no obstacle."""
    return dict(name="V2S1_normal_path_140x16",
                path=straight_path(0.0, L_MAIN, halfwidth=W_V2), obstacles=[],
                cfg=_base_cfg(), T_max=T_MAX_MAIN, s0=S0, design=[])


def scenario_s1n():
    """140 m arc, 16 m wide, one 75 deg bend at s = 60 m (V1 S1N angle)."""
    path = bent_path(0.0, L_MAIN, bend_at=60.0, bend_angle=1.31,
                     halfwidth=W_V2)
    return dict(name="V2S1N_narrow_turn_140x16", path=path, obstacles=[],
                cfg=_base_cfg(), T_max=T_MAX_MAIN + 20.0, s0=S0, design=[])


def scenario_s2(sid="V2S2"):
    return _single_obstacle_scene(sid, _S2_SPEEDS[sid])


def scenario_s3():
    """220 m, 16 m wide, three sequential releases.

    Release times are *derived*: ``t_spawn_k = t_arr(e_k) - 2R/v_k``, the
    smallest lead the Stage-3 rule allows, so each obstacle starts exactly
    2R = 140 m from its intended encounter point and the release order is a
    designed quantity (e_1 < e_2 < e_3, hence t_spawn_1 < t_spawn_2 <
    t_spawn_3).
    """
    path = straight_path(0.0, L_S3, halfwidth=W_V2)
    obs, recs = [], []
    for i, p in enumerate(_S3_PLAN, start=1):
        t_enc = T_ENC_MIN_FACTOR * R_SENSE / p["speed"]
        t_spawn = t_arr(p["e_s"], L_S3) - t_enc
        o, rec = release(path, p["e_s"], p["lateral"], p["u"], p["speed"],
                         t_enc, t_spawn=t_spawn, name=f"o{i}")
        rec["tag"] = p["tag"]
        obs.append(o)
        recs.append(rec)
    return dict(name="V2S3_sequential_risk_220x16", path=path, obstacles=obs,
                cfg=_base_cfg(), T_max=T_MAX_S3, s0=S0,
                params=dict(divert_shift=3.0), design=recs)


def scenario_s4(sid):
    ns, nr = _S4_GROUPS[sid]
    return _single_obstacle_scene(sid, _S2_SPEEDS["V2S2"], ns, nr)


SCENARIOS_V2 = {
    "V2S1": scenario_s1,
    "V2S1N": scenario_s1n,
    "V2S2": lambda: scenario_s2("V2S2"),
    "V2S2R3": lambda: scenario_s2("V2S2R3"),
    "V2S2R2": lambda: scenario_s2("V2S2R2"),
    "V2S2R1": lambda: scenario_s2("V2S2R1"),
    "V2S3": scenario_s3,
    "V2S4S": lambda: scenario_s4("V2S4S"),
    "V2S4M": lambda: scenario_s4("V2S4M"),
    "V2S4L": lambda: scenario_s4("V2S4L"),
}

# case types of the pilot screening (manuscript Table 15 / §4.8)
PILOT_CASES = {"no_obstacle": "V2S1",
               "single_obstacle": "V2S2",
               "sequential_obstacle": "V2S3"}


def is_v2(sid):
    return str(sid).startswith("V2")


def resolve_scene(sid):
    """Scene for a Stage-4 runner: Stage-4 ids, or a V1 id passed through.

    Passing a V1 id through is what makes acceptance test A possible: the old
    V1 scene definition can be pushed through the Stage-4 runner and must give
    the Stage-3 trajectory bit-exactly.
    """
    if is_v2(sid):
        return SCENARIOS_V2[sid]()
    from scenarios import SCENARIOS                 # V1, read-only
    return SCENARIOS[sid]()


def cfg_for(scn, v2cfg, compat=False):
    """SimCfg for a scene with the V2 record mirrored onto it."""
    from dataclasses import replace

    from v2.config_v2 import sim_overrides
    cfg = scn["cfg"]
    if compat:
        return cfg
    cfg = replace(cfg, **sim_overrides(v2cfg))
    v2cfg.assert_consistent(cfg)
    return cfg


# ---------------------------------------------------------------------------
def design_table():
    """Flat per-obstacle design rows (report Table 1 / geometry_table.csv)."""
    rows = []
    for sid in SCENARIOS_V2:
        scn = SCENARIOS_V2[sid]()
        for k, r in enumerate(scn.get("design") or [], start=1):
            rows.append(dict(
                scenario=sid, scenario_name=scn["name"], path_len_m=float(
                    scn["path"].total), halfwidth_m=float(
                    scn["path"].halfwidth_at(0.0)), T_max_s=float(
                    scn["T_max"]), obstacle=k, e_s_m=r["e_s"],
                lateral_m=r["lateral"], speed_mps=r["speed"],
                angle_deg=r["angle_deg"], u=",".join(f"{x:+.3f}"
                                                     for x in r["u"]),
                t_enc_s=r["t_enc"], t_spawn_s=r["t_spawn"],
                start_x=r["start"][0], start_y=r["start"][1],
                travel_m=r["travel"], t_enc_min_s=r["t_enc_min"],
                rule_ok=r["rule_ok"], tag=r.get("tag", "")))
    return rows


def main():
    import csv
    from pathlib import Path as _P
    out = _P(__file__).resolve().parent.parent / "results_v2_stage4" / "geometry"
    out.mkdir(parents=True, exist_ok=True)
    rows = design_table()
    keys = list(rows[0])
    with open(out / "scene_design.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    hdr = (f"{'scenario':8s}{'obs':>4s}{'L':>6s}{'W':>5s}{'e_s':>7s}"
           f"{'lat':>6s}{'v':>5s}{'ang':>6s}{'T_enc':>7s}{'t_spawn':>8s}"
           f"{'start x':>9s}{'start y':>9s}{'travel':>8s}{'2R/v':>6s}  ok")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['scenario']:8s}{r['obstacle']:>4d}{r['path_len_m']:>6.0f}"
              f"{2 * r['halfwidth_m']:>5.0f}{r['e_s_m']:>7.1f}"
              f"{r['lateral_m']:>+6.1f}{r['speed_mps']:>5.1f}"
              f"{r['angle_deg']:>6.1f}{r['t_enc_s']:>7.2f}"
              f"{r['t_spawn_s']:>8.2f}{r['start_x']:>9.1f}"
              f"{r['start_y']:>9.1f}{r['travel_m']:>8.1f}"
              f"{r['t_enc_min_s']:>6.1f}  {r['rule_ok']}")
    print(f"\nwrote {out / 'scene_design.csv'}  ({len(rows)} obstacle rows)")


if __name__ == "__main__":
    main()
