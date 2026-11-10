"""Unit tests for the V2 transcription / projection claims.

    python -m v2.unit_tests_v2

These are the *structural* checks behind the report's claims; the end-to-end
checks are in :mod:`v2.acceptance_v2`.
"""
from __future__ import annotations

import numpy as np

import corrections

corrections.install(environment=True, contact=True)

import robots  # noqa: E402
import sheep as sheep_mod  # noqa: E402
from v2.animal_m0 import advance_sheep_m0  # noqa: E402
from v2.config_v2 import compat_v1cfg, default_v2cfg  # noqa: E402
from v2.fleet_v2 import RobotFleetV2  # noqa: E402
from v2.limits_v2 import (project_sheep_increment,  # noqa: E402
                          robot_step_project)
from v2.robot_safety_v2 import apply_robot_safety_v2  # noqa: E402

FAILED = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
          + (f"  -- {detail}" if detail else ""))
    if not ok:
        FAILED.append(name)


# ---------------------------------------------------------------------------
def t_safety_transcription():
    """apply_robot_safety_v2(guard off) == robots.apply_robot_safety, bit-exact."""
    rng = np.random.default_rng(0)
    cfg = _v1cfg()
    worst = 0
    for trial in range(200):
        M = int(rng.integers(1, 8))
        K = int(rng.integers(0, 4))
        pos = rng.uniform(-6, 6, (M, 2))
        cmd = rng.normal(0, 3.0, (M, 2))
        obs = rng.uniform(-6, 6, (K, 2)) if K else None
        a = robots.apply_robot_safety(cmd.copy(), pos, obs, cfg)
        b = apply_robot_safety_v2(cmd.copy(), pos, obs, cfg, hard=None,
                                  guard=False)
        if not np.array_equal(a, b):
            worst += 1
    check("apply_robot_safety_v2 == robots.apply_robot_safety (guard off)",
          worst == 0, f"{worst}/200 trials differ")


def t_sheep_transcription():
    """advance_sheep_m0(C3 off) == sheep._advance_sheep, bit-exact."""
    rng = np.random.default_rng(1)
    cfg = _v1cfg()
    v2c = compat_v1cfg()
    bad = 0
    for trial in range(20):
        n = int(rng.integers(2, 20))
        pos = rng.uniform(-5, 5, (n, 2))
        head = rng.uniform(-np.pi, np.pi, n)
        spd = rng.uniform(1.0, 1.5, n)
        rob = rng.uniform(-6, 6, (6, 2))
        st1 = sheep_mod.SheepState(pos.copy(), head.copy(), spd.copy())
        st2 = sheep_mod.SheepState(pos.copy(), head.copy(), spd.copy())
        sheep_mod._advance_sheep(st1, cfg, rob, None)
        advance_sheep_m0(st2, cfg, v2c, rob)
        if not (np.array_equal(st1.pos, st2.pos)
                and np.array_equal(st1.head, st2.head)
                and np.array_equal(st1.speed, st2.speed)):
            bad += 1
    check("advance_sheep_m0 (C3 off) == sheep._advance_sheep", bad == 0,
          f"{bad}/20 trials differ")


def t_fleet_transcription():
    """RobotFleetV2.step == robots.RobotFleet.step in compat configuration."""
    rng = np.random.default_rng(2)
    cfg = _v1cfg()
    v2c = compat_v1cfg()
    bad = 0
    for trial in range(30):
        M = int(rng.integers(2, 12))
        pos = rng.uniform(-5, 5, (M, 2))
        head = rng.uniform(-np.pi, np.pi, M)
        f1 = robots.RobotFleet(pos.copy(), head.copy(), cfg=cfg)
        f1.speed = rng.uniform(0, 2.6, M)
        f2 = RobotFleetV2(pos.copy(), head.copy(), cfg=cfg)
        f2.speed = f1.speed.copy()
        cmd = rng.normal(0, 3.0, (M, 2))
        K = 2
        obs = rng.uniform(-6, 6, (K, 2))
        for _ in range(5):                      # several steps
            f1.step(cmd.copy(), cfg, obs)
            f2.step(cmd.copy(), cfg, v2c, obs_pos=obs, ctrl_tick=False)
            if not (np.array_equal(f1.pos, f2.pos)
                    and np.array_equal(f1.head, f2.head)
                    and np.array_equal(f1.speed, f2.speed)):
                bad += 1
                break
    check("RobotFleetV2.step == robots.RobotFleet.step (compat)", bad == 0,
          f"{bad}/30 trials differ")


def t_box_projection():
    """The (V2) box projection is always admissible and is a bit-exact no-op."""
    rng = np.random.default_rng(3)
    v2 = default_v2cfg()
    dt = 0.05
    A = v2.a_sheep * dt
    bad_box = bad_noop = bad_lo = bad_out_of_band = 0
    n = 0
    for _ in range(4000):
        # The plant invariant is s in [s_min, s_max]; starting outside the band
        # is the documented "cannot reach the band in one step" case and is
        # counted separately below.
        s_old = float(rng.uniform(1.0, 1.5))
        dth = float(rng.uniform(-0.3, 0.3))
        s_des = float(rng.uniform(1.0, 1.5))
        # record whether the request was already admissible
        v_old = s_old * np.array([1.0, 0.0])
        v_req = s_des * np.array([np.cos(dth), np.sin(dth)])
        req_ok = (np.linalg.norm(v_req - v_old) <= A * (1 + 1e-12)
                  and abs(dth) <= 3.0 * dt + 1e-15
                  and 1.0 <= s_des <= 1.5)
        d_j, s_j, _ = project_sheep_increment(
            s_old, dth, s_des, dt=dt, a_s=v2.a_sheep, omega_s=v2.omega_sheep,
            s_lo=v2.s_min, s_hi=v2.s_max, eps_v=v2.eps_v)
        v_new = s_j * np.array([np.cos(d_j), np.sin(d_j)])
        n += 1
        if np.linalg.norm(v_new - v_old) > A * (1 + 1e-9):
            bad_box += 1
        if abs(d_j) > v2.omega_sheep * dt * (1 + 1e-12):
            bad_box += 1
        if s_j > v2.s_max * (1 + 1e-15):
            bad_box += 1
        if s_j < v2.s_min * (1 - 1e-9):
            bad_lo += 1
        if req_ok and (d_j != dth or s_j != s_des):
            bad_noop += 1
    check("(V2) box: projected state always admissible", bad_box == 0,
          f"{bad_box} violations over {n} random cases")
    check("(V2) box: bit-exact no-op when the request is already admissible",
          bad_noop == 0, f"{bad_noop}/{n}")
    check("(V2) box: C1 lower bound preserved (no spurious C1 violation)",
          bad_lo == 0, f"{bad_lo}/{n} below s_min")

    # from rest the band cannot be reached in one step: must be flagged, and the
    # result must still be inside the acceleration ball
    flagged = 0
    trials = 0
    for _ in range(500):
        s_old = float(rng.uniform(0.0, 0.2))     # near-zero speed
        dth = float(rng.uniform(-0.3, 0.3))
        s_des = float(rng.uniform(1.0, 1.5))
        d_j, s_j, info = project_sheep_increment(
            s_old, dth, s_des, dt=dt, a_s=v2.a_sheep, omega_s=v2.omega_sheep,
            s_lo=v2.s_min, s_hi=v2.s_max, eps_v=1e-6)
        v_old = s_old * np.array([1.0, 0.0])
        v_new = s_j * np.array([np.cos(d_j), np.sin(d_j)])
        trials += 1
        if info["plant_infeasible"] and np.linalg.norm(v_new - v_old) <= A * 1.001:
            flagged += 1
    check("(V2) box: near-zero speed -> C1 unreachable is flagged, C3 kept",
          flagged == trials, f"{flagged}/{trials}")


def t_robot_projection():
    """R1-R4 + lateral always hold after the robot projection."""
    rng = np.random.default_rng(4)
    v2 = default_v2cfg()
    dt = 0.05
    bad_r4 = bad_lat = bad_lin = bad_v = 0
    n = 0
    for _ in range(6000):
        v = float(rng.uniform(0, 4.5))
        om = float(rng.uniform(-3.5, 3.5))
        if v * abs(om) > v2.a_lat:             # keep the invariant assumption
            om = np.sign(om) * v2.a_lat / max(v, 1e-9)
        head = float(rng.uniform(-np.pi, np.pi))
        cmd = rng.normal(0, 3.0, 2) * rng.uniform(0, 1.7)
        v2n, h2, om2, info = robot_step_project(
            cmd, v, head, om, v_max=4.5, omega_max=3.5, a_lin=4.0,
            a_ang=v2.a_ang, a_lat=v2.a_lat, dt=dt, ang_on=True, lat_on=True)
        n += 1
        if abs(om2 - om) > v2.a_ang * dt * (1 + 1e-9):
            bad_r4 += 1
        if v2n * abs(om2) > v2.a_lat * (1 + 1e-9):
            bad_lat += 1
        if abs(v2n - v) > 4.0 * dt * (1 + 1e-9):
            bad_lin += 1
        if v2n > 4.5 * (1 + 1e-15) or v2n < 0:
            bad_v += 1
    check("R4 |dw| <= a_ang*dt always", bad_r4 == 0, f"{bad_r4}/{n}")
    check("lateral |v*omega| <= a_lat always", bad_lat == 0, f"{bad_lat}/{n}")
    check("R3 |dv| <= a_lin*dt always", bad_lin == 0, f"{bad_lin}/{n}")
    check("R1 0 <= v <= v_max always", bad_v == 0, f"{bad_v}/{n}")


def t_counterexample():
    """v=1.5 m/s, omega=3 rad/s violates both caps if clipped separately."""
    v, om = 1.5, 3.0
    lat = v * om
    check("counterexample: |v*omega| = 4.5 > a_lat = 2.0 (joint solve required)",
          lat > 2.0, f"{lat:.2f} m/s^2")
    # a robot at that state cannot satisfy the lateral bound, so the projection
    # must reduce one of them; check that the *plant* never emits such a state
    v2 = default_v2cfg()
    vn, _h, omn, _ = robot_step_project(
        np.array([v, 0.0]), 3.0, 0.0, om, v_max=4.5, omega_max=3.5, a_lin=4.0,
        a_ang=v2.a_ang, a_lat=v2.a_lat, dt=0.05, ang_on=True, lat_on=True)
    check("robot projection repairs an inadmissible (v, omega) state",
          vn * abs(omn) <= v2.a_lat * (1 + 1e-9),
          f"v={vn:.3f}, omega={omn:.3f}, |v*omega|={vn * abs(omn):.3f}")


def t_config_hash():
    a = default_v2cfg()
    b = default_v2cfg()
    c = default_v2cfg(d_ro_safe=2.5)
    check("config id is stable and content-addressed",
          a.config_id == b.config_id and a.config_id != c.config_id,
          f"{a.config_id} / {c.config_id}")


# ---------------------------------------------------------------------------
def _v1cfg():
    from scenarios import SCENARIOS
    return SCENARIOS["S3"]()["cfg"]


def main():
    print("V2 unit tests")
    t_safety_transcription()
    t_sheep_transcription()
    t_fleet_transcription()
    t_box_projection()
    t_robot_projection()
    t_counterexample()
    t_config_hash()
    print(f"\n{'ALL PASS' if not FAILED else 'FAILURES: ' + str(FAILED)}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    raise SystemExit(main())
