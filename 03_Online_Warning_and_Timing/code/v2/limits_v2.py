"""Joint limit enforcement for the V2 physical execution layer.

The manuscript's (V2) box is an **intersection** constraint, reproduced here
verbatim from the revision document:

    0 <= s_i^+ <= s_i,max
    |wrap(theta_i^+ - theta_i)| <= omega_s * dt
    ||v_i^+ - v_i||            <= a_s * dt          (velocity increment is a VECTOR)

Everything below projects a requested (turn, speed) pair onto that intersection
instead of clipping the scalar speed and then the heading independently.  For
the robot the same idea is applied to R1-R4 plus the lateral cap |v*omega|.

Two properties matter and are both enforced by construction:

1. **No-op exactness.**  If the request already satisfies every bound, the
   projection returns the request *bit-for-bit* (no arithmetic is performed on
   it).  This is what lets acceptance test A reproduce the V1 archive exactly
   when the new caps are switched off -- the disabled code path is not merely
   "numerically close", it is the V1 expression.

2. **Feasibility is proved, not assumed.**  For the robot, the existence of an
   admissible (v, omega) is guaranteed by the invariant

       v_prev * |omega_prev| <= a_lat

   which holds at t=0 (both zero) and is preserved by the projection.  Under it,

       omega_min = max(0, |omega_prev| - a_ang*dt)   (smallest reachable |omega|)
       v_lo      = max(0, v_prev - a_lin*dt)         (smallest reachable v)
       v_lo <= a_lat / omega_min                     (always, see report)

   so (v_lo, omega_min) is in the admissible set and the set is never empty.
   The projection below therefore never has to fall back to a violation; the
   defensive counter `LAT_UNRESOLVED` proves that in the acceptance run.

Sheep side, the same argument with the (V2) box: for a turn |dtheta| the
reachable speeds form

       c = s_old cos(dtheta),  D = (a_s dt)^2 - (s_old sin(dtheta))^2
       s in [c - sqrt(D), c + sqrt(D)]      (non-empty iff |dtheta| <= asin(A/s_old))

   and if the C1 lower bound s_min is not reachable at the requested turn, the
   turn is shrunk to the largest value that *is* compatible with reaching s_min
   (law of cosines: cos dtheta = (s_min^2 + s_old^2 - A^2) / (2 s_min s_old)).
   This is the "respect the intersection" requirement of the (V2) box; a
   sequential clip would instead have produced a spurious C1 violation.
"""
from __future__ import annotations

import math

import numpy as np

TWO_PI = 2.0 * np.pi


def _wrap(a):
    """Same expression as robots._wrap / simulation._wrap (bit-identical)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# counters
# ---------------------------------------------------------------------------
BIND_NAMES = (
    "bind_C1_low_sheep", "bind_C1_high_sheep", "bind_C2_sheep",
    "bind_C3_sheep", "bind_C3_turn_sheep", "bind_C3_speed_sheep",
    "bind_R1_robot", "bind_R2_robot", "bind_R3_robot", "bind_R4_robot",
    "bind_LAT_robot",
)
VIOL_NAMES = (
    "viol_C1_low", "viol_C1_high", "viol_C2", "viol_C3",
    "viol_R1", "viol_R2", "viol_R3", "viol_R4", "viol_LAT",
)
OTHER_NAMES = ("plant_infeasible", "LAT_UNRESOLVED", "LAT_JOINT_FALLBACK")


class Counters:
    """Small deterministic integer-counter bag (no numpy, easy to pickle)."""

    def __init__(self):
        self.d = {}

    def hit(self, name, n=1):
        self.d[name] = self.d.get(name, 0) + int(n)

    def get(self, name):
        return self.d.get(name, 0)

    def update(self, other):
        for k, v in other.d.items():
            self.d[k] = self.d.get(k, 0) + int(v)
        return self

    def to_dict(self):
        return dict(sorted(self.d.items()))

    @classmethod
    def from_dict(cls, d):
        c = cls()
        c.d = dict(d)
        return c


def empty_stats(opportunities=0):
    """All bind/viol counters present (so CSV columns never shift)."""
    c = Counters()
    for k in BIND_NAMES + VIOL_NAMES + OTHER_NAMES:
        c.d[k] = 0
    c.d["opp_sheep_steps"] = int(opportunities)
    c.d["opp_robot_steps"] = int(opportunities)
    return c


# ---------------------------------------------------------------------------
# sheep: the (V2) box
# ---------------------------------------------------------------------------
def project_sheep_increment(s_old, dth_des, s_des, *, dt, a_s, omega_s,
                            s_lo, s_hi, eps_v=1e-6, counters=None,
                            c3_on=True, bind_rtol=1e-12):
    """Project the requested heading increment / speed onto the (V2) box.

    Parameters
    ----------
    s_old : current speed (>= 0)
    dth_des : requested heading increment [rad] (already C2-clipped upstream)
    s_des : requested speed [m/s] (already C1-clipped upstream)
    a_s : C3 acceleration bound [m/s^2]
    omega_s : C2 turn-rate bound [rad/s]
    s_lo, s_hi : C1 band [m/s]
    eps_v : near-zero speed threshold; below it the heading is retained and the
        reachable speed set is the disc of radius a_s*dt around the origin.

    Returns
    -------
    (dth_out, s_out, info) where info is a small dict with keys
    ``c3``, ``c2``, ``c1_low``, ``plant_infeasible`` (booleans).
    Bit-exact no-op when the request is already admissible.
    """
    info = {"c3": False, "c2": False, "c1_low": False, "plant_infeasible": False}
    if not c3_on:
        return dth_des, s_des, info

    A = a_s * dt
    # ---- C2 (turn rate) ---------------------------------------------------
    dth = float(np.clip(dth_des, -omega_s * dt, omega_s * dt))
    if dth != dth_des:
        info["c2"] = True

    # ---- C3 + C1: joint projection ---------------------------------------
    if s_old <= eps_v:
        # Near-zero speed: |v| = s <= A in *any* direction (ball around origin),
        # so the requested direction is reachable and only the magnitude binds.
        # The heading state is retained (the direction is not re-derived).
        s_cap = min(s_hi, A)
        if s_lo > s_cap:
            info["plant_infeasible"] = True
            s_lo_eff = 0.0
        else:
            s_lo_eff = s_lo
        s_new = float(np.clip(s_des, s_lo_eff, s_cap))
        if s_new != s_des:
            info["c3"] = True
    else:
        ratio = A / s_old
        if ratio >= 1.0:
            dth_acc = math.pi                      # whole circle reachable
        else:
            dth_acc = math.asin(ratio)             # ball/chord intersection
        dth_try = float(np.clip(dth, -dth_acc, dth_acc))

        c = s_old * math.cos(dth_try)
        D = A * A - (s_old * math.sin(dth_try)) ** 2
        r = math.sqrt(max(D, 0.0))
        lo_s, hi_s = c - r, c + r
        s_lo_eff = max(s_lo, lo_s)
        s_hi_eff = min(s_hi, hi_s)

        if s_lo_eff > s_hi_eff:
            # The C1 lower bound is not reachable at this turn -> shrink the
            # turn to the largest one compatible with reaching s_lo.
            info["c1_low"] = True
            if abs(s_old - s_lo) <= A and s_lo > 0.0:
                cos_k = (s_lo * s_lo + s_old * s_old - A * A) / (2.0 * s_lo * s_old)
                dth_keep = math.acos(float(np.clip(cos_k, -1.0, 1.0)))
            else:
                dth_keep = 0.0
                info["plant_infeasible"] = True
            sgn = 1.0 if dth_try >= 0 else -1.0
            dth_try = sgn * min(abs(dth_try), dth_keep)
            c = s_old * math.cos(dth_try)
            D = A * A - (s_old * math.sin(dth_try)) ** 2
            r = math.sqrt(max(D, 0.0))
            lo_s, hi_s = c - r, c + r
            s_lo_eff = max(s_lo, lo_s)
            s_hi_eff = min(s_hi, hi_s)
            if s_lo_eff > s_hi_eff:
                # Genuinely infeasible: keep the C3 radius, relax C1 for this
                # step only (logged; counted by the caller).
                info["plant_infeasible"] = True
                s_lo_eff, s_hi_eff = min(s_hi, max(0.0, lo_s)), min(s_hi, hi_s)

        s_new = float(np.clip(s_des, s_lo_eff, s_hi_eff))
        if s_new != s_des or dth_try != dth:
            info["c3"] = True
            if s_new != s_des:
                info["c3_speed"] = True
            if dth_try != dth:
                info["c3_turn"] = True
        dth = dth_try

    if counters is not None:
        if info["c2"]:
            counters.hit("bind_C2_sheep")
        if info["c3"]:
            counters.hit("bind_C3_sheep")
            if info.get("c3_turn"):
                counters.hit("bind_C3_turn_sheep")
            if info.get("c3_speed"):
                counters.hit("bind_C3_speed_sheep")
        if info["c1_low"]:
            counters.hit("bind_C1_low_sheep")
        if info["plant_infeasible"]:
            counters.hit("plant_infeasible")

    # ---- bit-exact no-op guard -------------------------------------------
    if dth == dth_des and s_new == s_des:
        return dth_des, s_des, info
    return dth, s_new, info


# ---------------------------------------------------------------------------
# robot: R1-R4 + lateral acceleration
# ---------------------------------------------------------------------------
def robot_step_project(ci, v_old, head, omega_old, *, v_max, omega_max, a_lin,
                       a_ang, a_lat, dt, ang_on, lat_on, eps_v=1e-6,
                       counters=None):
    """One robot's joint-limited update for a commanded velocity ``ci``.

    Returns ``(v_new, head_new, omega_new, info)``.  When ``ang_on`` and
    ``lat_on`` are both False the arithmetic is exactly
    ``robots.RobotFleet.step``'s per-robot body (verified in
    ``v2/unit_tests_v2.py``).
    """
    info = {"R1": False, "R2": False, "R3": False, "R4": False, "LAT": False,
            "lat_joint": False, "unresolved": False}

    spd_des = min(float(np.linalg.norm(ci)), v_max)
    if spd_des > 1e-3:
        th_des = np.arctan2(ci[1], ci[0])
        err = _wrap(th_des - head)
        turn_des = float(np.clip(err, -omega_max * dt, omega_max * dt))
        if turn_des != err:
            info["R2"] = True
    else:
        th_des = math.nan
        err = 0.0
        turn_des = 0.0

    # ---- R4: angular acceleration, turn rate as a state -------------------
    omega_des = turn_des / dt
    if ang_on:
        w_lo = omega_old - a_ang * dt
        w_hi = omega_old + a_ang * dt
        w = float(np.clip(omega_des, w_lo, w_hi))
        if w != omega_des:
            info["R4"] = True
        if abs(w) > omega_max:
            w = float(np.clip(w, -omega_max, omega_max))
            info["R2"] = True
        omega = w
        turn = turn_des if not info["R4"] else w * dt
    else:
        omega = omega_des
        turn = turn_des

    head1 = head + turn
    if spd_des > 1e-3:
        align = float(np.cos(_wrap(th_des - head1)))
        vd = spd_des * max(0.0, align)
    else:
        vd = 0.0
    dv = float(np.clip(vd - v_old, -a_lin * dt, a_lin * dt))
    if dv != (vd - v_old):
        info["R3"] = True
    v = float(np.clip(v_old + dv, 0.0, v_max))
    if v != (v_old + dv):
        info["R1"] = True

    # ---- lateral acceleration |v * omega| <= a_lat ------------------------
    if lat_on and v * abs(omega) > a_lat * (1.0 + 1e-15):
        v_lo = max(0.0, v_old - a_lin * dt)
        v_hi = min(v_max, v_old + a_lin * dt)
        w_min = max(0.0, abs(omega_old) - a_ang * dt) if ang_on else 0.0
        v_cap = math.inf
        sgn = 1.0 if omega >= 0 else -1.0
        for _ in range(3):
            if v <= 0.0:
                omega = 0.0
                break
            w_new = a_lat / v
            if w_new >= w_min:
                omega = sgn * w_new
            else:
                # R4 forbids slowing the turn enough: trade speed as well.
                omega = sgn * w_min
                if w_min > 0.0:
                    v_cap = min(v_cap, a_lat / w_min)
                info["lat_joint"] = True
            turn = omega * dt
            head1 = head + turn
            if spd_des > 1e-3:
                align = float(np.cos(_wrap(th_des - head1)))
                vd = spd_des * max(0.0, align)
            else:
                vd = 0.0
            dv = float(np.clip(vd - v_old, -a_lin * dt, a_lin * dt))
            v = min(float(np.clip(v_old + dv, 0.0, v_max)), v_cap)
            v = max(v, min(v_lo, v_cap))
            if v * abs(omega) <= a_lat * (1.0 + 1e-15):
                break
            v = min(v, a_lat / abs(omega)) if omega != 0.0 else v
        info["LAT"] = True
        if v * abs(omega) > a_lat * (1.0 + 1e-9):
            info["unresolved"] = True
        if info["lat_joint"]:
            # (v_lo, w_min) is feasible under the invariant; reaching here means
            # numerical stress only -- count it so the claim is testable.
            pass

    if counters is not None:
        for k, name in (("R1", "bind_R1_robot"), ("R2", "bind_R2_robot"),
                        ("R3", "bind_R3_robot"), ("R4", "bind_R4_robot"),
                        ("LAT", "bind_LAT_robot")):
            if info[k]:
                counters.hit(name)
        if info["lat_joint"]:
            counters.hit("LAT_JOINT_FALLBACK")
        if info["unresolved"]:
            counters.hit("LAT_UNRESOLVED")
    return v, head1, omega, info
