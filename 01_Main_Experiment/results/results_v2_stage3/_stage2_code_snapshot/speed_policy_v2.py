"""Layered speed policy -- a **controller-layer rule**, not a physical limit.

THE RULE
--------
``v_max`` = 4.5 m/s applies to straight-line repositioning only.  While
pre-positioning or diverting, the commanded speed is bounded by

    v  <=  sqrt(a_lat * r_req),          r_req = (L^2 + h^2) / (2 h)

where ``h`` is the **commanded lateral offset** (the distance the flock is
still being asked to move sideways, measured in the path frame) and ``L`` the
along-path distance over which that offset is commanded.  ``r_req`` is the
radius of the circular arc through the displacement ``(L, h)``:

    tan(psi/2) = h/L,  chord = sqrt(L^2+h^2),  r = chord / (2 sin(psi/2))
               = (L^2 + h^2) / (2h)

For the V1 DIVERT geometry (``L = 4 m`` lookahead, ``h = 3 m`` shift) this gives
``r_req = 25/6 = 4.167 m`` and ``v <= sqrt(2*4.167) = 2.887 m/s`` -- the Stage-1
report's number, now *derived* instead of hard-coded.  For the rollout-based
DIVERT (offset 3.6 m at 4 m lookahead) it gives 4.02 m and 2.836 m/s.

The bound is *self-releasing*: as the flock actually moves sideways, ``h``
shrinks, ``r_req`` grows and the allowed speed rises back toward ``v_max``.  The
minimum possible bound is ``r_req = L`` (at ``h = L``) i.e. 2.83 m/s at
``L = 4`` -- so the rule can never deadlock a manoeuvre.

WHY THIS IS A CONTROLLER RULE
-----------------------------
The physical limit is the lateral acceleration ``|v*omega| <= a_lat``, already
enforced by Stage 1 (R4/LAT).  A 4.5 m/s robot simply reaches 10.1 m minimum
turn radius, so the *controller* was asking for a manoeuvre the plant cannot
fly.  Reducing the commanded speed is the controller's job; ``a_lat`` itself is
author-fixed and must not be relaxed (Stage-1 report 9.3).

WHAT IS LOGGED
--------------
Every control tick the policy appends ``(t, mode, L, h, r_req, v_allow,
mean|cmd| before, mean|cmd| after, n_clamped)`` to its log, so "the bound
actually applied" is auditable per tick and not just reported as a mean.
"""
from __future__ import annotations

import math

import numpy as np


def required_radius(lookahead, lateral, h_eps=0.05):
    """Arc radius through a displacement (``lookahead`` along, ``lateral`` across)."""
    h = abs(float(lateral))
    if h <= h_eps:
        return math.inf                      # straight line: no turn required
    return (lookahead * lookahead + h * h) / (2.0 * h)


def speed_bound(a_lat, v_max, r_req):
    """``min(v_max, sqrt(a_lat * r_req))``."""
    if not math.isfinite(r_req):
        return float(v_max)
    return float(min(v_max, math.sqrt(max(a_lat * r_req, 0.0))))


def apply_speed_policy(cmd, v_allow):
    """Scale the commanded velocity field so no robot exceeds ``v_allow``."""
    cmd = np.asarray(cmd, dtype=float)
    n = np.linalg.norm(cmd, axis=1)
    if v_allow >= np.inf:
        return cmd.copy(), 0, 0.0
    over = n > v_allow
    out = cmd.copy()
    if over.any():
        out[over] *= (v_allow / n[over])[:, None]
    mean_before = float(n.mean()) if len(n) else 0.0
    mean_after = float(np.linalg.norm(out, axis=1).mean()) if len(out) else 0.0
    return out, int(over.sum()), (mean_before - mean_after)


class SpeedPolicy:
    """Stateful wrapper: decides the bound from the declared manoeuvre."""

    def __init__(self, v2cfg):
        self.v2cfg = v2cfg
        self.log = []
        self.n_ticks = 0
        self.n_bound_ticks = 0
        self.n_clamped_robots = 0
        self.r_req_sum = 0.0
        self.v_allow_sum = 0.0
        self.speed_loss_sum = 0.0
        self.v_allow_min = math.inf
        self.r_req_min = math.inf

    def active(self, mode):
        return (self.v2cfg.speed_policy == "manoeuvre"
                and mode in tuple(self.v2cfg.speed_policy_modes))

    def limit(self, cmd, guide, t=0.0, mode="normal"):
        """Apply the policy; ``guide`` is ``dict(ahead, lateral)`` or ``None``.

        ``guide`` must be the *commanded* manoeuvre geometry (lookahead and
        lateral offset), taken from the controller.  If the controller does not
        declare one, the policy reports ``applied=False`` and leaves the
        command untouched -- it never guesses a geometry.
        """
        info = dict(t=float(t), mode=mode, applied=False, ahead=np.nan,
                    lateral=np.nan, r_req=np.inf, v_allow=np.inf,
                    n_clamped=0, speed_loss=0.0)
        if not self.active(mode) or guide is None:
            self.log.append(info)
            return cmd, info
        L = float(guide.get("ahead", np.nan))
        h = float(guide.get("lateral", np.nan))
        if not np.isfinite(L) or not np.isfinite(h):
            self.log.append(info)
            return cmd, info
        r_req = required_radius(L, h, self.v2cfg.speed_policy_h_eps)
        v_allow = speed_bound(self.v2cfg.a_lat, self.v2cfg.v_max, r_req)
        out, n_cl, loss = apply_speed_policy(cmd, v_allow)
        info.update(applied=True, ahead=L, lateral=h, r_req=r_req,
                    v_allow=v_allow, n_clamped=n_cl, speed_loss=loss)
        self.log.append(info)
        self.n_ticks += 1
        self.n_clamped_robots += n_cl
        self.speed_loss_sum += loss
        if math.isfinite(r_req):
            self.n_bound_ticks += 1
            self.r_req_sum += r_req
            self.v_allow_sum += v_allow
            self.r_req_min = min(self.r_req_min, r_req)
            self.v_allow_min = min(self.v_allow_min, v_allow)
        return out, info

    def summary(self):
        n = max(self.n_bound_ticks, 1)
        return dict(
            speed_policy=self.v2cfg.speed_policy,
            sp_n_ticks=self.n_ticks,
            sp_n_bounded_ticks=self.n_bound_ticks,
            sp_n_clamped_robots=self.n_clamped_robots,
            sp_r_req_mean=(self.r_req_sum / n) if self.n_bound_ticks else np.nan,
            sp_r_req_min=self.r_req_min if self.n_bound_ticks else np.nan,
            sp_v_allow_mean=(self.v_allow_sum / n) if self.n_bound_ticks
            else np.nan,
            sp_v_allow_min=self.v_allow_min if self.n_bound_ticks else np.nan,
            sp_speed_loss_sum=self.speed_loss_sum,
        )
