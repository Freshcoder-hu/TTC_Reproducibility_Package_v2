"""Command-level spacing filter for V2: soft target + hard-geometry guard.

This is a **parameterised transcription** of ``robots.apply_robot_safety``.
The transcription exists for one reason: V1's filter reads its targets from
``cfg.d_rr_safe`` / ``cfg.d_ro_safe`` and has no notion of a body radius, and
the V1 file must not be edited.  Passing a V2 ``SimCfg`` through the original
function would already give the new *soft* targets, but the hard-geometry guard
is new code, so the function is reproduced here and verified bit-identical to
V1's when the guard is off (``v2/unit_tests_v2.py``).

ROLE SPLIT (documented in the report)
-------------------------------------
* **soft target** (``d_rr_safe`` = 1.20 m, ``d_ro_safe`` = 2.00 m): a command
  shaping term, exactly like V1's, at a *decided margin* above the geometric
  limit.
* **hard limit** (0.90 / 1.65 / 0.80 m): first a cheap analytic guard here
  (cancel the closing component once inside the hard band), and then the real
  enforcement in :mod:`v2.feasibility_v2`, which predicts the motion over the
  execution interval and brakes/escapes when a hard breach is projected.
* Nothing here can *guarantee* non-penetration -- with ``a_lin`` = 4 m/s^2 the
  per-tick velocity authority is only 0.4 m/s.  The report states this
  explicitly instead of claiming the hard constraint is met by construction.
"""
from __future__ import annotations

import numpy as np

from robots import _norm1


def apply_robot_safety_v2(cmd, robot_pos, obs_pos, cfg, hard=None, guard=True,
                          counters=None):
    """V1's command filter, with the V2 soft targets and a hard-geometry guard.

    Parameters
    ----------
    cmd : (M,2) commanded velocity
    robot_pos : (M,2)
    obs_pos : (K,2) or None
    cfg : SimCfg carrying ``d_rr_safe``, ``d_ro_safe``, ``v_max``, ``dt``
    hard : dict with ``rr`` / ``ro`` hard centre distances (None = no guard)
    guard : set False to get the pure V1 behaviour (used by acceptance test A)
    """
    cmd = cmd.copy()
    M = len(robot_pos)
    dt = cfg.dt

    # ---- soft robot-robot projection on the predicted next position (V1) ----
    for _ in range(4):
        p_next = robot_pos + cmd * dt
        for i in range(M):
            for j in range(i + 1, M):
                d = p_next[i] - p_next[j]
                dist = float(np.linalg.norm(d))
                if dist < cfg.d_rr_safe:
                    if dist < 1e-6:
                        d = np.array([1.0, 0.0])
                        dist = 1.0
                    corr = _norm1(d) * ((cfg.d_rr_safe - dist) / (2.0 * dt))
                    cmd[i] += corr
                    cmd[j] -= corr

    # ---- cancel the closing component inside the soft target (V1) ----
    for i in range(M):
        for j in range(i + 1, M):
            n = robot_pos[i] - robot_pos[j]
            dist = float(np.linalg.norm(n))
            if dist < 1e-6:
                continue
            n = n / dist
            closing = float((cmd[i] - cmd[j]) @ n)
            if dist < cfg.d_rr_safe and closing < 0:
                corr = 0.5 * closing * n
                cmd[i] -= corr
                cmd[j] += corr

    # ---- robot-obstacle soft push (V1) ----
    if obs_pos is not None and len(obs_pos):
        for i in range(M):
            d = robot_pos[i] - obs_pos
            dist = np.linalg.norm(d, axis=1)
            k = int(np.argmin(dist))
            if dist[k] < cfg.d_ro_safe:
                cmd[i] += _norm1(d[k]) * (cfg.d_ro_safe - dist[k]) * 2.0

    # ---- NEW: hard-geometry guard ----------------------------------------
    if guard and hard is not None:
        # robot-robot: below the hard centre distance, forbid any further
        # approach at all (the soft term above is a *target*, not a barrier).
        for i in range(M):
            for j in range(i + 1, M):
                n = robot_pos[i] - robot_pos[j]
                dist = float(np.linalg.norm(n))
                if dist < 1e-6 or dist >= hard["rr"]:
                    continue
                n = n / dist
                closing = float((cmd[i] - cmd[j]) @ n)
                if closing < 0.0:
                    corr = 0.5 * closing * n
                    cmd[i] -= corr
                    cmd[j] += corr
                    if counters is not None:
                        counters.hit("guard_rr")
        # robot-obstacle: same, inside the hard band the robot may not close.
        if obs_pos is not None and len(obs_pos):
            for i in range(M):
                d = robot_pos[i] - obs_pos
                dist = np.linalg.norm(d, axis=1)
                k = int(np.argmin(dist))
                if dist[k] >= hard["ro"]:
                    continue
                n = _norm1(d[k])
                closing = float(cmd[i] @ n)
                if closing < 0.0:
                    cmd[i] -= closing * n
                    if counters is not None:
                        counters.hit("guard_ro")

    # ---- speed cap (V1) ----
    n = np.linalg.norm(cmd, axis=1)
    over = n > cfg.v_max
    if over.any():
        cmd[over] *= (cfg.v_max / n[over])[:, None]
    return cmd
