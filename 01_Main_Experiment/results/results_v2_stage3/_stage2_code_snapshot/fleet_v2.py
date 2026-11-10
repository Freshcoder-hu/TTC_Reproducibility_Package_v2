"""V2 robot fleet: the V1 unicycle, plus turn rate as a state and the joint caps.

``RobotFleetV2.step`` executes, in this order:

1. the command-level filter (soft target + hard-geometry guard),
2. the feasibility layer (braking supervisor + hard-separation response),
3. the joint-limit projection of R1-R4 and |v*omega| <= a_lat, per robot,
4. the position update ``p <- p + u(theta) * v * dt``.

In V1-compatibility mode (``enforce_r4=False``, ``enforce_lat=False``, no
feasibility layer, guard off) steps 1-3 reduce *bit-for-bit* to
``robots.RobotFleet.step``; that is what acceptance test A checks.
"""
from __future__ import annotations

import numpy as np

from robots import _u
from v2.limits_v2 import robot_step_project
from v2.robot_safety_v2 import apply_robot_safety_v2


class RobotFleetV2:
    def __init__(self, pos, head=None, cfg=None):
        pos = np.asarray(pos, dtype=float)
        self.pos = pos.copy()
        M = pos.shape[0]
        if head is None:
            head = np.zeros(M)
        self.head = np.asarray(head, dtype=float)
        self.speed = np.zeros(M)
        self.omega = np.zeros(M)          # R4 needs the turn rate as a state

    # ------------------------------------------------------------------
    def state(self, t, obs_pos, obs_vel, sheep_pos, sheep_vel, obs_ids=None):
        n_obs = 0 if obs_pos is None else len(obs_pos)
        return dict(t=float(t), pos=self.pos, head=self.head,
                    speed=self.speed, omega=self.omega,
                    obs_pos=obs_pos if obs_pos is not None else np.zeros((0, 2)),
                    obs_vel=obs_vel if obs_vel is not None else np.zeros((0, 2)),
                    obs_ids=(np.arange(n_obs) if obs_ids is None
                             else obs_ids),
                    sheep_pos=sheep_pos, sheep_vel=sheep_vel)

    # ------------------------------------------------------------------
    def step(self, cmd, cfg, v2cfg, obs_pos=None, obs_vel=None,
             sheep_pos=None, sheep_vel=None, t=0.0, ctrl_tick=True,
             feas=None, counters=None, obs_ids=None):
        hard = v2cfg.hard_contacts()
        cmd = apply_robot_safety_v2(
            cmd, self.pos, obs_pos, cfg, hard=hard,
            guard=v2cfg.enforce_hard_separation, counters=counters)

        if feas is not None and v2cfg.feas_enabled and sheep_pos is not None:
            st = self.state(t, obs_pos, obs_vel, sheep_pos, sheep_vel,
                            obs_ids=obs_ids)
            cmd = feas.filter(cmd, st, ctrl_tick=ctrl_tick)
            if ctrl_tick:
                feas.push_state(st)

        dt = cfg.dt
        M = len(self.pos)
        if counters is not None:
            counters.hit("opp_robot_steps", M)
        for i in range(M):
            v, head, omega, _info = robot_step_project(
                cmd[i], self.speed[i], self.head[i], self.omega[i],
                v_max=cfg.v_max, omega_max=cfg.omega_max, a_lin=cfg.a_lin,
                a_ang=v2cfg.a_ang, a_lat=v2cfg.a_lat, dt=dt,
                ang_on=v2cfg.enforce_r4, lat_on=v2cfg.enforce_lat,
                eps_v=v2cfg.eps_v, counters=counters)
            self.pos[i] += v * _u(head) * dt
            self.head[i] = head
            self.speed[i] = v
            self.omega[i] = omega
        return self.pos
