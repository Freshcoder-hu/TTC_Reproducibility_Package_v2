"""Rollout ablation rungs -- mechanically derived, no new control law is invented.

Ladder (top rung = the paper's final "Ours"):

  R0  base        proposed.Proposed              four-state FSM only
                                                  (this is the OLD `Ours`; ablation-only)
  R1  w/o Rollout proposed_event.EventProposed   event coordination, no rollout
  R2  w/o event   RolloutBare (this file)        rollout planner on the bare four-state FSM
  R3  full        proposed_rollout.RolloutProposed  event coordination + rollout  = "Ours"

`RolloutBare` is a textual composition of `RolloutProposed.step`: the *only*
difference is that the nominal command comes from `Proposed.step` instead of
`EventProposed.step`.  `test_rollout_ablation.py` enforces that the two bodies
are byte-identical apart from that one call, so R2 cannot silently drift away
from R3.
"""
import numpy as np

from proposed import Proposed
from proposed_rollout import RolloutProposed
from robots import flock_geom, drive_command, _norm1


class RolloutBare(RolloutProposed):
    """Rollout planning on the bare four-state controller (no event coordination)."""

    name = 'Ours_rollout_wo_event'

    def step(self, view, cfg):
        nominal, tag = Proposed.step(self, view, cfg)
        if self._heading is None:
            s, _, _ = view.path.closest(view.centroid)
            d = view.path.tangent_at(s)
            self._heading = np.full(len(view.robot_pos), np.arctan2(d[1], d[0]))
        if tag in ('pre', 'divert', 'recover') and view.obs_active_n:
            if view.t - self._plan_t >= self.plan_period - 1e-7:
                s, _, _ = view.path.closest(view.centroid)
                bound = min(3.6, .6 * view.path.halfwidth_at(s))
                offsets = np.unique(np.clip([0., -1.35, 1.35, -2.4, 2.4, -3.6, 3.6,
                                             self._offset], -bound, bound))
                outcomes = self._predict(view, cfg, offsets)
                cost = (100 * outcomes[:, 0] + 10 * outcomes[:, 1] + 30 * outcomes[:, 2]
                        + .2 * outcomes[:, 3] + 2 * outcomes[:, 4] - .05 * outcomes[:, 5]
                        + .02 * (offsets - self._offset) ** 2)
                best = int(np.argmin(cost))
                self._offset = float(offsets[best])
                self._plan_t = float(view.t)
                self.plans.append(dict(t=float(view.t), offset=self._offset,
                                       offsets=offsets.tolist(), outcomes=outcomes.tolist(), cost=cost.tolist()))
            c, radius = flock_geom(view)
            s, _, _ = view.path.closest(c)
            ahead = 6. if abs(self._offset) < 1e-8 else 4.
            tangent = view.path.tangent_at(s + ahead)
            normal = np.array([-tangent[1], tangent[0]])
            target = view.path.point_at(s + ahead) + normal * self._offset
            direction = _norm1(target - c)
            nominal, _, _ = drive_command(view, cfg, c, radius, direction, tag)
        else:
            self._offset = 0.
        self._previous_pos = view.robot_pos.copy()
        self._previous_t = float(view.t)
        return nominal, tag
