"""Explicit ablations of the original controller. No scenario/seed branches."""
import numpy as np
from proposed import Proposed
from robots import flock_geom, task_direction, drive_command, divert_target
from ttc import ttc_matrix


class HandoverProposed(Proposed):
    name = 'Ours_handover'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._role_threat = None

    def _select_primary(self, view, cfg, threat, pair):
        # The sheep index, contact time and obstacle must belong to the same pair.
        if threat is not None and len(view.sheep_pos):
            T, _, _ = ttc_matrix(view.sheep_pos, view.sheep_vel,
                                np.array([threat.pos]), np.array([threat.vel]),
                                cfg.r_sheep + threat.radius)
            if np.isfinite(T).any():
                i = int(np.argmin(T[:, 0]))
                pair = (i, 0)
                self._ttc_last = float(T[i, 0])
        if threat is not self._role_threat:
            self._role_threat = threat
            previous = list(self.primary_idx)
            # Re-evaluate roles on a new event; preserve counts and transitions
            # only for robots whose membership actually changes.
            self.primary_idx = []
            count = self.role_switch_count
            trans = dict(self._trans)
            chosen = super()._select_primary(view, cfg, threat, pair)
            changed = set(previous) ^ set(chosen)
            self.role_switch_count = count + int(bool(changed))
            self._trans = trans
            for j in changed:
                self._trans[j] = float(view.t)
            return chosen
        return super()._select_primary(view, cfg, threat, pair)


class CoordinatedProposed(HandoverProposed):
    """Full formation shares the urgent direction; selective PRE is retained."""
    name = 'Ours_coordinated'

    def step(self, view, cfg):
        cmd, tag = super().step(view, cfg)
        threat = getattr(self, '_threat_obj', None)
        if tag == 'divert' and threat is not None and threat.is_alive():
            c, radius = flock_geom(view)
            direction, _, _ = divert_target(
                view, cfg, c, radius, threat,
                shift=max(self.divert_shift, threat.radius+cfg.r_sheep+1),
                max_lateral_ratio=self.divert_max_lateral_ratio,
                side=self.event_side)
            cmd, _, _ = drive_command(view, cfg, c, radius, direction, tag,
                                      cmd_scale=self.divert_cmd_scale)
        return cmd, tag
