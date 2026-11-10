"""Shared corrections; never enable them for only one comparison method."""
import numpy as np
import obstacle
import simulation
import ttc

_OriginalObstacle = obstacle.Obstacle
_original_ttc = ttc._ttc_pair_analytic


class OneShotObstacle(_OriginalObstacle):
    """A released obstacle expires once, rather than respawning every life seconds."""
    def update(self, t):
        if (self.trigger is None and self.t0_active is None
                and t >= self.t_spawn):
            self._activate(t)
        if self.active:
            self.pos = self.start + self.vel * (t - self.t0_active)
            if self.life is not None and t - self.t0_active >= self.life:
                self.active = False


def contact_ttc(r, v_rel, r_sum):
    """Return zero for current overlap, including stationary/receding overlap."""
    if float(r @ r) <= r_sum * r_sum:
        return 0.0
    return _original_ttc(r, v_rel, r_sum)


def install(environment=True, contact=True):
    simulation.Obstacle = OneShotObstacle if environment else _OriginalObstacle
    ttc._ttc_pair_analytic = contact_ttc if contact else _original_ttc
