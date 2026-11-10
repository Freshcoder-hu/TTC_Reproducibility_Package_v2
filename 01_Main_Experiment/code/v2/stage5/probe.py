"""Record, inside the run, the TTC the controller could actually compute.

WHY THIS EXISTS (and why masking the log is not enough)
------------------------------------------------------
``v2.sim_v2`` writes the log row for a control tick **after** it has advanced
the robots, the animals and the obstacles by one integration step (the frozen
V1 layout: ``o.update(t + dt)`` then ``view.refresh`` then the log block).  So
``logs[i]["ttc_each"]`` is the truth TTC of the state at ``t + dt``, not at the
control instant ``t`` at which the controller's command was computed.

``v2.stage5.gated_probe`` measures the consequence: at the publish instant the
TTC of the visible obstacle and the truth TTC at the *same* instant agree to the
last bit (published track position == truth position, velocity too), but the
logged series differs by up to **5.83 s** on V2S3/1000, because near the closest
approach d(TTC)/dt is large and a 0.05 s state offset is not a 0.05 s TTC
offset.  A risk *onset* (TTC <= T_PRE) read from the logged series can therefore
be off by one tick, and a preemption claim built on it would be built on a
series the controller never saw.

This module installs a ``Sensorium`` subclass for Stage-5 runs only.  It adds
one row field to ``res["sense_log"][i]``:

    ``ttc_ctrl``  {obstacle index -> TTC}  -- min over the **published** animal
                  channel of the TTC against the **published** track of every
                  obstacle in the controller's view, computed at the publish
                  instant with ``ttc.ttc_matrix`` (the same call ``sim_v2``
                  uses, horizon 12 s, r_sum = r_sheep + r_obs).
                  An obstacle that is not in the published view has **no key**
                  (unknown, not safe).
    ``n_sheep_ctrl``  size of the published animal channel on that tick.

The probe is strictly read-only: it adds no arithmetic to the state, consumes no
RNG and runs outside the controller-timing region, so the trajectory and every
existing metric are unchanged (acceptance A re-runs a Stage-4 cell through this
path and demands bit equality of all Stage-4 columns).
"""
from __future__ import annotations

import contextlib

import numpy as np

import v2.sim_v2 as sim_v2
from ttc import ttc_matrix
from v2.sensing_v2 import Sensorium


class ProbeSensorium(Sensorium):
    """Sensorium + a per-tick record of the controller-side TTC."""

    def __init__(self, v2cfg, cfg, seed=0):
        super().__init__(v2cfg, cfg, seed)
        self._r_sum = float(cfg.r_sheep + cfg.r_obs)
        self._horizon = 12.0
        self.n_ttc_calls = 0

    # -- the only addition ---------------------------------------------
    def _ttc_ctrl(self, v):
        act = list(getattr(v, "obstacles_active", ()) or ())
        if not act or not len(v.sheep_pos):
            return {}
        sp = np.asarray(v.sheep_pos, dtype=float).reshape(-1, 2)
        sv = np.asarray(v.sheep_vel, dtype=float).reshape(-1, 2)
        op = np.asarray(v.obs_pos, dtype=float).reshape(-1, 2)
        ov = np.asarray(v.obs_vel, dtype=float).reshape(-1, 2)
        T, _, _ = ttc_matrix(sp, sv, op, ov, self._r_sum,
                             horizon=self._horizon)
        self.n_ttc_calls += 1
        gmin = np.min(T, axis=0)
        return {int(o.index): float(gmin[j]) for j, o in enumerate(act)}

    def publish(self, view_truth, t):
        v = super().publish(view_truth, t)
        if self._ctrl_tick and self.log and \
                abs(float(self.log[-1]["t"]) - float(t)) <= 1e-12:
            self.log[-1]["ttc_ctrl"] = self._ttc_ctrl(v)
            self.log[-1]["n_sheep_ctrl"] = int(len(v.sheep_pos))
        return v


def _build(v2cfg, cfg, seed):
    mode = str(getattr(v2cfg, "sense_mode", "exact"))
    if mode == "exact":
        return None
    if mode != "range":
        raise ValueError(f"unknown sense_mode {mode!r} (exact|range)")
    return ProbeSensorium(v2cfg, cfg, seed)


@contextlib.contextmanager
def installed(enabled=True):
    """Patch ``v2.sim_v2.build_sensing`` for the duration of one run."""
    if not enabled:
        yield
        return
    saved = sim_v2.build_sensing
    sim_v2.build_sensing = _build
    try:
        yield
    finally:
        sim_v2.build_sensing = saved
