"""On-board obstacle-detection gate (port of the v4 lineage gate, opt-in).

WHAT THIS IS
------------
The frozen simulator (`simulation.View.refresh`) makes every *released* obstacle
visible to the controller at **any distance**: `self.obstacles_active` becomes
the full list of live obstacles.  This module adds an optional, explicitly
opt-in detection gate: an obstacle becomes **known** to the controller only when
at least one robot of the fleet is within ``R`` of it (``--sensing-radius R``).

Ported from an earlier internal tree (``ttc_shepherding_final_v4``, not shipped)
(read only; that tree is not modified).  The v4 gate used the *flock* sensor
cone, i.e. ``known = (dist <= R) & (angle <= cfg.phi_sense)``.  The 30 m series
in this code base uses the **range-only** reading of the same rule, because

  * the task specification defines the gate as "at least one robot is within
    R of it" (range only), and
  * with the FOV term active the gate is **not** inert at R -> infinity
    (a cone still hides obstacles), so the mandatory inertness test
    ("R = 10000 m must reproduce the full-visibility baseline bit-exactly")
    would be unsatisfiable.

The v4 cone semantics remain available through ``--gate-fov`` for sensitivity
checks; ``use_fov`` is a keyword of :func:`install` and is recorded in every
run's manifest.  Default is range-only.

SEMANTICS (exactly)
-------------------
An obstacle ``o`` is *known* at control cycle ``t`` iff

    min_j ||p_o - p_j|| <= R          (j over all robots; p from `view.robot_pos`)

This is memoryless: it is re-evaluated from the current geometry at every
``View.refresh`` (twice per integration step in ``run_scene``).  The controller
then sees only the *known* obstacles -- position, velocity and radius are those
of the real obstacle objects -- and nothing else changes:

  * the evaluation side (collision detection, ``dmin``, TTC logs, termination)
    keeps using the **full truth**.  The gated fields on the view are restored
    the instant ``step()`` returns (``try/finally``), so robot-obstacle safety
    in ``robots.step``, sheep dynamics, metrics and termination are untouched;
  * no RNG is drawn anywhere in this module;
  * when the known subset is all live obstacles (in particular when
    ``R -> infinity``), ``step()`` is forwarded **without touching the view at
    all** -- this is what makes the port bit-exactly inert.

Explicitly OUT of scope (as in v4): sensing noise, latency, missed detections,
persistent tracks / memory.  The gate is pure geometry.
"""
import numpy as np

import simulation

__all__ = ["GatedView", "SensingGatedController", "install", "uninstall",
           "gate_config", "known_mask_for"]

# ---------------------------------------------------------------------------
# process-local gate configuration (set by install())
# ---------------------------------------------------------------------------
_RADIUS = None          # float metres, or None -> gate not installed
_USE_FOV = False        # v4 cone semantics (range AND bearing)
_RECORDER = None        # optional callable, called from GatedView.refresh
_ORIGINAL_VIEW = simulation.View     # pristine class, restored by uninstall()


def gate_config():
    """Return the currently installed gate as ``(radius, use_fov)``."""
    return _RADIUS, bool(_USE_FOV)


def known_mask_for(robot_pos, robot_head, obs_pos, radius, phi_sense=None):
    """Boolean mask: is each obstacle known to the fleet?

    Parameters
    ----------
    robot_pos : (M, 2) array
    robot_head : (M,) array of headings [rad] -- used only when ``phi_sense``
        is not None (v4 cone semantics).
    obs_pos : (K, 2) array
    radius : float  detection radius R [m]
    phi_sense : float or None  half-angle; None = range-only (default here).

    Returns
    -------
    (K,) bool array, plus ``(K,)`` array of the nearest robot distance
    (returned so the probe/telemetry can report the detection geometry).
    """
    obs_pos = np.asarray(obs_pos, dtype=float).reshape(-1, 2)
    robot_pos = np.asarray(robot_pos, dtype=float).reshape(-1, 2)
    K = len(obs_pos)
    if K == 0:
        return np.zeros(0, dtype=bool), np.zeros(0)
    if len(robot_pos) == 0:
        return np.zeros(K, dtype=bool), np.full(K, np.inf)
    d = obs_pos[:, None, :] - robot_pos[None, :, :]          # (K, M, 2)
    dist = np.sqrt((d * d).sum(axis=2))                      # (K, M)
    dmin = dist.min(axis=1)
    if phi_sense is None:
        return dmin <= radius, dmin
    head = np.stack([np.cos(robot_head), np.sin(robot_head)], axis=1)
    cosang = (d * head[None, :, :]).sum(axis=2) / np.maximum(dist, 1e-6)
    ang = np.arccos(np.clip(cosang, -1.0, 1.0))
    inside = (dist <= radius) & (ang <= float(phi_sense))
    return inside.any(axis=1), dmin


# ---------------------------------------------------------------------------
# View subclass: adds known_mask, changes nothing the simulation consumes
# ---------------------------------------------------------------------------
class GatedView(simulation.View):
    """`simulation.View` + a read-only ``known_mask`` attribute.

    ``refresh`` delegates to the frozen implementation first, then computes the
    gate mask on the **full** obstacle list (so the per-obstacle index k stays
    stable for the whole run even if obstacle lifecycles interleave, cf. S3)
    and stores the alive-subset mask aligned with ``view.obstacles_active``.
    No field used by the simulation or by the metrics is modified here.
    """

    known_mask = None            # (obs_active_n,) bool, aligned to obstacles_active

    def refresh(self, sheep, robots, cfg, obstacles):
        super().refresh(sheep, robots, cfg, obstacles)
        radius, use_fov = _RADIUS, _USE_FOV
        if radius is None:
            self.known_mask = None
            return
        n_all = len(obstacles)
        if n_all:
            pos_all = np.array([o.pos for o in obstacles], dtype=float) \
                .reshape(-1, 2)
            alive = np.array([o.is_alive() for o in obstacles], dtype=bool)
            known_all, dmin_all = known_mask_for(
                self.robot_pos, robots.head, pos_all, radius,
                cfg.phi_sense if use_fov else None)
        else:
            alive = np.zeros(0, dtype=bool)
            known_all = np.zeros(0, dtype=bool)
            dmin_all = np.zeros(0)
        self.known_mask = known_all[alive]
        rec = _RECORDER
        if rec is not None:
            rec(float(getattr(self, "t", 0.0)), alive, known_all, dmin_all,
                self.robot_pos)


class SensingGatedController:
    """Wrap a controller: during ``step()`` the view exposes known obstacles only.

    If every live obstacle is known (in particular when the gate radius is very
    large) the call is forwarded untouched -- bit-exact inertness.
    """

    def __init__(self, inner):
        self.inner = inner
        self.name = getattr(inner, "name", type(inner).__name__)

    def __getattr__(self, k):
        if k == "inner":                 # guard against half-initialised use
            raise AttributeError(k)
        return getattr(self.inner, k)

    def step(self, view, cfg):
        known = getattr(view, "known_mask", None)
        if known is None or known.all():
            return self.inner.step(view, cfg)
        idx = np.where(known)[0]
        obs_all = view.obstacles_active
        pos_all, vel_all, n_all = view.obs_pos, view.obs_vel, view.obs_active_n
        keep = [obs_all[i] for i in idx]
        try:
            view.obstacles_active = keep
            view.obs_active_n = len(keep)
            view.obs_pos = pos_all[idx] if len(idx) else np.zeros((0, 2))
            view.obs_vel = vel_all[idx] if len(idx) else np.zeros((0, 2))
            return self.inner.step(view, cfg)
        finally:
            view.obstacles_active = obs_all
            view.obs_active_n = n_all
            view.obs_pos = pos_all
            view.obs_vel = vel_all


# ---------------------------------------------------------------------------
# installation
# ---------------------------------------------------------------------------
def install(radius=None, use_fov=False, recorder=None):
    """Install (or clear) the gate in the **current process**.

    Must be called in every worker process (run_scene resolves ``View`` through
    the ``simulation`` module global at call time).  Idempotent.

    ``radius is None`` -> remove the gate (pristine simulator).
    """
    global _RADIUS, _USE_FOV, _RECORDER
    _RADIUS = None if radius is None else float(radius)
    _USE_FOV = bool(use_fov)
    _RECORDER = recorder
    if _RADIUS is None:
        simulation.View = _ORIGINAL_VIEW
        return None
    simulation.View = GatedView
    return gate_config()


def uninstall():
    install(None)


def wrap(controller):
    """Wrap a controller with the gate (no-op when the gate is not installed)."""
    if _RADIUS is None:
        return controller
    if isinstance(controller, SensingGatedController):
        return controller
    return SensingGatedController(controller)
