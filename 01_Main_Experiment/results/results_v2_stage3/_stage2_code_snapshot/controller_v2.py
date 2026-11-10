"""Stage-2 controller layer: speed policy, predictor selection, shadow mode.

Three wrappers, all additive -- the V1 controller classes are imported, never
edited:

``OursRolloutV2``
    ``Ours_rollout`` with (a) the predictor engine selectable by configuration
    (``predictor_engine`` / ``predictor_model``) and (b) a *declared manoeuvre*
    (mode + along-path lookahead + commanded lateral offset), which is what the
    layered speed policy needs.  With ``predictor_engine="dll"``,
    ``predictor_model="M0"`` and ``speed_policy="off"`` it is the V1 controller:
    same state machine, same offsets, same cost, same DLL predictor.

``SpeedPolicyController``
    Wraps *any* controller and applies the layered speed rule
    (``v2/speed_policy_v2.py``) to the command, using the manoeuvre the inner
    controller declares.  It logs every tick.

``ShadowPredictorController``
    Wraps *any* controller, evaluates the Stage-2 predictor at the same
    planning cadence and on the same candidate offsets as the inner
    controller, records the outcome matrix, and **does not actuate it**.  This
    is what makes the predictor/environment decoupling provable: two runs that
    differ only in ``predictor_model`` produce different predictions while the
    environment trajectory is identical, because the environment never sees the
    prediction.

The decoupling is by construction: the plant calls
``v2.animal_models.make_animal_model(v2cfg.animal_model, ...)`` while the
controller calls ``evaluate_rollouts(..., predictor_model=v2cfg.predictor_model,
...)``.  Nothing else connects them.
"""
from __future__ import annotations

import ctypes
import sys
from pathlib import Path

import numpy as np

from proposed_event import EventProposed
from proposed_rollout import RolloutProposed
from robots import flock_geom

from v2.config_v2 import default_v2cfg
from v2.predictor_v2 import evaluate_rollouts
from v2.speed_policy_v2 import SpeedPolicy


# ---------------------------------------------------------------------------
class OursRolloutV2(RolloutProposed):
    """``Ours_rollout`` + selectable predictor engine + declared manoeuvre."""

    name = "Ours_rollout_v2"
    _DLL_PARS_LOOKAHEAD = 4.0          # same constant the V1 predictor uses

    def __init__(self, v2cfg=None, horizon=12.0, prediction_dt=0.15,
                 plan_period=1.0, **kw):
        self.v2cfg = v2cfg or default_v2cfg()
        EventProposed.__init__(self, **kw)
        self.horizon = float(horizon)
        self.prediction_dt = float(prediction_dt)
        self.plan_period = float(plan_period)
        self._plan_t = -np.inf
        self._offset = 0.0
        self._previous_pos = None
        self._previous_t = None
        self._heading = None
        self.plans = []
        self.pred_log = []
        self._lib = None
        self._evaluate = None
        if self.v2cfg.predictor_engine == "dll":
            self._load_dll()

    # -- predictor engines ---------------------------------------------
    def _load_dll(self):
        root = Path(__file__).resolve().parent.parent
        lib = root / ("_rollout.dll" if sys.platform.startswith("win")
                      else "_rollout.so")
        if not lib.exists():
            raise SystemExit("build _rollout.dll first")
        self._lib = ctypes.CDLL(str(lib))
        self._evaluate = self._lib.evaluate_rollouts
        self._evaluate.restype = None

    def _robot_heading(self, view):
        """Robot heading estimate from successive positions (V1 code path)."""
        rp = np.ascontiguousarray(view.robot_pos, dtype="float64")
        speed = np.zeros(len(rp))
        if self._previous_pos is not None and view.t > self._previous_t:
            velocity = (rp - self._previous_pos) / (view.t - self._previous_t)
            speed = np.linalg.norm(velocity, axis=1)
            moving = speed > 1e-4
            self._heading[moving] = np.arctan2(velocity[moving, 1],
                                               velocity[moving, 0])
        if self._heading is None:
            s, _, _ = view.path.closest(view.centroid)
            d = view.path.tangent_at(s)
            self._heading = np.full(len(rp), np.arctan2(d[1], d[0]))
        return rp, speed

    def _predict(self, view, cfg, offsets):
        rp, speed = self._robot_heading(view)
        m = self.v2cfg.predictor_model
        if self.v2cfg.predictor_engine == "dll" and m == "M0":
            p = np.ascontiguousarray(view.sheep_pos, dtype="float64")
            v = np.ascontiguousarray(view.sheep_vel, dtype="float64")
            op = np.ascontiguousarray(view.obs_pos, dtype="float64")
            ov = np.ascontiguousarray(view.obs_vel, dtype="float64")
            radii = np.array([o.radius for o in view.obstacles_active],
                             dtype="float64")
            pars = np.array([cfg.v_base, cfg.v_flee, cfg.turn_max, cfg.r_rep,
                             cfg.r_ori, cfg.r_att, cfg.r_avo, cfg.avo_k,
                             cfg.goal_bias, cfg.v_max, cfg.omega_max,
                             cfg.a_lin, cfg.kp, cfg.d_rr_safe, cfg.d_ro_safe,
                             cfg.slot_gap, cfg.r_sheep,
                             self._DLL_PARS_LOOKAHEAD], dtype="float64")
            actions = np.ascontiguousarray(offsets, dtype="float64")
            output = np.zeros((len(actions), 7), dtype="float64")
            arrays = [p, v, rp, self._heading, speed, op, ov, radii,
                      np.ascontiguousarray(view.path.verts),
                      np.ascontiguousarray(view.path.halfwidth),
                      pars, actions]
            ptr = lambda a: a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))  # noqa: E731
            self._evaluate(
                *[ctypes.c_int(n) for n in (len(p), len(rp), len(op),
                                            len(view.path.verts),
                                            len(actions))],
                *[ptr(a) for a in arrays], ctypes.c_double(self.prediction_dt),
                ctypes.c_int(round(self.horizon / self.prediction_dt)),
                ptr(output))
            return output
        radii = np.array([o.radius for o in view.obstacles_active],
                         dtype="float64")
        pars = np.array([cfg.v_base, cfg.v_flee, cfg.turn_max, cfg.r_rep,
                         cfg.r_ori, cfg.r_att, cfg.r_avo, cfg.avo_k,
                         cfg.goal_bias, cfg.v_max, cfg.omega_max, cfg.a_lin,
                         cfg.kp, cfg.d_rr_safe, cfg.d_ro_safe, cfg.slot_gap,
                         cfg.r_sheep, self._DLL_PARS_LOOKAHEAD],
                        dtype="float64")
        return evaluate_rollouts(
            view.sheep_pos, view.sheep_vel, rp, self._heading, speed,
            view.obs_pos, view.obs_vel, radii, view.path, pars, offsets,
            dt=self.prediction_dt,
            steps=int(round(self.horizon / self.prediction_dt)),
            predictor_model=m, mcfg=self.v2cfg,
            speed_policy=bool(self.v2cfg.predictor_speed_policy),
            a_lat=float(self.v2cfg.a_lat))

    # -- declared manoeuvre (for the speed policy) ---------------------
    def step(self, view, cfg):
        cmd, tag = super().step(view, cfg)
        if tag in ("pre", "divert", "recover") and view.obs_active_n:
            ahead = 6.0 if abs(self._offset) < 1e-8 else 4.0
            c, _ = flock_geom(view)
            _, lat_c, _ = view.path.closest(c)
            self.last_manoeuvre = dict(mode=tag, ahead=float(ahead),
                                       lateral=float(self._offset - lat_c),
                                       offset=float(self._offset),
                                       lat_c=float(lat_c))
        else:
            self.last_manoeuvre = None
        return cmd, tag

    def manoeuvre(self):
        return getattr(self, "last_manoeuvre", None)

    def summary(self):
        out = dict(predictor_engine=self.v2cfg.predictor_engine,
                   predictor_model=self.v2cfg.predictor_model,
                   n_plans=len(self.plans))
        return out


# ---------------------------------------------------------------------------
class SpeedPolicyController:
    """Apply the layered speed rule to whatever the inner controller commands."""

    def __init__(self, inner, v2cfg, log_ticks=True):
        self.inner = inner
        self.v2cfg = v2cfg
        self.policy = SpeedPolicy(v2cfg)
        self.log_ticks = bool(log_ticks)
        self.name = getattr(inner, "name", "ctrl")

    def __getattr__(self, item):
        return getattr(self.inner, item)

    def step(self, view, cfg):
        cmd, tag = self.inner.step(view, cfg)
        mf = None
        getter = getattr(self.inner, "manoeuvre", None)
        if callable(getter):
            mf = getter()
        guide = None
        if mf is not None:
            guide = dict(ahead=mf.get("ahead"), lateral=mf.get("lateral"))
        out, info = self.policy.limit(cmd, guide, t=view.t, mode=tag)
        if not self.log_ticks:
            self.policy.log = self.policy.log[-1:]
        return out, tag

    def summary(self):
        s = dict(self.policy.summary())
        inner = getattr(self.inner, "summary", None)
        if callable(inner):
            s.update(inner())
        return s

    @property
    def policy_log(self):
        return self.policy.log


# ---------------------------------------------------------------------------
class ShadowPredictorController:
    """Evaluate the Stage-2 predictor in parallel; never actuate it.

    At the same cadence as the inner controller (``plan_period``) and on the
    same candidate offsets, this records the 7-column outcome matrix.  Because
    the recorded value is discarded, two runs that differ only in
    ``predictor_model`` have *identical* environment trajectories; the
    predictions differ.  That is the decoupling proof (acceptance test D).
    """

    def __init__(self, inner, v2cfg):
        self.inner = inner
        self.v2cfg = v2cfg
        self.name = getattr(inner, "name", "ctrl")
        self.log = []
        self._t_last = -np.inf
        self._heading = None
        self._prev_pos = None
        self._prev_t = None

    def __getattr__(self, item):
        return getattr(self.inner, item)

    def _robot_heading(self, view):
        rp = np.asarray(view.robot_pos, dtype=float)
        speed = np.zeros(len(rp))
        if self._heading is None:
            s, _, _ = view.path.closest(view.centroid)
            d = view.path.tangent_at(s)
            self._heading = np.full(len(rp), np.arctan2(d[1], d[0]))
        if self._prev_pos is not None and view.t > self._prev_t:
            vel = (rp - self._prev_pos) / (view.t - self._prev_t)
            speed = np.linalg.norm(vel, axis=1)
            mv = speed > 1e-4
            self._heading[mv] = np.arctan2(vel[mv, 1], vel[mv, 0])
        return rp, speed

    def step(self, view, cfg):
        cmd, tag = self.inner.step(view, cfg)
        period = float(getattr(self.inner, "plan_period", 1.0))
        if (tag in ("pre", "divert", "recover") and view.obs_active_n
                and view.t - self._t_last >= period - 1e-7):
            s, _, _ = view.path.closest(view.centroid)
            bound = min(3.6, 0.6 * view.path.halfwidth_at(s))
            off = float(getattr(self.inner, "_offset", 0.0))
            offsets = np.unique(np.clip([0.0, -1.35, 1.35, -2.4, 2.4,
                                         -3.6, 3.6, off], -bound, bound))
            rp, speed = self._robot_heading(view)
            radii = np.array([o.radius for o in view.obstacles_active],
                             dtype=float)
            pars = np.array([cfg.v_base, cfg.v_flee, cfg.turn_max, cfg.r_rep,
                             cfg.r_ori, cfg.r_att, cfg.r_avo, cfg.avo_k,
                             cfg.goal_bias, cfg.v_max, cfg.omega_max,
                             cfg.a_lin, cfg.kp, cfg.d_rr_safe, cfg.d_ro_safe,
                             cfg.slot_gap, cfg.r_sheep, 4.0], dtype=float)
            out = evaluate_rollouts(
                view.sheep_pos, view.sheep_vel, rp, self._heading, speed,
                view.obs_pos, view.obs_vel, radii, view.path, pars, offsets,
                dt=self.v2cfg.predictor_dt,
                steps=int(round(self.v2cfg.predictor_horizon
                                / self.v2cfg.predictor_dt)),
                predictor_model=self.v2cfg.predictor_model, mcfg=self.v2cfg,
                speed_policy=bool(self.v2cfg.predictor_speed_policy),
                a_lat=float(self.v2cfg.a_lat))
            self.log.append(dict(t=float(view.t), tag=str(tag),
                                 offsets=offsets.tolist(),
                                 outcomes=out.tolist()))
            self._t_last = float(view.t)
        self._prev_pos = np.asarray(view.robot_pos, dtype=float).copy()
        self._prev_t = float(view.t)
        return cmd, tag

    def summary(self):
        if not self.log:
            return dict(shadow_n=0)
        first = np.array(self.log[0]["outcomes"])
        return dict(shadow_n=len(self.log),
                    shadow_sig=float(np.abs(first).sum()))


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
def unpack(controller):
    """Unwrap our wrappers -> (core controller, SpeedPolicyController|None,
    ShadowPredictorController|None)."""
    pol = shadow = None
    cur = controller
    for _ in range(3):
        if isinstance(cur, SpeedPolicyController):
            pol = cur
            cur = cur.inner
        elif isinstance(cur, ShadowPredictorController):
            shadow = cur
            cur = cur.inner
        else:
            break
    return cur, pol, shadow


def make_controller(alg, algs, v2cfg, params=None, shadow=False):
    """Build the Stage-2 controller stack for a run.

    ``algs`` is ``run_experiments.ALGS``.  ``Ours_rollout`` is replaced by the
    Stage-2 subclass (same state machine, selectable predictor, declared
    manoeuvre); every other algorithm is built unchanged, so the baselines keep
    their V1 behaviour, and the speed policy / shadow predictor -- when enabled
    -- wraps whatever was built.
    """
    params = params or {}
    if shadow:
        # In shadow mode the LIVE predictor stays the V1 one (DLL, M0), so the
        # environment cannot depend on the model under study; the shadow
        # evaluation uses the configured predictor_model.  Without this the
        # "same environment, different predictions" proof would be circular.
        inner_cfg = v2cfg.with_(predictor_engine="dll", predictor_model="M0")
    else:
        inner_cfg = v2cfg
    if alg == "Ours_rollout":
        ctrl = OursRolloutV2(v2cfg=inner_cfg, **params)
    else:
        ctrl = algs[alg](**params)
    if shadow:
        ctrl = ShadowPredictorController(ctrl, v2cfg)
    if v2cfg.speed_policy != "off":
        ctrl = SpeedPolicyController(ctrl, v2cfg)
    return ctrl
