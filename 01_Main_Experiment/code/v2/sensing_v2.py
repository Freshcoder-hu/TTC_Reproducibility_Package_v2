"""Stage-3 sensing interface: two independent channels + observation quality.

WHY A SEPARATE MODULE
---------------------
The frozen ``simulation.View`` has exactly one sensing radius (``cfg.r_sense``
= 10 m) that drives the robot->animal disc, and it makes *every* released
obstacle visible to the controller at any distance.  Stage 3 needs three things
the V1 interface does not have:

1. **two separate channels** -- robot->animal radius 25 m and robot->obstacle
   range 70 m (swept 40/70/100 m) -- which must not be coupled;
2. **observation quality** -- obstacle position error, delivery delay and
   packet loss, all config-selected and all *off* by default;
3. **honest gating** -- the controller, the rollout predictor and the
   execution-safety layer must see only the obstacles that have actually been
   detected and delivered, with a timestamped track age.

The V1 tree must stay bit-reproducible, so nothing here is imported by it and
``View`` is neither subclassed nor monkey-patched.  ``run_scene_v2`` keeps the
frozen ``View`` as the **truth** view (collision detection, metrics,
termination) and asks this module for a second, gated view that only the
controller and the execution layer consume.

SHARING MODEL (author decision, stated explicitly)
--------------------------------------------------
Option (a) "per-robot independent" is **not** the shipped model; option (b) is:

    every robot samples its own animal disc and its own obstacle range,
    timestamps each sample, and broadcasts it; the fleet-wide observation at
    time ``t`` is the **union** of the messages that have *arrived* by ``t``
    (arrival = stamp + delay) and whose stamp is younger than ``track_ttl``.

Because ``View.refresh`` in V1 already unions *all* robots' discs (the pairwise
``r_share`` term is a subset of the union and therefore inert -- see the
Stage-3 report §2.3), the 25 m/delay/loss version below is the same union,
restricted to delivered messages.  ``sense_share="independent"`` is a two-line
alternative that is *not* used.

The obstacle channel keeps a **track** per obstacle index: position estimate,
velocity, radius, the stamp of the last received sample, the age ``t - stamp``,
the detecting robot and the first-detection time.  Tracks are logged
(``res["v2"]["sense_*"]`` and ``res["sense_log"]``).

WHAT IS *NOT* MODELLED (reported as 待确认)
------------------------------------------
* velocity error -- the manuscript's Table 13 lists a position-error knob only,
  so the track velocity/radius are those carried by the received message
  (no added noise);
* bursty packet loss -- loss is i.i.d. per sample per robot;
* occlusion / field of view -- the Stage-3 default is range-only, matching the
  ``sensing_gate.py`` precedent (a cone would make the interface non-inert at
  R -> infinity, which the inertness test requires).

INERTNESS
---------
``sense_mode="exact"`` (the default) makes :func:`build` return ``None``;
``run_scene_v2`` then uses the truth view for the controller, which is exactly
the Stage-1/2 code path.  ``sense_mode="range"`` with ``sense_r_obs=1e9``,
``sense_err=0``, ``sense_delay=0``, ``sense_loss=0`` and ``sense_r_animal=10``
is the *V1-equivalent* configuration and must reproduce the Stage-2 trajectory
bit-for-bit (acceptance test A).
"""
from __future__ import annotations

import copy
from collections import deque

import numpy as np

__all__ = ["Sensorium", "ObsTrack", "build", "TAG_ERR", "TAG_LOSS"]

TAG_ERR = 20260401          # RNG stream tag: obstacle position error
TAG_LOSS = 20260402         # RNG stream tag: packet loss
INF_RANGE = 1.0e9           # "full visibility" sentinel (finite: no inf math)
_EPS = 1e-9


# ---------------------------------------------------------------------------
class ObsTrack:
    """A published obstacle track: the only obstacle object the controller sees.

    ``pos``/``vel``/``radius`` are the **estimates carried by the last delivered
    message** (not the simulator's ground truth), and ``is_alive()`` is True
    while the track is inside its expiry window.  The object identity is stable
    for the life of the track, which is what the V1 controller's
    ``cur is not threat`` bookkeeping relies on.
    """

    __slots__ = ("index", "pos", "vel", "radius", "t_stamp", "age",
                 "source", "n_updates", "first_detect_t")

    def __init__(self, index, radius=0.0):
        self.index = int(index)
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.radius = float(radius)
        self.t_stamp = -np.inf
        self.age = np.inf
        self.source = -1
        self.n_updates = 0
        self.first_detect_t = None

    def is_alive(self):
        return True

    def __repr__(self):
        return (f"ObsTrack(k={self.index}, age={self.age:.3f}, "
                f"pos=({self.pos[0]:.2f},{self.pos[1]:.2f}))")


# ---------------------------------------------------------------------------
class Sensorium:
    """Sampling + delivery + fusion for the two Stage-3 sensing channels."""

    def __init__(self, v2cfg, cfg, seed=0):
        self.v2cfg = v2cfg
        self.mode = str(v2cfg.sense_mode)
        self.r_animal = float(v2cfg.sense_r_animal)
        self.r_obs = float(v2cfg.sense_r_obs)
        self.err = float(v2cfg.sense_err)
        self.delay = float(v2cfg.sense_delay)
        self.loss = float(v2cfg.sense_loss)
        self.ttl = float(v2cfg.sense_ttl)
        self.extrapolate = bool(v2cfg.sense_extrapolate)
        self.share = str(v2cfg.sense_share)
        self.sample_every = max(1, int(v2cfg.sense_every))
        # Perception frames run at the integration rate (20 Hz by default):
        # ``sense_every`` is in integration ticks, not in control periods.
        self.dt = float(cfg.dt) * self.sample_every
        self.assert_on = bool(v2cfg.sense_assert)
        self._seed = int(seed)
        # Dedicated RNG streams: plant / animal / sensing streams are
        # independent, so switching the sensing quality on can never change the
        # seeded initial condition or the animal model.
        self._rng_err = (np.random.default_rng([self._seed, TAG_ERR])
                         if self.err > 0 else None)
        self._rng_loss = (np.random.default_rng([self._seed, TAG_LOSS])
                          if self.loss > 0 else None)
        self._buf_obs = {}          # (robot, k) -> deque[(stamp, pos, vel, rad)]
        self._buf_ani = {}          # robot -> deque[(stamp, idx, pos, vel)]
        self._tracks = {}           # k -> ObsTrack
        self._last_step = -10 ** 9
        self._ctrl_tick = False
        self._truth = (np.zeros((0, 2)), np.zeros(0, dtype=bool))
        self._last_fallback = False
        # statistics
        self._n_samples = 0
        self._n_dropped = 0
        self._ages = []
        self._n_steps = 0
        self._n_empty_steps = 0
        self._n_fallback = 0
        self._ani_count = []
        self._first_track = {}
        self.n_robots = int(cfg.n_robots)
        self.log = []               # one row per published tick

    # -- configuration summary -----------------------------------------
    def config_row(self):
        return dict(sense_mode=self.mode,
                    sense_r_animal=self.r_animal,
                    sense_r_obs=self.r_obs,
                    sense_err=self.err, sense_delay=self.delay,
                    sense_loss=self.loss, sense_ttl=self.ttl,
                    sense_share=self.share, sense_dt=self.dt,
                    sense_extrapolate=self.extrapolate)

    # ------------------------------------------------------------------
    # 1. sampling (per robot, own channel, own obstacle range)
    # ------------------------------------------------------------------
    def _sample(self, t, robot_pos, obs):
        if obs:
            op = np.array([o.pos for o in obs], dtype=float).reshape(-1, 2)
            ov = np.array([o.vel for o in obs], dtype=float).reshape(-1, 2)
            rad = np.array([o.radius for o in obs], dtype=float)
            alive = np.array([o.is_alive() for o in obs], dtype=bool)
        else:
            op = np.zeros((0, 2))
            ov = np.zeros((0, 2))
            rad = np.zeros(0)
            alive = np.zeros(0, dtype=bool)
        self._truth = (op, alive)
        self._n_samples += 1
        M = len(robot_pos)
        if len(op) == 0 or M == 0:
            return
        d = np.linalg.norm(op[:, None, :] - np.asarray(robot_pos)[None, :, :],
                           axis=2)                       # (K, M)
        seen = (d <= self.r_obs) & alive[:, None]
        for k in range(len(op)):
            for j in np.where(seen[k])[0]:
                if self._drop():
                    continue
                p = op[k].copy()
                if self._rng_err is not None:
                    p = p + self._rng_err.normal(0.0, self.err, 2)
                self._buf_obs.setdefault((int(j), k), deque()).append(
                    (t, p, ov[k].copy(), float(rad[k]), float(d[k, j])))

    def _drop(self):
        if self._rng_loss is not None and self._rng_loss.random() < self.loss:
            self._n_dropped += 1
            return True
        return False

    def _sample_animals(self, t, robot_pos, sheep_pos, sheep_vel):
        M = len(robot_pos)
        n = len(sheep_pos)
        if M == 0 or n == 0:
            return
        d = np.linalg.norm(sheep_pos[None, :, :] - np.asarray(robot_pos)[:, None, :],
                           axis=2)                       # (M, n)
        for j in range(M):
            if self._drop():
                continue
            idx = np.where(d[j] <= self.r_animal)[0]
            self._buf_ani.setdefault(j, deque()).append(
                (t, idx, sheep_pos[idx].copy(), sheep_vel[idx].copy()))

    # ------------------------------------------------------------------
    # 2. delivery + fusion
    # ------------------------------------------------------------------
    def _trim(self, t):
        keep = t - self.ttl - self.delay - self.dt - _EPS
        for key in list(self._buf_obs):
            dq = self._buf_obs[key]
            while dq and dq[0][0] < keep:
                dq.popleft()
            if not dq:
                del self._buf_obs[key]
        for j in list(self._buf_ani):
            dq = self._buf_ani[j]
            while dq and dq[0][0] < keep:
                dq.popleft()
            if not dq:
                del self._buf_ani[j]

    def _delivered(self, dq, t):
        """Newest message in ``dq`` that has arrived by ``t`` (or ``None``)."""
        cut = t - self.delay + _EPS
        for msg in reversed(dq):
            if msg[0] <= cut:
                return msg
        return None

    def update(self, t, step, ctrl_tick, robot_pos, obs, sheep_pos, sheep_vel):
        """Advance the sensing pipeline to time ``t`` (integration tick ``step``)."""
        self._ctrl_tick = bool(ctrl_tick)
        if obs:
            self._truth = (np.array([o.pos for o in obs], dtype=float)
                           .reshape(-1, 2),
                           np.array([o.is_alive() for o in obs], dtype=bool))
        else:
            self._truth = (np.zeros((0, 2)), np.zeros(0, dtype=bool))
        if step - self._last_step >= self.sample_every:
            self._last_step = step
            self._sample(t, robot_pos, obs)
            self._sample_animals(t, robot_pos, sheep_pos, sheep_vel)
        self._trim(t)

    # -- the gated view ------------------------------------------------
    def publish(self, view_truth, t):
        """Return a *copy* of the truth view with the sensing interface applied.

        Only the fields a controller may read are overwritten:
        ``obstacles_active`` / ``obs_pos`` / ``obs_vel`` / ``obs_active_n`` and
        ``sheep_pos`` / ``sheep_vel`` / ``mask_direct`` / ``mask_shared``.
        Everything else (path, robot_pos, and the global truth the metrics use)
        is inherited unchanged.
        """
        v = copy.copy(view_truth)
        # ---------------- obstacle channel: newest delivered sample per k ----
        by_k = {}
        for (j, k), dq in self._buf_obs.items():
            msg = self._delivered(dq, t)
            if msg is None:
                continue
            cur = by_k.get(k)
            if cur is None or msg[0] > cur[0] or (msg[0] == cur[0]
                                                  and j < cur[1]):
                by_k[k] = (msg[0], j, msg[1], msg[2], msg[3])
        tracks = []
        for k in sorted(by_k):
            stamp, j, p, vel, rad = by_k[k]
            if (t - stamp) > self.ttl + _EPS:
                continue
            tr = self._tracks.get(k)
            if tr is None:
                tr = ObsTrack(k, rad)
                self._tracks[k] = tr
            tr.pos = np.asarray(p, dtype=float).copy()
            tr.vel = np.asarray(vel, dtype=float).copy()
            tr.radius = float(rad)
            tr.t_stamp = float(stamp)
            tr.age = float(t - stamp)
            tr.source = int(j)
            tr.n_updates += 1
            if self.extrapolate:
                tr.pos = tr.pos + tr.vel * tr.age
            if tr.first_detect_t is None:
                tr.first_detect_t = float(stamp)
                self._first_track.setdefault(k, float(stamp))
            tracks.append(tr)
        v.obstacles_active = tracks
        v.obs_active_n = len(tracks)
        v.obs_pos = (np.array([o.pos for o in tracks], dtype=float)
                     .reshape(-1, 2) if tracks else np.zeros((0, 2)))
        v.obs_vel = (np.array([o.vel for o in tracks], dtype=float)
                     .reshape(-1, 2) if tracks else np.zeros((0, 2)))
        # ---------------- animal channel: union of delivered messages -------
        best = {}
        for j, dq in self._buf_ani.items():
            msg = self._delivered(dq, t)
            if msg is None:
                continue
            stamp, idx, pos, vel = msg
            if (t - stamp) > self.ttl + _EPS:
                continue
            for q, i in enumerate(idx):
                prev = best.get(int(i))
                if prev is None or stamp > prev[0]:
                    best[int(i)] = (stamp, pos[q], vel[q])
        n_all = len(view_truth.sheep_pos_global)
        self._last_fallback = not (n_all and best)
        if not self._last_fallback:
            order = sorted(best)
            v.sheep_pos = np.array([best[i][1] for i in order], dtype=float)
            v.sheep_vel = np.array([best[i][2] for i in order], dtype=float)
            m = np.zeros(n_all, dtype=bool)
            m[order] = True
            v.mask_direct = m
            v.mask_shared = m
        else:
            # Stage-1 fallback rule, retained verbatim: an empty observed set
            # hands the controller the whole flock.  This is the ONLY path by
            # which unsensed animal state can reach the controller, and it is
            # counted (``sense_animal_fallback_ticks``).
            v.sheep_pos = view_truth.sheep_pos_global.copy()
            v.sheep_vel = view_truth.sheep_vel_global.copy()
            v.mask_direct = np.ones(n_all, dtype=bool)
            v.mask_shared = np.ones(n_all, dtype=bool)
            self._n_fallback += 0        # counted per control tick below
        # ---------------- bookkeeping --------------------------------------
        if self._ctrl_tick:
            self._n_steps += 1
            if not tracks:
                self._n_empty_steps += 1
            self._ages.extend([o.age for o in tracks])
            self._ani_count.append(len(v.sheep_pos))
            self._n_fallback += int(self._last_fallback)
            self.log.append(dict(
                t=float(t), n_obs=len(tracks),
                obs_index=[int(o.index) for o in tracks],
                obs_age=[float(o.age) for o in tracks],
                obs_pos=[o.pos.tolist() for o in tracks],
                n_animals=int(len(v.sheep_pos)),
                animal_fallback=bool(self._last_fallback)))
        if self.assert_on:
            self.check(view_truth, v, t, by_k)
        return v

    # ------------------------------------------------------------------
    # 3. assertions (acceptance test: the interface is real)
    # ------------------------------------------------------------------
    def check(self, view_truth, view_ctrl, t, by_k=None):
        """Assert that ``view_ctrl`` carries no information the interface denies.

        Raises ``AssertionError`` with a precise message.  Cheap enough to run
        on every tick for a pilot run (``sense_assert=True``), off by default.
        """
        op_all, alive = self._truth
        for o in view_ctrl.obstacles_active:
            assert 0 <= o.index < len(alive), "track index out of range"
            # (a) only a delivered message can be published
            msg = self._delivered(self._buf_obs.get((o.source, o.index),
                                                    deque()), t)
            assert msg is not None, "published obstacle was never delivered"
            # (b) the published position IS the (possibly stale) estimate
            est = np.asarray(msg[1], dtype=float)
            if self.extrapolate:
                est = est + np.asarray(msg[2], dtype=float) * (t - msg[0])
            assert np.array_equal(o.pos, est), \
                f"track {o.index} position is not the delivered estimate"
            # (c) no future information
            assert o.t_stamp <= t + _EPS, "track stamped in the future"
            if self.delay > 0:
                assert o.t_stamp <= t - self.delay + _EPS, \
                    "track younger than the delivery delay"
            assert o.age >= -_EPS, "negative track age"
            # (d) the message that produced this track was acquired inside the
            #     configured obstacle range -- a track can never outlive its
            #     expiry window and cannot be invented
            assert float(msg[4]) <= self.r_obs + 1e-6, (
                f"track {o.index} published from a {float(msg[4]):.1f} m "
                f"detection > R={self.r_obs:.1f} m")
        # (e) animal set is a subset of the flock
        n_all = len(view_truth.sheep_pos_global)
        assert len(view_ctrl.sheep_pos) <= n_all, "animal set larger than flock"
        # (f) the gated view cannot alias the truth obstacle arrays
        if len(view_ctrl.obs_pos):
            assert not np.shares_memory(view_ctrl.obs_pos, view_truth.obs_pos), \
                "gated obstacle array aliases the truth array"
        assert view_ctrl.path is view_truth.path
        assert view_ctrl.robot_pos is view_truth.robot_pos

    # ------------------------------------------------------------------
    def summary(self):
        ages = np.array(self._ages, dtype=float) if self._ages else np.zeros(0)
        ani = np.array(self._ani_count, dtype=float) if \
            self._ani_count else np.zeros(0)
        nt = (float(np.mean([r["n_obs"] for r in self.log]))
              if self.log else 0.0)
        return dict(
            sense_mode=self.mode, sense_r_animal=self.r_animal,
            sense_r_obs=self.r_obs, sense_err=self.err,
            sense_delay=self.delay, sense_loss=self.loss,
            sense_ttl=self.ttl, sense_share=self.share,
            sense_samples=self._n_samples, sense_dropped=self._n_dropped,
            sense_publish_steps=self._n_steps,
            sense_empty_obs_steps=self._n_empty_steps,
            sense_n_tracks_mean=nt,
            sense_track_age_mean=(float(ages.mean()) if ages.size else np.nan),
            sense_track_age_max=(float(ages.max()) if ages.size else np.nan),
            sense_animal_fallback_ticks=int(self._n_fallback),
            sense_animal_obs_mean=(float(ani.mean()) if ani.size else np.nan),
            sense_first_detect_1=self._first_track.get(0, np.nan),
            sense_first_detect_2=self._first_track.get(1, np.nan),
            sense_first_detect_3=self._first_track.get(2, np.nan),
        )


# ---------------------------------------------------------------------------
def build(v2cfg, cfg, seed):
    """Return a :class:`Sensorium`, or ``None`` for the inert configuration."""
    mode = str(getattr(v2cfg, "sense_mode", "exact"))
    if mode == "exact":
        return None
    if mode != "range":
        raise ValueError(f"unknown sense_mode {mode!r} (exact|range)")
    return Sensorium(v2cfg, cfg, seed)
