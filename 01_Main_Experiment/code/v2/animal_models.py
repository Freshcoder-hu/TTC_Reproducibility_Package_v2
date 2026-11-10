"""Stage-2 animal behaviour models M1 (local obstacle avoidance) and M2 (stress).

WHERE THIS SITS
---------------
``v2/animal_m0.py::desired_sheep_update`` is the frozen "what the animal wants"
function of Stage 1 (a transcription of ``sheep._advance_sheep``).  Stage 2
replaces **only** that function -- the (V2) box projection below it, the
execution loop, the contact detector and the metrics do not change.  M0 keeps
calling the Stage-1 function verbatim, so M0 stays bit-exact.

M1  = M0 + one added term, the local obstacle avoidance:

    f_obs,i = sum_o  w(g_io) * u(p_i - p_o)

    g_io = ||p_i - p_o|| - r_s - r_o          (SURFACE gap, not centre distance)
    w(g) = k_M1 * (1 - g/R)^p  (or k_M1 * exp(-g/L)),  and w = 0 for g >= R

``R`` is the animal's *own* sensing range (3/6/9 m sweep), applied to the
surface gap; ``obs`` is a **delayed snapshot** of the animal-local observation
(``m1_obs_delay``), never a robot-shared track and never a future trajectory.

M2  = M1 + per-animal response gain (seeded lognormal, mean 1), a per-animal
reaction delay (extra transport delay on the observed stimulus) and a
stimulus-dependent speed target in [0, m_v_max] with the exploratory upper
reference 1.8-2.0 m/s.  **M2 is a behavioural stress model, not a validated
description of frightened sheep** (manuscript 2.1; see the Stage-2 report).

SEPARATION PRIORITY (both models)
---------------------------------
The V1 separation branch (a neighbour inside ``r_rep``, or a robot inside
``r_avo``) is retained unchanged.  When it is active, the obstacle term is added
to the separation direction but its component *against* separation is removed,
so

    (u_sep + a_perp) . u_sep = 1 > 0

i.e. the weighted sum can never reverse the separation direction.  If a body
overlap exists while an obstacle is sensed (surface gap <= ``m1_body_guard``),
the obstacle term is dropped entirely and pure separation is used.  Both events
are counted (``m1_sep_guard`` / ``m1_body_priority``), so the claim is testable.

WHY THE ZERO-OBSTACLE PATH IS A SEPARATE BRANCH
-----------------------------------------------
If no obstacle is inside any animal's range, M1/M2 take the **verbatim V1
composition** (``_compose_v1``) instead of adding a zero vector: adding 0.0 and
re-normalising would perturb the last bits.  This is what makes

    M1(m1_gain=0) == M0      and      M2(sigma=0, delay=0, m_v_max=m_v_base) == M1

bit-exact, which is checked in ``v2/acceptance_s2.py``.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from sheep import _heading
from v2.animal_m0 import advance_sheep_m0, desired_sheep_update
from v2.limits_v2 import project_sheep_increment

FAR = 1.0e6          # position written for an obstacle that is not alive
_EPS = 1e-9


# ---------------------------------------------------------------------------
# obstacle weight
# ---------------------------------------------------------------------------
def obstacle_weight(gap, sense_range, decay="linear", decay_len=2.0, gain=1.0):
    """Bounded, distance-decaying weight on the surface gap (compact support).

    ``gap`` is ``||p_i - p_o|| - r_s - r_o``; a body overlap (gap < 0) is
    clamped to 0 so the weight is bounded by ``gain``.  ``w == 0`` for
    ``gap >= sense_range`` -- that is the "vanishes outside the animal's own
    sensing range" requirement, enforced by construction rather than by a
    later mask.
    """
    if sense_range <= 0.0:
        return np.zeros_like(gap)
    g = np.clip(gap, 0.0, None)
    if decay == "linear":
        f = 1.0 - g / sense_range
    elif decay == "quadratic":
        f = (1.0 - g / sense_range) ** 2
    elif decay == "exp":
        # compact support: e-folding length decay_len, cut at sense_range
        f = np.exp(-g / max(decay_len, 1e-9))
    else:
        raise ValueError(f"unknown m1_decay={decay!r}")
    f = np.clip(f, 0.0, 1.0)
    # NaN-safe: a dead obstacle sits at FAR, so gap is huge and the mask is False
    return np.where(gap < sense_range, gain * f, 0.0)


# ---------------------------------------------------------------------------
# the V1 composition, transcribed (used when no obstacle is sensed)
# ---------------------------------------------------------------------------
def _compose_v1(rep_act, rep_total, steer, d):
    m = len(d)
    want = np.zeros((m, 2))
    rn = np.linalg.norm(rep_total, axis=1)
    use = rep_act & (rn > 1e-6)
    want[use] = rep_total[use] / np.maximum(rn[use], 1e-6)[:, None]
    other = ~use
    sn = np.linalg.norm(steer, axis=1)
    good = other & (sn > 1e-6)
    want[good] = steer[good] / sn[good, None]
    idle = ~(use | good)
    want[idle] = d[idle]
    return want


# ---------------------------------------------------------------------------
# obstacle term (used by both the simulator and the M2 reaction-delay buffer)
# ---------------------------------------------------------------------------
def obstacle_term(pos, cfg, mcfg, obs_pos, obs_r, gain=None):
    """``f_obs``, its magnitude, the per-animal sensed count and the gap."""
    if obs_pos is None or len(obs_pos) == 0:
        m = len(pos)
        return (np.zeros((m, 2)), np.zeros(m), np.zeros(m, dtype=int),
                np.full(m, np.inf))
    ddo = pos[:, None, :] - np.asarray(obs_pos, dtype=float)[None, :, :]
    disto = np.linalg.norm(ddo, axis=2)
    gap = disto - cfg.r_sheep - np.asarray(obs_r, dtype=float)[None, :]
    w = obstacle_weight(gap, mcfg.m1_sense_range, mcfg.m1_decay,
                        mcfg.m1_decay_len, mcfg.m1_gain)
    if gain is not None:
        w = w * gain[:, None]
    uo = ddo / np.maximum(disto, 1e-6)[:, :, None]
    f_obs = (uo * w[:, :, None]).sum(axis=1)
    return (f_obs, np.linalg.norm(f_obs, axis=1), (w > 0).sum(axis=1),
            gap.min(axis=1))


# ---------------------------------------------------------------------------
# M1 / M2 desired update
# ---------------------------------------------------------------------------
def desired_sheep_update_m1(st, cfg, mcfg, robots_pos, path,
                            obs_pos=None, obs_r=None, gain=None, mode="M1",
                            diag=None, f_obs_pre=None):
    """M1/M2 requested (heading increment, speed) for every alive animal.

    Parameters
    ----------
    obs_pos, obs_r : delayed animal-local obstacle snapshot, ``(K,2)`` / ``(K,)``.
        Dead obstacles must already be written at :data:`FAR`.
    gain : ``(m,)`` per-animal response gain (M2); ``None`` means 1 (M1).
    mode : ``"M1"`` or ``"M2"`` (M2 adds the stimulus-dependent speed target).
    diag : optional dict, filled with per-animal diagnostics (``f_obs``,
        ``stim``, ``gap_min``, counts) -- read-only, used by the acceptance
        tests; it never feeds back into the dynamics.

    Returns ``(dth, spd_des, dth_raw, flee)`` exactly like the M0 function.
    """
    pos = st.pos[st.alive]
    head = st.head[st.alive]
    m = len(pos)
    if m == 0:
        empty = np.zeros(0)
        return empty, empty, empty, np.zeros(0, dtype=bool)

    d = _heading(head)

    # ---- sheep-sheep pairwise zones (V1, unchanged) ----
    dpos = pos[:, None, :] - pos[None, :, :]
    dist = np.linalg.norm(dpos, axis=2)
    np.fill_diagonal(dist, np.inf)

    rep_m = (dist < cfg.r_rep)
    ali_m = (dist >= cfg.r_rep) & (dist < cfg.r_ori)
    att_m = (dist >= cfg.r_ori) & (dist < cfg.r_att)

    with np.errstate(invalid="ignore", divide="ignore"):
        u = dpos / np.maximum(dist[..., None], 1e-6)
    rep_dir = np.where(rep_m[..., None], u, 0.0).sum(axis=1)
    rep_n = rep_m.sum(axis=1)

    # ---- robot repulsion (V1 loops per animal; kept identical) ----
    rob_dir = np.zeros((m, 2))
    rob_n = np.zeros(m)
    dsrc = pos[:, None, :] - robots_pos[None, :, :]
    dsrc_d = np.linalg.norm(dsrc, axis=2)
    src_m = dsrc_d < cfg.r_avo
    for i in range(m):
        idx = np.where(src_m[i])[0]
        if len(idx):
            dd = dsrc_d[i, idx]
            w = cfg.avo_k * (1.0 - dd / cfg.r_avo)
            rob_dir[i] += (dsrc[i, idx] / np.maximum(dd, 1e-6)[:, None]
                           * w[:, None]).sum(axis=0)
            rob_n[i] += len(idx)

    flee = (rob_n > 0)

    # ---- M1 obstacle term ----
    if f_obs_pre is not None:
        f_obs, f_mag, n_sensed, gap_min = f_obs_pre
    else:
        f_obs, f_mag, n_sensed, gap_min = obstacle_term(
            pos, cfg, mcfg, obs_pos, obs_r, gain=gain)

    obs_any = bool((f_mag > 0.0).any())

    # ---- alignment / attraction / path bias (V1, unchanged) ----
    steer = np.zeros((m, 2))
    if (~flee).any():
        nf = ~flee
        ali_d = np.where(ali_m[..., None], d[None, :, :], 0.0).sum(axis=1)
        att_cnt = np.where(att_m[..., None], pos[None, :, :], 0.0).sum(axis=1)
        att_num = att_m.sum(axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            att_v = att_cnt / np.maximum(att_num[:, None], 1e-6) - pos
        steer = 0.6 * ali_d + 0.4 * att_v
        if path is not None and cfg.goal_bias > 0:
            s_i, _, _ = path.closest(pos)
            for i in np.where(nf)[0]:
                steer[i] = steer[i] + cfg.goal_bias * path.tangent_at(s_i[i])

    # ---- combine ----
    rep_act = (rep_n > 0) | flee
    rep_total = rep_dir + rob_dir

    n_sep_guard = 0
    n_body_prior = 0
    if not obs_any:
        want = _compose_v1(rep_act, rep_total, steer, d)
    else:
        # (a) hard body-separation rule: an obstacle may never override it
        body_over = (dist - 2.0 * cfg.r_sheep).min(axis=1) <= mcfg.m1_body_guard
        body_over &= (np.linalg.norm(rep_dir, axis=1) > 1e-6)
        n_body_prior = int(body_over.sum())

        # (b) soft branch: obstacle term joins the weighted sum
        steer_o = steer.copy()
        soft = ~flee
        steer_o[soft] += f_obs[soft]

        want = _compose_v1(rep_act, rep_total, steer_o, d)

        # (c) separation branch: keep the separation direction, drop the part
        #     of the obstacle term that pushes against it
        rn = np.linalg.norm(rep_total, axis=1)
        use = rep_act & (rn > 1e-6) & ~body_over
        if use.any():
            u_sep = rep_total[use] / rn[use, None]
            a = f_obs[use]
            par = (a * u_sep).sum(axis=1)
            neg = par < 0.0
            if neg.any():
                a = a.copy()
                a[neg] = a[neg] - par[neg, None] * u_sep[neg]
                n_sep_guard = int(neg.sum())
            summ = u_sep + a
            nn = np.linalg.norm(summ, axis=1)
            ok = nn > 1e-9
            wv = np.empty_like(summ)
            wv[ok] = summ[ok] / nn[ok, None]
            wv[~ok] = u_sep[~ok]
            want[use] = wv

        # (d) body-overlap override: pure separation
        if body_over.any():
            rd = rep_dir[body_over]
            rdn = np.linalg.norm(rd, axis=1)
            okb = rdn > 1e-6
            wb = np.empty_like(rd)
            wb[okb] = rd[okb] / rdn[okb, None]
            wb[~okb] = want[body_over][~okb]
            want[body_over] = wb

        # (e) animals with no social direction at all but sensing an obstacle
        idle = (np.linalg.norm(want, axis=1) <= 1e-9) & (f_mag > 1e-9)
        if idle.any():
            want[idle] = f_obs[idle] / f_mag[idle, None]

    # ---- C2 (turn rate), exactly as V1 ----
    cur = head
    target = np.arctan2(want[:, 1], want[:, 0])
    dth_raw = np.angle(np.exp(1j * (target - cur)))
    dth = np.clip(dth_raw, -cfg.turn_max * cfg.dt, cfg.turn_max * cfg.dt)

    # ---- desired speed ----
    if mode == "M1" or (mode == "M2" and not getattr(mcfg, "m2_speed_map", True)):
        spd = np.where(flee, mcfg.m_v_flee, mcfg.m_v_base)
        spd = np.clip(spd, mcfg.m_s_min, mcfg.m_v_max)
    elif mode == "M2":
        sat = np.clip(f_mag / max(mcfg.m2_stim_ref, 1e-9), 0.0, 1.0)
        s_hi = mcfg.m_v_base + (mcfg.m_v_max - mcfg.m_v_base) * sat
        # turn-demand slowdown: a hard turn reduces the speed target, and with
        # m2_slow_gain = 0 the target reaches 0 (the admissible lower bound of
        # the (V2) box), i.e. the animal slows down and can stop.
        hd = np.abs(np.angle(np.exp(1j * (target - cur))))
        slow = mcfg.m2_slow_gain + (1.0 - mcfg.m2_slow_gain) * np.cos(
            np.minimum(hd, 0.5 * np.pi))
        spd = s_hi * slow
        spd = np.where(flee, np.maximum(spd, mcfg.m_v_flee), spd)
        spd = np.clip(spd, mcfg.m_s_min, mcfg.m_v_max)
    else:
        raise ValueError(mode)

    if diag is not None:
        diag["f_obs"] = f_obs
        diag["stim"] = f_mag
        diag["gap_min"] = gap_min
        diag["n_sensed"] = n_sensed
        diag["sep_guard"] = n_sep_guard
        diag["body_priority"] = n_body_prior

    return dth, spd, dth_raw, flee


# ---------------------------------------------------------------------------
# M1/M2 integration (same (V2) box as M0)
# ---------------------------------------------------------------------------
def advance_sheep_model(st, cfg, v2cfg, model, robots_pos, path=None,
                        counters=None, projection="joint", obs_pos=None,
                        obs_r=None, diag=None):
    """Integrate one ``dt`` for the M1/M2 models with the (V2) box imposed.

    Mirrors ``v2.animal_m0.advance_sheep_m0`` statement for statement; only the
    desired-update call and the model's own C1 band differ.
    """
    alive = st.alive
    if not alive.any():
        return
    pos = st.pos[alive]
    head = st.head[alive]
    spd_old = st.speed[alive]

    dth, spd_des, dth_raw, _flee = model.desire(
        st, cfg, robots_pos, path, obs_pos=obs_pos, obs_r=obs_r, diag=diag)

    if counters is not None:
        c2 = (dth != dth_raw)
        if c2.any():
            counters.hit("bind_C2_sheep", int(c2.sum()))
        counters.hit("opp_sheep_steps", int(len(pos)))

    dth_out = dth
    spd_out = spd_des
    if v2cfg.enforce_c3:
        for i in range(len(pos)):
            dth_out[i], spd_out[i], _info = project_sheep_increment(
                float(spd_old[i]), float(dth[i]), float(spd_des[i]),
                dt=cfg.dt, a_s=v2cfg.a_sheep, omega_s=cfg.turn_max,
                s_lo=model.s_lo, s_hi=model.s_hi, eps_v=v2cfg.eps_v,
                counters=counters)

    head_new = head + dth_out
    pos_new = pos + spd_out[:, None] * _heading(head_new) * cfg.dt

    st.pos[alive] = pos_new
    st.head[alive] = head_new
    st.speed[alive] = spd_out


# ---------------------------------------------------------------------------
# model objects (one per run; they own the observation-delay buffer and the
# seeded per-animal parameters, so nothing global can leak between runs)
# ---------------------------------------------------------------------------
class AnimalModelM0:
    """Stage-1 model, untouched code path (bit-exact by construction)."""

    name = "M0"
    senses = False

    def __init__(self, v2cfg, cfg, seed, n_sheep, record=False):
        self.s_lo = v2cfg.s_min
        self.s_hi = v2cfg.s_max
        self.n = int(n_sheep)
        self.record = bool(record)
        self.log = []
        self.counts = {}

    # -- the executor calls this signature for every model --
    def step(self, st, cfg, v2cfg, robots_pos, path=None, counters=None,
             projection="joint", obs_pos=None, obs_r=None, t=0.0):
        advance_sheep_m0(st, cfg, v2cfg, robots_pos, path, counters,
                         projection=projection)

    def desire(self, *a, **kw):        # not used by step(); kept for symmetry
        raise NotImplementedError

    def summary(self):
        return dict(animal_model=self.name, **self.counts)


class _ModelM1Base(AnimalModelM0):
    """Shared plumbing for M1/M2: delayed animal-local obstacle snapshots."""

    name = "M1"
    senses = True

    def __init__(self, v2cfg, cfg, seed, n_sheep, record=False):
        self.v2cfg = v2cfg
        self.cfg = cfg
        self.n = int(n_sheep)
        self.record = bool(record)
        self.log = []
        self.counts = {}
        self.s_lo = v2cfg.m_s_min
        self.s_hi = v2cfg.m_v_max

        self.d_obs = int(round(v2cfg.m1_obs_delay / cfg.dt))
        # per-animal parameters; the subclasses fill them in _setup()
        self.gain = np.ones(self.n)
        self.d_react = np.zeros(self.n, dtype=int)
        self._homogeneous = True
        self._setup(v2cfg, cfg, seed)          # M2 fills delays/gains
        self.has_response_delay = bool(int(self.d_react.max(initial=0)) > 0)
        self.d_max = self.d_obs          # observation buffer only
        self.buf = deque(maxlen=self.d_max + 1)
        self._uniform = bool(np.all(self.d_react == self.d_react[0]))
        self._delay = self.d_obs + self.d_react
        self._term = None

        # diagnostics accumulators
        self.counts = dict(n_steps=0, n_animal_steps=0,
                           n_animal_steps_sensing=0,
                           sep_guard=0, body_priority=0,
                           stim_sum=0.0, stim_max=0.0, n_sensed_sum=0.0)

    # hooks
    def _setup(self, v2cfg, cfg, seed):
        pass

    def _speed_mode(self):
        return "M1"

    # ------------------------------------------------------------------
    def _push(self, obs_pos, obs_r):
        k = 0 if obs_pos is None else len(obs_pos)
        if k == 0:
            self.buf.append((np.zeros((0, 2)), np.zeros(0)))
            return
        self.buf.append((np.asarray(obs_pos, dtype=float),
                         np.asarray(obs_r, dtype=float)))

    def _snapshot(self, delay):
        """Observation-delayed obstacle snapshot -> (pos (m,K,2), r (K,))."""
        if not self.buf:
            return None, None
        if self._uniform:
            idx = max(len(self.buf) - 1 - int(delay[0]), 0)
            p, r = self.buf[idx]
            return p, r
        arr = list(self.buf)
        while len(arr) < self.d_max + 1:
            arr.insert(0, arr[0])
        P = np.stack([a[0] for a in arr])            # (T,K,2)
        R = arr[-1][1]
        T = P.shape[0]
        idx = np.clip(T - 1 - delay, 0, T - 1)
        return P[idx], R                             # (m,K,2),(K,)

    def _react(self, term, alive):
        """Apply the per-animal *response* delay to the obstacle term.

        The reaction delay is a delay of the animal's **response**, not of its
        observation: the term is computed from the observation-delayed snapshot
        and then held in a buffer for ``d_react`` ticks.  (Delaying the
        observation instead -- which is what ``m1_obs_delay`` does -- would not
        shift the onset latency by the delay, because the animal's own position
        keeps moving while the perceived obstacle position is stale.)
        """
        if not hasattr(self, "_react_buf"):
            kmax0 = int(self.d_react.max()) if self.d_react.size else 0
            self._react_buf = deque(maxlen=kmax0 + 1)
        self._react_buf.append(term)
        kmax = int(self.d_react.max()) if self.d_react.size else 0
        if kmax == 0:
            return term
        arr = list(self._react_buf)
        while len(arr) < kmax + 1:
            arr.insert(0, arr[0])
        T = len(arr)
        f_obs = np.stack([a[0] for a in arr])          # (T,m,2)
        f_mag = np.stack([a[1] for a in arr])
        n_sen = np.stack([a[2] for a in arr])
        gap = np.stack([a[3] for a in arr])
        idx = np.clip(T - 1 - np.asarray(self.d_react)[alive], 0, T - 1)
        cols = np.arange(len(idx))
        return (f_obs[idx, cols], f_mag[idx, cols], n_sen[idx, cols],
                gap[idx, cols])

    # ------------------------------------------------------------------
    def desire(self, st, cfg, robots_pos, path, obs_pos=None, obs_r=None,
               diag=None):
        return desired_sheep_update_m1(
            st, cfg, self.v2cfg, robots_pos, path, obs_pos, obs_r,
            gain=self.gain_for_desire(st), mode=self._speed_mode(), diag=diag,
            f_obs_pre=self._term)

    def gain_for_desire(self, st):
        if self._homogeneous:
            return None
        return self.gain[st.alive]

    # ------------------------------------------------------------------
    def step(self, st, cfg, v2cfg, robots_pos, path=None, counters=None,
             projection="joint", obs_pos=None, obs_r=None, t=0.0):
        self._push(obs_pos, obs_r)
        p, r = self._snapshot(np.full(st.alive.sum(), self.d_obs))
        diag = {}
        self._term = None
        # geometric (zero-delay, current-tick) surface gap: the reference
        # instant for the onset-latency measurement, so that the *received*
        # stimulus (delayed) can be compared against it
        if obs_pos is not None and len(obs_pos):
            pos0 = st.pos[st.alive]
            dnow = np.linalg.norm(pos0[:, None, :]
                                  - np.asarray(obs_pos)[None, :, :], axis=2)
            gap_now = (dnow - cfg.r_sheep
                       - np.asarray(obs_r)[None, :]).min(axis=1)
        else:
            gap_now = np.full(int(st.alive.sum()), np.inf)
        if p is not None and len(p) and self.has_response_delay:
            # M2: compute the observation-delayed term, then delay the response
            pos = st.pos[st.alive]
            term = obstacle_term(pos, cfg, v2cfg, p, r,
                                 gain=self.gain_for_desire(st))
            self._term = self._react(term, st.alive)
        advance_sheep_model(st, cfg, v2cfg, self, robots_pos, path, counters,
                            projection=projection, obs_pos=p, obs_r=r,
                            diag=diag)
        diag["gap_now"] = gap_now
        # ---- diagnostics (read-only) ----
        c = self.counts
        c["n_steps"] += 1
        n = int(st.alive.sum())
        c["n_animal_steps"] += n
        stim = diag.get("stim")
        if stim is not None and len(stim):
            c["n_animal_steps_sensing"] += int((stim > 0).sum())
            c["stim_sum"] += float(stim.sum())
            c["stim_max"] = max(c["stim_max"], float(stim.max()))
            c["n_sensed_sum"] += float(diag["n_sensed"].sum())
            c["sep_guard"] += int(diag.get("sep_guard", 0))
            c["body_priority"] += int(diag.get("body_priority", 0))
        if self.record:
            self.log.append(dict(
                t=float(t),
                stim=(None if stim is None else stim.copy()),
                gap=(None if diag.get("gap_now") is None
                     else np.array(diag["gap_now"], copy=True)),
                spd=st.speed[st.alive].copy(),
                head=st.head[st.alive].copy(),
                pos=st.pos[st.alive].copy()))

    def summary(self):
        c = dict(self.counts)
        n = max(c.get("n_animal_steps", 0), 1)
        c["stim_mean"] = c.get("stim_sum", 0.0) / n
        c["n_sensed_mean"] = c.get("n_sensed_sum", 0.0) / n
        c["animal_model"] = self.name
        c["obs_delay_ticks"] = self.d_obs
        c["delay_ticks_mean"] = float(np.mean(self._delay))
        c["gain_min"] = float(self.gain.min())
        c["gain_max"] = float(self.gain.max())
        c["gain_mean"] = float(self.gain.mean())
        c["gain_sd"] = float(self.gain.std(ddof=1)) if self.n > 1 else 0.0
        return c


class AnimalModelM1(_ModelM1Base):
    """M1: local, finite obstacle avoidance on top of the Couzin zones."""

    name = "M1"


class AnimalModelM2(_ModelM1Base):
    """M2: M1 + heterogeneous gains + reaction delay + stimulus speed target."""

    name = "M2"

    def _setup(self, v2cfg, cfg, seed):
        n = self.n
        # seeded, run-independent stream: seed sequence [run seed, tag]
        rng = np.random.default_rng([int(seed), int(v2cfg.m2_seed_tag)])
        sig = float(v2cfg.m2_gain_sigma)
        g = np.exp(rng.normal(0.0, sig, n)) if sig > 0 else np.ones(n)
        g = g / g.mean()                       # mean 1: same total gain as M1
        self.gain = g
        self._homogeneous = bool(sig <= 0.0)
        d = np.zeros(n, dtype=int)
        if v2cfg.m2_react_delay > 0:
            d = np.full(n, int(round(v2cfg.m2_react_delay / cfg.dt)))
        # NOTE: no per-animal spread of the reaction delay is used; a spread
        # would add a parameter with nothing to calibrate it against.
        self.d_react = d

    def _speed_mode(self):
        return "M2"


MODEL_CLASSES = {"M0": AnimalModelM0, "M1": AnimalModelM1, "M2": AnimalModelM2}


def make_animal_model(v2cfg, cfg, seed, record=False):
    name = str(v2cfg.animal_model)
    if name not in MODEL_CLASSES:
        raise ValueError(f"unknown animal_model={name!r}")
    return MODEL_CLASSES[name](v2cfg, cfg, seed, cfg.n_sheep, record=record)
