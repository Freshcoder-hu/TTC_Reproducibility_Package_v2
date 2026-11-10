"""M0 animal model: the V1 social model, with C3 finally imposed.

This module is the **Stage-2 hook**.  ``desired_sheep_update`` reproduces the
desired heading/speed computation of ``sheep._advance_sheep`` statement by
statement (a transcription, not a re-derivation, so the numbers are identical),
and returns the *requested* increment instead of integrating it.  Stage 2
replaces that function with M1 (finite obstacle avoidance) and M2
(heterogeneous gains, reaction delay, stimulus-dependent speed target); the
execution layer below it -- the (V2) box projection -- does not change.

Stage 1 ships exactly one model, ``M0``, whose only difference from V1 is that
the (V2) box is enforced (C3 on the velocity vector; C2 unchanged; the C1 upper
bound is preserved by construction).
"""
from __future__ import annotations

import numpy as np

from sheep import _heading
from v2.limits_v2 import project_sheep_increment


def desired_sheep_update(st, cfg, robots_pos, path=None):
    """Requested (heading increment, speed) for every alive animal.

    Transcription of ``sheep._advance_sheep`` up to (but excluding) the
    integration, i.e. everything that decides *what the animal wants*.

    Returns ``(dth, spd_des, dth_raw, flee)``; ``dth`` is already clipped by C2
    exactly as V1 does, ``dth_raw`` is the unclipped increment (used only to
    count how often C2 binds).
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

    # ---- alignment / attraction / path bias ----
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
    want = np.zeros((m, 2))
    rep_act = (rep_n > 0) | flee
    rep_total = rep_dir + rob_dir
    rn = np.linalg.norm(rep_total, axis=1)
    use = rep_act & (rn > 1e-6)
    want[use] = rep_total[use] / np.maximum(rn[use], 1e-6)[:, None]
    other = ~use
    sn = np.linalg.norm(steer, axis=1)
    good = other & (sn > 1e-6)
    want[good] = steer[good] / sn[good, None]
    idle = ~(use | good)
    want[idle] = d[idle]

    # ---- C2 (turn rate), exactly as V1 ----
    cur = head
    target = np.arctan2(want[:, 1], want[:, 0])
    dth_raw = np.angle(np.exp(1j * (target - cur)))
    dth = np.clip(dth_raw, -cfg.turn_max * cfg.dt, cfg.turn_max * cfg.dt)

    # ---- C1 desired speed (band applied to the *desired* value, as V1) ----
    spd = np.where(flee, cfg.v_flee, cfg.v_base)
    spd = np.clip(spd, cfg.v_base, cfg.v_flee)
    return dth, spd, dth_raw, flee


def advance_sheep_m0(st, cfg, v2cfg, robots_pos, path=None, counters=None,
                     projection="joint"):
    """Integrate one ``dt`` with the (V2) box imposed.

    ``projection="joint"``  -- the intersection projection of :mod:`limits_v2`.
    ``projection="sequential"`` -- counterfactual arm for the report only: clip
        the *scalar* speed increment and the turn independently.  It is never a
        V2 default; it exists to measure what "separate clipping" would do.
    """
    alive = st.alive
    if not alive.any():
        return
    pos = st.pos[alive]
    head = st.head[alive]
    spd_old = st.speed[alive]

    dth, spd_des, dth_raw, _flee = desired_sheep_update(st, cfg, robots_pos,
                                                        path)

    if counters is not None:
        c2 = (dth != dth_raw)
        if c2.any():
            counters.hit("bind_C2_sheep", int(c2.sum()))
        counters.hit("opp_sheep_steps", int(len(pos)))

    if projection == "sequential" and v2cfg.enforce_c3:
        A = v2cfg.a_sheep * cfg.dt
        dth_out = dth
        spd_out = spd_old + np.clip(spd_des - spd_old, -A, A)   # scalar cap
        v_old = spd_old[:, None] * _heading(head)
        v_new = spd_out[:, None] * _heading(head + dth_out)
        over = np.linalg.norm(v_new - v_old, axis=1) > A * (1 + v2cfg.viol_tol)
        if counters is not None and over.any():
            counters.hit("viol_C3", int(over.sum()))
            counters.hit("bind_C3_sheep", int(over.sum()))
    else:
        dth_out = dth
        spd_out = spd_des
        if v2cfg.enforce_c3:
            for i in range(len(pos)):
                dth_out[i], spd_out[i], _info = project_sheep_increment(
                    float(spd_old[i]), float(dth[i]), float(spd_des[i]),
                    dt=cfg.dt, a_s=v2cfg.a_sheep, omega_s=cfg.turn_max,
                    s_lo=v2cfg.s_min, s_hi=v2cfg.s_max, eps_v=v2cfg.eps_v,
                    counters=counters)

    head_new = head + dth_out
    pos_new = pos + spd_out[:, None] * _heading(head_new) * cfg.dt

    st.pos[alive] = pos_new
    st.head[alive] = head_new
    st.speed[alive] = spd_out


def sheep_velocity(st):
    """Current velocity vectors of the alive animals (state-derived)."""
    return st.speed[st.alive][:, None] * _heading(st.head[st.alive])


def check_sheep_limits(vel_old, st, cfg, v2cfg, counters):
    """State-based C1/C2/C3 verification after an update (independent of code).

    ``vel_old`` is the velocity vector *before* the update; the check compares
    realised state changes, so it cannot be satisfied by a bug in the
    projection's bookkeeping.
    """
    if not v2cfg.enforce_c3:
        return
    A = v2cfg.a_sheep * cfg.dt
    v_new = sheep_velocity(st)
    dv = np.linalg.norm(v_new - vel_old, axis=1)
    bad = dv > A * (1 + v2cfg.viol_tol)
    if bad.any():
        counters.hit("viol_C3", int(bad.sum()))
    sp = st.speed[st.alive]
    bad = sp > v2cfg.s_max * (1 + v2cfg.viol_tol)
    if bad.any():
        counters.hit("viol_C1_high", int(bad.sum()))
    bad = sp < v2cfg.s_min * (1 - v2cfg.viol_tol)
    if bad.any():
        counters.hit("viol_C1_low", int(bad.sum()))
    # C2 from the state is checked in the simulation loop (needs prev heading).
