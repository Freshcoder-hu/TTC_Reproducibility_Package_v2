"""Stage-2 rollout predictor: the V1 native predictor, re-expressed in NumPy.

WHY A SECOND PREDICTOR
----------------------
The Stage-1 rollout predictor is ``_rollout.dll`` (``rollout.cpp``) and its
animal step is a hard-coded transcription of the V1 Couzin model -- it *is* M0.
"The predictor may use a different animal model from the one the simulator
integrates" therefore cannot be done by configuration with the DLL: the animal
model has to live in code that can select one.  This module is a transcription
of ``rollout.cpp`` in which the animal step is dispatched to M0 / M1 / M2, and
every other component (slot geometry, spacing projection, safety filter,
unicycle integration, penalties, 7-column output) is kept identical.

**The predictor is still only a prediction model**: it never replaces the
evaluator, it receives only the controller's available view, and the candidate
offsets are the controller's own (``proposed_rollout``).

FIDELITY
--------
For ``predictor_model="M0"`` the two engines solve the same equations with the
same integration; they differ only in floating-point summation order (the DLL
sums in a C++ loop, NumPy sums with pairwise blocking).  ``v2/acceptance_s2.py``
measures the agreement and reports it (~1e-15 relative, not bit equality).

FAITHFULLY KEPT QUIRKS OF THE V1 PREDICTOR (not introduced here)
----------------------------------------------------------------
* the attraction term is ``att / max(na, 1) - p``, so an animal with no
  attraction-zone neighbour is pulled toward the *origin* (rollout.cpp does
  exactly this).  It is kept so the two engines stay comparable; it is a
  predictor artefact, not a simulator behaviour.
* the predicted animals do not run the (V2) box: the rollout is a coarse
  prediction (``prediction_dt`` = 0.15 s), as in V1.

PREDICTOR-SIDE SIMPLIFICATIONS (documented in the report)
---------------------------------------------------------
* the animal obstacle-sensing *delay* is not modelled inside the rollout (the
  rollout uses the obstacle state as observed now, propagated at constant
  velocity); the M1/M2 *dynamics* are modelled.
* M2's per-animal gain is taken at its mean (mean-field prediction).
* the layered speed policy is applied to the predicted robot commands when
  ``predictor_speed_policy`` is on (it is a controller rule, so the predictor
  must obey it too); otherwise the predicted robots run at ``v_max``.
"""
from __future__ import annotations

import numpy as np

from v2.animal_models import obstacle_weight

_TWO_PI = 2.0 * np.pi


def _wrap(a):
    return (a + np.pi) % _TWO_PI - np.pi


def _unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return np.where(n > 1e-9, v / np.maximum(n, 1e-12), 0.0)


# ---------------------------------------------------------------------------
# vectorised path queries (same arithmetic as path.Path, batched)
# ---------------------------------------------------------------------------
def _seg_index(path, s):
    s = np.clip(s, 0.0, path.total)
    k = np.searchsorted(path.cum, s, side="right") - 1
    return np.clip(k, 0, len(path.verts) - 2)


def _tangent_batch(path, s):
    return path.tang[_seg_index(path, s)]


def _point_batch(path, s):
    s = np.clip(s, 0.0, path.total)
    k = _seg_index(path, s)
    t = (s - path.cum[k]) / np.maximum(path.seg_len[k], 1e-9)
    return path.verts[k] + t[..., None] * (path.verts[k + 1] - path.verts[k])


def _halfwidth_batch(path, s):
    s = np.clip(s, 0.0, path.total)
    k = _seg_index(path, s)
    t = (s - path.cum[k]) / np.maximum(path.seg_len[k], 1e-9)
    return path.halfwidth[k] * (1 - t) + path.halfwidth[k + 1] * t


def _radius95(p, c):
    """``radius95`` of rollout.cpp: 95th percentile, floor 1.6 m.  p (A,N,2)."""
    r = np.linalg.norm(p - c[:, None, :], axis=2)
    out = np.empty(r.shape[0])
    for i in range(r.shape[0]):
        q = np.sort(r[i])
        k = 0.95 * (len(q) - 1)
        i0 = int(k)
        i1 = min(i0 + 1, len(q) - 1)
        out[i] = max(1.6, q[i0] * (1 - (k - i0)) + q[i1] * (k - i0))
    return out


def _speed_policy_cap(a_lat, v_max, L, h, h_eps=0.05):
    """v <= sqrt(a_lat * r_req), r_req = (L^2 + h^2) / (2 h)."""
    ah = np.abs(h)
    r = np.where(ah > h_eps,
                 (L * L + ah * ah) / (2.0 * np.maximum(ah, 1e-12)), np.inf)
    return np.minimum(v_max, np.sqrt(a_lat * np.maximum(r, 0.0)))


# ---------------------------------------------------------------------------
def evaluate_rollouts(sheep_pos, sheep_vel, robot_pos, robot_head, robot_speed,
                      obs_pos, obs_vel, obs_radius, path, pars, offsets,
                      dt=0.15, steps=80, predictor_model="M0",
                      mcfg=None, speed_policy=False, a_lat=2.0):
    """Batched rollouts, one row per candidate offset.

    ``pars`` is the same 18-element vector as ``rollout.cpp``:
    ``v_base, v_flee, turn, r_rep, r_ori, r_att, r_avo, avo_k, goal_bias,
    v_max, omega_max, a_lin, kp, d_rr_safe, d_ro_safe, slot_gap, r_sheep,
    lookahead``.

    Returns ``out`` ``(A,7)`` = ``[contacted fraction, pressure, outside,
    centre penalty, spread penalty, progress, worst margin]``.
    """
    (v_base, v_flee, turn_max, r_rep, r_ori, r_att, r_avo, avo_k, goal_bias,
     v_max, omega_max, a_lin, kp, d_rr, d_ro, slot_gap, r_sheep,
     lookahead) = [float(x) for x in pars]

    sp = np.asarray(sheep_pos, dtype=float)
    sv = np.asarray(sheep_vel, dtype=float)
    rp = np.asarray(robot_pos, dtype=float)
    op0 = np.asarray(obs_pos, dtype=float).reshape(-1, 2)
    ov = np.asarray(obs_vel, dtype=float).reshape(-1, 2)
    orad = np.asarray(obs_radius, dtype=float).reshape(-1)
    offs = np.asarray(offsets, dtype=float)
    A, N, M, K = len(offs), len(sp), len(rp), len(op0)
    if N == 0:
        return np.zeros((A, 7))

    head0 = np.arctan2(sv[:, 1], sv[:, 0])
    p = np.repeat(sp[None, :, :], A, axis=0)
    head = np.repeat(head0[None, :], A, axis=0)
    rob = np.repeat(rp[None, :, :], A, axis=0)
    rh = np.repeat(np.asarray(robot_head, dtype=float)[None, :], A, axis=0)
    rv = np.repeat(np.asarray(robot_speed, dtype=float)[None, :], A, axis=0)
    op = np.repeat(op0[None, :, :], A, axis=0)

    use_obs = predictor_model in ("M1", "M2") and mcfg is not None and K > 0
    margin = np.full((A, N), np.inf)
    outside = np.zeros(A)
    centre_pen = np.zeros(A)
    spread_pen = np.zeros(A)

    eye_N = np.eye(N, dtype=bool)
    s_init, _, _ = path.closest(p.mean(axis=1))
    s_init = np.asarray(s_init, dtype=float)

    for _tick in range(int(steps)):
        c = p.mean(axis=1)                                   # (A,2)
        R = _radius95(p, c)
        s_c, lat_c, w_c = path.closest(c)
        s_c = np.asarray(s_c, dtype=float)

        ahead = np.where(np.abs(offs) < 1e-8, 6.0, lookahead)
        s_a = np.clip(s_c + ahead, 0.0, path.total)
        tang = _tangent_batch(path, s_a)                      # (A,2)
        targ = _point_batch(path, s_a)
        nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
        dvec = targ + nrm * offs[:, None] - c                  # fleet intent
        direction = _unit(dvec)
        perp = np.stack([-direction[:, 1], direction[:, 0]], axis=1)

        # ---- slot geometry ----
        j = np.arange(M)
        phi = (np.pi + (-0.95 + 1.9 * j / max(1, M - 1))) if M > 1 \
            else np.full(1, np.pi)
        rad = R + 0.62 * r_avo
        slot = (c[:, None, :]
                + direction[:, None, :] * np.cos(phi)[None, :, None]
                * rad[:, None, None]
                + perp[:, None, :] * np.sin(phi)[None, :, None]
                * rad[:, None, None])
        limit = d_rr + slot_gap
        for _it in range(6):
            moved = False
            for i in range(M):
                for k in range(i + 1, M):
                    d = slot[:, i] - slot[:, k]
                    dist = np.linalg.norm(d, axis=1)
                    bad = dist < limit
                    if bad.any():
                        d2 = np.where(dist < 1e-6, 1.0, dist)
                        dd = np.where((dist < 1e-6)[:, None], direction, d)
                        corr = dd * ((limit - d2) / (2.0 * d2))[:, None]
                        slot[bad, i] += corr[bad]
                        slot[bad, k] -= corr[bad]
                        moved = True
            if not moved:
                break

        # ---- commanded velocity (+ layered speed policy) ----
        cmd = (slot - rob) * kp
        cap = np.full(A, v_max)
        if speed_policy:
            h = np.einsum("ai,ai->a", dvec, nrm)
            cap = _speed_policy_cap(float(a_lat), v_max, ahead, h)
        n = np.linalg.norm(cmd, axis=2)
        over = n > cap[:, None]
        cmd = np.where(over[:, :, None],
                       cmd * (cap[:, None] / np.maximum(n, 1e-12))[:, :, None],
                       cmd)

        # ---- safety filter (rollout.cpp) ----
        for _it in range(4):
            nxt = rob + cmd * dt
            for i in range(M):
                for k in range(i + 1, M):
                    d = nxt[:, i] - nxt[:, k]
                    dist = np.linalg.norm(d, axis=1)
                    bad = dist < d_rr
                    if bad.any():
                        d2 = np.where(dist < 1e-6, 1.0, dist)
                        dd = np.where((dist < 1e-6)[:, None],
                                      np.array([1.0, 0.0]), d)
                        corr = _unit(dd) * ((d_rr - d2) / (2.0 * dt))[:, None]
                        cmd[bad, i] += corr[bad]
                        cmd[bad, k] -= corr[bad]
        for i in range(M):
            for k in range(i + 1, M):
                d = rob[:, i] - rob[:, k]
                dist = np.linalg.norm(d, axis=1)
                ok = dist > 1e-6
                if not ok.any():
                    continue
                nn = d / np.maximum(dist, 1e-12)[:, None]
                close = np.einsum("ai,ai->a", cmd[:, i] - cmd[:, k], nn)
                act = ok & (dist < d_rr) & (close < 0)
                if act.any():
                    corr = 0.5 * close[:, None] * nn
                    cmd[act, i] -= corr[act]
                    cmd[act, k] += corr[act]
        if K:
            for jj in range(M):
                dd = rob[:, jj][:, None, :] - op
                dist = np.linalg.norm(dd, axis=2)
                kk = np.argmin(dist, axis=1)
                near = dist[np.arange(A), kk]
                bad = near < d_ro
                if bad.any():
                    z = dd[np.arange(A), kk]
                    cmd[bad, jj] += (_unit(z) * ((d_ro - near) * 2.0)[:, None])[bad]
        n = np.linalg.norm(cmd, axis=2)
        over = n > v_max
        cmd = np.where(over[:, :, None],
                       cmd * (v_max / np.maximum(n, 1e-12))[:, :, None], cmd)

        # ---- unicycle robots ----
        n = np.linalg.norm(cmd, axis=2)
        tgt = np.arctan2(cmd[:, :, 1], cmd[:, :, 0])
        rh = rh + np.clip(_wrap(tgt - rh), -omega_max * dt, omega_max * dt)
        vd = np.where(n > 1e-3, n * np.maximum(0.0, np.cos(_wrap(tgt - rh))), 0.0)
        rv = np.clip(rv + np.clip(vd - rv, -a_lin * dt, a_lin * dt), 0.0, v_max)
        rob = rob + np.stack([np.cos(rh), np.sin(rh)], axis=2) \
            * (rv * dt)[:, :, None]

        # ---- animals ----
        dp = p[:, :, None, :] - p[:, None, :, :]
        dist = np.linalg.norm(dp, axis=3)
        dist = np.where(eye_N[None, :, :], np.inf, dist)
        rep_m = dist < r_rep
        ali_m = (dist >= r_rep) & (dist < r_ori)
        att_m = (dist >= r_ori) & (dist < r_att)
        u = dp / np.maximum(dist[..., None], 1e-6)
        rep = np.where(rep_m[..., None], u, 0.0).sum(axis=2)
        nr = rep_m.sum(axis=2)
        hu = np.stack([np.cos(head), np.sin(head)], axis=2)
        ali = np.einsum("anm,amj->anj", ali_m.astype(float), hu)
        att = np.einsum("anm,amj->anj", att_m.astype(float), p)
        na = att_m.sum(axis=2)

        dr = p[:, :, None, :] - rob[:, None, :, :]
        drd = np.linalg.norm(dr, axis=3)
        rm = drd < r_avo
        wr = np.where(rm, avo_k * (1.0 - drd / r_avo) / np.maximum(drd, 1e-6),
                      0.0)
        rep_rob = (dr * wr[..., None]).sum(axis=2)
        robot_n = rm.sum(axis=2)

        f_obs = np.zeros((A, N, 2))
        if use_obs:
            do = p[:, :, None, :] - op[:, None, :, :]
            disto = np.linalg.norm(do, axis=3)
            gap = disto - r_sheep - orad[None, None, :]
            w = obstacle_weight(gap, mcfg.m1_sense_range, mcfg.m1_decay,
                                mcfg.m1_decay_len, mcfg.m1_gain)
            f_obs = ((do / np.maximum(disto, 1e-6)[..., None]) * w[..., None]
                     ).sum(axis=2)

        rep_total = rep + rep_rob
        rn = np.linalg.norm(rep_total, axis=2)
        sep = ((nr > 0) | (robot_n > 0)) & (rn > 1e-6)
        us = rep_total / np.maximum(rn, 1e-12)[:, :, None]
        par = np.einsum("anj,anj->an", f_obs, us)
        a_use = np.where((par < 0)[..., None], f_obs - par[..., None] * us, f_obs)
        summ = us + a_use
        sn2 = np.linalg.norm(summ, axis=2)
        want_sep = np.where(sn2[..., None] > 1e-9,
                            summ / np.maximum(sn2, 1e-12)[..., None], us)

        # soft branch: weighted sum of Couzin + path + obstacle
        flat = p.reshape(-1, 2)
        s_flat, _, _ = path.closest(flat)
        tl = _tangent_batch(path, np.asarray(s_flat, dtype=float)
                            ).reshape(A, N, 2)
        steer = (0.6 * ali + 0.4 * (att / np.maximum(na, 1)[..., None] - p)
                 + tl * goal_bias + f_obs)
        sn = np.linalg.norm(steer, axis=2)
        want_soft = np.where(sn[..., None] > 1e-6,
                             steer / np.maximum(sn, 1e-12)[..., None], hu)
        want = np.where(sep[..., None], want_sep, want_soft)

        head = head + np.clip(_wrap(np.arctan2(want[:, :, 1], want[:, :, 0])
                                    - head), -turn_max * dt, turn_max * dt)
        spd = np.where(robot_n > 0, v_flee, v_base)
        p = p + np.stack([np.cos(head), np.sin(head)], axis=2) \
            * (spd * dt)[..., None]
        op = op + ov[None, :, :] * dt

        # ---- penalties ----
        #   NOTE: path.Path.closest returns (s, lateral, DISTANCE), not the
        #   half-width (rollout.cpp's closest() has a different signature), so
        #   the half-width must be interpolated from s explicitly.
        c = p.mean(axis=1)
        s_c2, lat_c2, _ = path.closest(c)
        w_c2 = _halfwidth_batch(path, np.asarray(s_c2, dtype=float))
        centre_pen += (np.asarray(lat_c2, dtype=float) ** 2
                       / np.maximum(w_c2 ** 2, 1e-9))
        s_all, lat_all, _ = path.closest(p.reshape(-1, 2))
        lat_a = np.asarray(lat_all, dtype=float).reshape(A, N)
        w_a = _halfwidth_batch(path, np.asarray(s_all, dtype=float)
                               ).reshape(A, N)
        ex = np.maximum(0.0, np.abs(lat_a) - w_a + r_sheep)
        outside += (ex * ex).sum(axis=1)
        rr = np.linalg.norm(p - c[:, None, :], axis=2)
        spread_pen += (np.maximum(0.0, rr - 12.0) ** 2).sum(axis=1)
        if K:
            dok = np.linalg.norm(p[:, :, None, :] - op[:, None, :, :], axis=3)
            m_ik = (dok - r_sheep - orad[None, None, :]).min(axis=2)
            margin = np.minimum(margin, m_ik)

    ns = max(int(steps), 1)
    contact = (margin < 0).sum(axis=1) / N
    pressure = np.power(np.maximum(0.0, 0.8 - margin), 2).mean(axis=1)
    worst = margin.min(axis=1)
    s_end, _, _ = path.closest(p.mean(axis=1))
    progress = np.asarray(s_end, dtype=float) - s_init
    return np.stack([contact, pressure, outside / (N * ns), centre_pen / ns,
                     spread_pen / (N * ns), progress, worst], axis=1)
