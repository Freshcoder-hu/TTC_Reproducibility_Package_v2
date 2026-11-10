"""Continuous (swept) contact detection for the V2 layer.

MOTION MODEL AND EXACTNESS
--------------------------
Both the V1 and the V2 integrators are *a single straight translation per* ``dt``::

    pos <- pos + s * [cos(theta_new), sin(theta_new)] * dt

so inside one integration interval every body moves on a straight segment and
the relative motion of a pair is linear.  The minimum surface clearance over the
interval is therefore available **in closed form** -- no time discretisation
error at all -- by minimising ``|r0 + tau*u|`` on ``tau in [0, 1]``:

    r0 = a0 - b0,  u = (a1 - b1) - r0
    tau* = clip(-(r0.u)/|u|^2, 0, 1)
    d_min = |r0 + tau* u|,      clearance = d_min - r_sum

and, when a contact occurs, its exact entry time solves ``|r0 + tau u| = r_sum``
(the smaller root in [0,1]).

This is ``contact_mode="chord"`` and it is *exact* for the simulated model -- the
resolution is machine precision, which is what acceptance test D verifies
against an analytic grazing scenario.

``contact_mode="arc"`` is the sensitivity check: a differential-drive robot with
turn rate omega does not translate on a chord, it follows an arc

    p(tau) = p0 + (v/omega) * [ sin(th0+omega*tau) - sin(th0),
                                -(cos(th0+omega*tau) - cos(th0)) ]

so the true continuous motion can pass closer to a neighbour than the chord
does.  Arc mode sub-samples the interval into ``contact_substeps`` arcs and
applies the same linear-swept formula on each sub-chord.  Worst-case resolution
is the sagitta of one sub-arc,

    eps = r_p (1 - cos(omega dt / (2 N_sub))) <= 2.0e-4 m

for r_robot = 0.45 m, omega = 3.5 rad/s, dt = 0.05 s, N_sub = 8.  Sheep
translate on a chord in both modes (a heading change never enters their position
update mid-step), so arc mode only changes robot-involving classes.
"""
from __future__ import annotations

import numpy as np

CLASSES = ("rr", "ro", "rs", "ss", "so")


# ---------------------------------------------------------------------------
# closed-form primitives
# ---------------------------------------------------------------------------
def seg_min_clearance(a0, a1, b0, b1):
    """Minimum centre distance over a linearly swept pair (broadcast arrays)."""
    r0 = a0 - b0
    u = (a1 - b1) - r0
    uu = (u * u).sum(axis=-1)
    ru = (r0 * u).sum(axis=-1)
    safe = np.where(uu > 1e-30, uu, 1.0)
    tau = np.where(uu > 1e-30, -ru / safe, 0.0)
    tau = np.clip(tau, 0.0, 1.0)
    r = r0 + tau[..., None] * u
    d = np.sqrt((r * r).sum(axis=-1))
    return d, tau


def seg_first_contact(a0, a1, b0, b1, r_sum):
    """Exact entry time of a linearly swept pair into contact.

    ``tau`` in [0,1] is the smaller root of |r0 + tau u| = r_sum when the pair is
    clear at tau=0 and touching at some tau; ``hit`` is False otherwise
    (including "already overlapping at tau=0", which the clearance reports).
    """
    r0 = a0 - b0
    u = (a1 - b1) - r0
    A = (u * u).sum(axis=-1)
    B = 2.0 * (r0 * u).sum(axis=-1)
    C = (r0 * r0).sum(axis=-1) - r_sum * r_sum
    disc = B * B - 4.0 * A * C
    ok = (A > 1e-30) & (C > 0.0) & (disc >= 0.0)
    sq = np.sqrt(np.where(disc >= 0.0, disc, 0.0))
    denom = np.where(A > 1e-30, 2.0 * A, 1.0)
    tau = (-B - sq) / denom
    hit = ok & (tau >= 0.0) & (tau <= 1.0)
    return np.where(hit, tau, np.inf), hit


def poly_min_clearance(A, B, r_sum):
    """Min clearance over a pair polyline.  A, B: (P, S, 2) -> (P,), (P,)."""
    P, S = A.shape[0], A.shape[1]
    best = np.full(P, np.inf)
    btau = np.zeros(P)
    for k in range(S - 1):
        d, tt = seg_min_clearance(A[:, k], A[:, k + 1], B[:, k], B[:, k + 1])
        upd = d < best
        best = np.where(upd, d, best)
        btau = np.where(upd, (k + tt) / max(S - 1, 1), btau)
    return best - r_sum, btau


def poly_first_contact(A, B, r_sum):
    """First entry tau in [0,1] over a polyline, exact per sub-segment."""
    P, S = A.shape[0], A.shape[1]
    tau = np.full(P, np.inf)
    for k in range(S - 1):
        tt, hit = seg_first_contact(A[:, k], A[:, k + 1],
                                    B[:, k], B[:, k + 1], r_sum)
        tt = np.where(hit, (k + tt) / max(S - 1, 1), np.inf)
        tau = np.minimum(tau, tt)
    return tau


def arc_samples(p0, th0, v, omega, dt, n_sub):
    """Positions along a differential-drive arc, ``n_sub + 1`` samples, (N,S,2)."""
    taus = np.linspace(0.0, 1.0, n_sub + 1)
    out = np.empty((len(p0), n_sub + 1, 2))
    for k, tau in enumerate(taus):
        th = th0 + omega * tau * dt
        out[:, k, 0] = p0[:, 0] + (v / omega) * (np.sin(th) - np.sin(th0))
        out[:, k, 1] = p0[:, 1] - (v / omega) * (np.cos(th) - np.cos(th0))
    return out


def _pairs(n):
    i, j = np.triu_indices(n, k=1)
    return i, j


# ---------------------------------------------------------------------------
# per-run monitor
# ---------------------------------------------------------------------------
class ContactMonitor:
    """Per-class minimum clearance, contact episodes, first contact per pair.

    Read-only w.r.t. the plant, so enabling it cannot change a trajectory
    (acceptance test A relies on that).
    """

    def __init__(self, hard: dict, tol=0.0):
        self.hard = dict(hard)
        self.tol = float(tol)
        self.min_clearance = {c: np.inf for c in CLASSES}
        self.any_contact = {c: False for c in CLASSES}
        self.n_events = {c: 0 for c in CLASSES}
        self.first_time = {c: np.inf for c in CLASSES}
        self.first_pair = {c: None for c in CLASSES}
        self.global_first_time = np.inf
        self.global_first_class = None
        self.global_first_pair = None
        self.new_events = []          # [(key, t_entry)] started in this interval
        self.n_preexisting = {c: 0 for c in CLASSES}
        self._active = set()
        self._first_done = False
        self.n_ticks = 0

    # ------------------------------------------------------------------
    def _record(self, cls, t, dt, clr, A, B, idx_i, idx_j, first=None):
        if clr.size == 0:
            return
        if first is None:
            first = not self._first_done
        m = float(clr.min())
        if m < self.min_clearance[cls]:
            self.min_clearance[cls] = m
        hit = clr <= self.tol
        hs = np.where(hit)[0]
        if len(hs):
            self.any_contact[cls] = True
            tau = poly_first_contact(A[hs], B[hs], self.hard[cls])
            tau = np.where(np.isfinite(tau), tau, 0.0)
            t_pair = t + tau * dt
            for n, k in enumerate(hs):
                key = (cls, int(idx_i[k]), int(idx_j[k]))
                if key not in self._active:
                    self._active.add(key)
                    if first:
                        # Pre-existing overlap at t = 0 is an initial-condition
                        # property of init_flock(), not a contact *event*.
                        self.n_preexisting[cls] += 1
                        continue
                    self.n_events[cls] += 1
                    self.new_events.append((key, float(t_pair[n])))
            k0 = hs[int(np.argmin(t_pair))]
            t_first = float(t_pair.min())
            pair = (int(idx_i[k0]), int(idx_j[k0]))
            if t_first < self.first_time[cls]:
                self.first_time[cls] = t_first
                self.first_pair[cls] = pair
            if t_first < self.global_first_time:
                self.global_first_time = t_first
                self.global_first_class = cls
                self.global_first_pair = pair
        if (~hit).any():
            for k in np.where(~hit)[0]:
                self._active.discard((cls, int(idx_i[k]), int(idx_j[k])))

    # ------------------------------------------------------------------
    def update(self, t, dt, *, rob, sheep, obs, obs_ids=None, rob_head0=None,
               rob_v=None, rob_omega=None, n_sub=1, arc=False):
        """Sample one integration interval.

        rob/sheep/obs are ``(pos_start, pos_end)`` pairs of ``(N,2)`` arrays.
        ``rob_head0``/``rob_v``/``rob_omega`` are required for ``arc=True``.
        """
        self.n_ticks += 1
        self.new_events = []
        first = not self._first_done
        r0, r1 = rob
        s0, s1 = sheep
        o0, o1 = obs
        S = (n_sub + 1) if arc else 2

        A_rob = self._rob_samples(r0, r1, S, rob_head0, rob_v, rob_omega, dt,
                                  arc)
        A_shp = self._lin_samples(s0, s1, S)
        A_obs = self._lin_samples(o0, o1, S)

        M, Ns, K = len(r0), len(s0), len(o0)
        oid = np.arange(K) if obs_ids is None else np.asarray(obs_ids)

        if M > 1:
            i, j = _pairs(M)
            clr, _ = poly_min_clearance(A_rob[i], A_rob[j], self.hard["rr"])
            self._record("rr", t, dt, clr, A_rob[i], A_rob[j], i, j)

        if M and Ns:
            ii = np.repeat(np.arange(M), Ns)
            jj = np.tile(np.arange(Ns), M)
            clr, _ = poly_min_clearance(A_rob[ii], A_shp[jj], self.hard["rs"])
            self._record("rs", t, dt, clr, A_rob[ii], A_shp[jj], ii, jj)

        if M and K:
            ii = np.repeat(np.arange(M), K)
            jj = np.tile(np.arange(K), M)
            clr, _ = poly_min_clearance(A_rob[ii], A_obs[jj], self.hard["ro"])
            self._record("ro", t, dt, clr, A_rob[ii], A_obs[jj], ii, oid[jj])

        if Ns > 1:
            i, j = _pairs(Ns)
            clr, _ = poly_min_clearance(A_shp[i], A_shp[j], self.hard["ss"])
            self._record("ss", t, dt, clr, A_shp[i], A_shp[j], i, j)

        if Ns and K:
            ii = np.repeat(np.arange(Ns), K)
            jj = np.tile(np.arange(K), Ns)
            clr, _ = poly_min_clearance(A_shp[ii], A_obs[jj], self.hard["so"])
            self._record("so", t, dt, clr, A_shp[ii], A_obs[jj], ii, oid[jj])
        self._first_done = True

    # ------------------------------------------------------------------
    @staticmethod
    def _lin_samples(p0, p1, S):
        n = len(p0)
        if S <= 2:
            out = np.empty((n, 2, 2))
            out[:, 0] = p0
            out[:, 1] = p1
            return out
        taus = np.linspace(0.0, 1.0, S)[None, :, None]
        return p0[:, None, :] + taus * (p1 - p0)[:, None, :]

    @staticmethod
    def _rob_samples(p0, p1, S, head0, v, omega, dt, arc):
        n = len(p0)
        if S <= 2 or not arc or head0 is None:
            return ContactMonitor._lin_samples(p0, p1, S)
        out = np.empty((n, S, 2))
        for i in range(n):
            vi = float(v[i])
            wi = float(omega[i]) if omega is not None else 0.0
            if abs(wi) < 1e-9 or vi < 1e-12:
                out[i] = ContactMonitor._lin_samples(
                    p0[i:i + 1], p1[i:i + 1], S)[0]
            else:
                out[i] = arc_samples(p0[i:i + 1], np.array([head0[i]]),
                                     np.array([vi]), wi, dt, S - 1)[0]
                out[i, 0] = p0[i]
                out[i, -1] = p1[i]
        return out

    # ------------------------------------------------------------------
    def summary(self):
        out = {}
        for c in CLASSES:
            out[f"min_surface_clearance_{c}"] = float(self.min_clearance[c])
            out[f"any_contact_{c}"] = bool(self.any_contact[c])
            out[f"n_contact_events_{c}"] = int(self.n_events[c])
            out[f"n_preexisting_contact_{c}"] = int(self.n_preexisting[c])
            out[f"first_contact_time_{c}"] = float(self.first_time[c])
            p = self.first_pair[c]
            out[f"first_contact_pair_{c}"] = "-".join(str(x) for x in p) if p \
                else "?"
        out["first_contact_time"] = float(self.global_first_time)
        out["first_contact_class"] = self.global_first_class or "?"
        p = self.global_first_pair
        out["first_contact_pair"] = "-".join(str(x) for x in p) if p else "?"
        out["contact_ticks"] = int(self.n_ticks)
        return out
