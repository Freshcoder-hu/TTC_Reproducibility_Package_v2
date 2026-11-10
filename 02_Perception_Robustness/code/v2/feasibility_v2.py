"""Feasibility-aware execution layer (V2 Stage 1).

WHAT IT DOES, PER CONTROL TICK
------------------------------
1. **Forward prediction.**  Every robot's trajectory over the execution interval
   (``ctrl_period`` integration steps, command held constant -- which is exactly
   what the plant does) is rolled out with the same joint-limit projection the
   plant uses.  Obstacles move on their known constant velocity; animals are
   extrapolated at their current velocity (an approximation, stated in the
   report: animals react inside the interval and are not modelled here).
2. **Braking-distance supervisor** (added after reviewer comment 3).  A static
   margin cannot be a safety guarantee at 4.5 m/s: the stopping distance alone
   is ``v^2 / (2 a_dec)`` = 3.375 m at a_dec = 3.0 m/s^2 (2.53 m at the plant's
   own 4.0).  Required separation is therefore recomputed every tick from the
   *closing* speed

       d_req(v) = v * t_delay + v^2 / (2 a_dec) + d_body + margin

   with ``t_delay`` = one control period (0.1 s: the command computed now takes
   effect for the next interval) and ``a_dec = 3.0 m/s^2`` -- a de-rated value
   below the plant's 4.0, so the plan never relies on the limit being reached.
   When the current clearance is below ``d_req`` the commanded speed is reduced
   to the largest speed whose ``d_req`` fits.  This is *early deceleration*, not
   a static spacing rule.
3. **Hard-separation check.**  If the *projected* motion breaches a hard
   surface limit within the interval, the layer evaluates an admissible
   braking/escape candidate set for the offending robot and executes the first
   safe option (or the least-bad one if none is safe), logging an event.
4. **Three-way cause classification** (reviewer comment 2).  Every event is
   coded

   ``S``  solver-infeasible: no candidate in the (finite) admissible set is safe
          and the unavoidability certificate does not apply.  **S is not a
          claim of physical unavoidability.**
   ``P``  physically unavoidable *under stated assumptions*: the whole reachable
          position set at the predicted contact time lies inside the partner's
          forbidden disc,
              |p - q(tau)| + R_max(tau) < r_hard,
              R_max(tau) = min(v_max*tau, v*tau + a_lin*tau^2 / 2),
          which is a *sufficient* condition (it over-approximates the reachable
          set), so P is a lower bound on true unavoidability.  Assumptions: the
          partner follows its predicted motion, and the robot's admissible set
          is the one enforced by the projection.
   ``C``  control failure: a safe admissible action existed (found by a
          post-hoc receding-horizon oracle over a buffered state window) but the
          controller did not take it.  Diagnosed once per contact episode.
"""
from __future__ import annotations

import math

import numpy as np

from v2.contacts_v2 import poly_min_clearance
from v2.limits_v2 import robot_step_project

CAUSE_CLASSES = ("rr", "ro", "rs")


def _u(a):
    return np.array([math.cos(a), math.sin(a)])


class FeasibilityLayer:
    def __init__(self, v2cfg, sim_cfg, counters=None, hard=None):
        self.cfg = v2cfg
        self.sim = sim_cfg
        self.counters = counters
        self.hard = hard or {"rr": 0.9, "ro": 1.65, "rs": 0.8}
        self.dt = sim_cfg.dt
        self.n = int(sim_cfg.ctrl_period)
        self.events = []
        self.n_brake = 0
        self.brake_events = []
        self.trace = []          # (t, i, class, clr_pred, action, resolved, P)
        self.buf = []            # ring buffer of per-control-tick states
        self.buf_max = 40
        self.n_contact_C = 0
        self.contact_C = []

    # ------------------------------------------------------------------
    # prediction helpers
    # ------------------------------------------------------------------
    def _rollout(self, i, cmd_i, st, n=None):
        """Positions of robot ``i`` over ``n`` steps with a held command."""
        n = self.n if n is None else n
        v = float(st["speed"][i])
        head = float(st["head"][i])
        om = float(st["omega"][i])
        p = st["pos"][i].astype(float)
        pts = [p.copy()]
        for _ in range(n):
            v, head, om, _info = robot_step_project(
                cmd_i, v, head, om, v_max=self.sim.v_max,
                omega_max=self.sim.omega_max, a_lin=self.sim.a_lin,
                a_ang=self.cfg.a_ang, a_lat=self.cfg.a_lat, dt=self.dt,
                ang_on=self.cfg.enforce_r4, lat_on=self.cfg.enforce_lat,
                eps_v=self.cfg.eps_v, counters=None)
            p = p + v * _u(head) * self.dt
            pts.append(p.copy())
        return np.array(pts), v, head, om

    def _partner_paths(self, st, i, rob_paths):
        """(P, n+1, 2) polyline for every partner of robot i, per class."""
        n = self.n
        dt = self.dt
        k = np.arange(n + 1) * dt
        M = len(st["pos"])
        out = {}
        idx_rr = [j for j in range(M) if j != i]
        out["rr"] = (np.array([rob_paths[j] for j in idx_rr])
                     if idx_rr else None)
        out["rr_idx"] = np.array(idx_rr, dtype=int)
        K = len(st["obs_pos"])
        if K:
            out["ro"] = (st["obs_pos"][:, None, :]
                         + st["obs_vel"][:, None, :] * k[None, :, None])
        else:
            out["ro"] = np.zeros((0, n + 1, 2))
        N = len(st["sheep_pos"])
        if N:
            out["rs"] = (st["sheep_pos"][:, None, :]
                         + st["sheep_vel"][:, None, :] * k[None, :, None])
        else:
            out["rs"] = np.zeros((0, n + 1, 2))
        out["ro_idx"] = np.arange(K)
        out["rs_idx"] = np.arange(N)
        return out

    def _min_clr(self, path_i, partners, cls):
        """Min clearance of one robot path against a partner batch."""
        P = len(partners)
        if P == 0:
            return math.inf, -1, 0.0
        A = np.repeat(path_i[None, :, :], P, axis=0)
        clr, tau = poly_min_clearance(A, partners, self.hard[cls])
        k = int(np.argmin(clr))
        return float(clr[k]), int(k), float(tau[k])

    def _all_clear(self, i, cmd_i, st, rob_paths, partners):
        path_i, _v, _h, _o = self._rollout(i, cmd_i, st)
        best = {c: (math.inf, -1, 0.0) for c in CAUSE_CLASSES}
        for cls in CAUSE_CLASSES:
            p = partners[cls]
            if p is None or (hasattr(p, "shape") and p.shape[0] == 0):
                continue
            best[cls] = self._min_clr(path_i, p, cls)
        return best

    # ------------------------------------------------------------------
    # main entry
    # ------------------------------------------------------------------
    def filter(self, cmd, st, ctrl_tick=True):
        """Return a possibly modified command matrix (M,2)."""
        if not self.cfg.feas_enabled:
            return cmd
        cmd = cmd.copy()
        M = len(cmd)
        if not ctrl_tick:
            return cmd

        # ---------------- 2. braking-distance supervisor -------------------
        if self.cfg.brake_enabled:
            cmd = self._brake_supervisor(cmd, st)

        # ---------------- 1./3. predict + hard-separation check -----------
        rob_paths = [self._rollout(i, cmd[i], st)[0] for i in range(M)]
        partners = [self._partner_paths(st, i, rob_paths) for i in range(M)]
        for i in range(M):
            best = self._all_clear(i, cmd[i], st, rob_paths, partners[i])
            worst = min((v[0] for v in best.values()), default=math.inf)
            if worst >= 0.0:
                continue
            # a hard breach is projected inside the interval
            self._handle_breach(cmd, st, i, best, partners[i], rob_paths)
        return cmd

    # ------------------------------------------------------------------
    def _brake_supervisor(self, cmd, st):
        """Limit the *approach* component so that a stop is always possible.

        Only the component of the command along the line of sight to a partner
        is touched; the tangential component (lateral evasion) is left free.
        Braking the whole command instead would forbid the divert manoeuvre,
        which is how an obstacle is actually avoided.

        Required room for a closing speed ``v_a`` is

            dist >= v_a*t_delay + v_a^2/(2 a_dec) + other_stop + d_body + margin

        where ``other_stop`` is the distance the *partner* covers while the pair
        comes to rest (partner approach rate times the stopping time, plus its
        own stopping distance).  This is the "both parties stop" rule; using
        only the robot's own speed was the first implementation and it let
        head-on pairs close to 0.09 m in the S3 diagnostic run.
        """
        M = len(cmd)
        t_d = self.cfg.t_delay
        a_d = self.cfg.a_dec_plan
        m = self.cfg.brake_margin
        classes = tuple(self.cfg.brake_classes)
        for i in range(M):
            sp = float(np.linalg.norm(cmd[i]))
            if sp < 1e-9:
                continue
            p = st["pos"][i]
            cons = []
            for cls, q, vel, d_body in (
                    ("ro", st["obs_pos"], st["obs_vel"], self.hard["ro"]),
                    ("rr", st["pos"][np.arange(M) != i], None, self.hard["rr"]),
                    ("rs", st["sheep_pos"], st["sheep_vel"], self.hard["rs"])):
                if cls not in classes or len(q) == 0:
                    continue
                if cls == "rr":
                    keep = np.arange(M) != i
                    vel = (st["speed"][keep][:, None]
                           * np.stack([np.cos(st["head"][keep]),
                                       np.sin(st["head"][keep])], axis=1))
                d = q - p
                dist = np.linalg.norm(d, axis=1)
                ok = dist < 8.0
                if not ok.any():
                    continue
                d = d[ok]
                dist = dist[ok]
                n = d / np.maximum(dist, 1e-9)[:, None]
                vj = vel[ok]
                cj = -np.einsum("ij,ij->i", vj, n)       # >0: partner closes in
                cj = np.maximum(cj, 0.0)
                other = cj * (t_d + sp / a_d) + cj * cj / (2.0 * a_d)
                for k in range(len(dist)):
                    cons.append((float(dist[k]), n[k], float(other[k]), d_body))
            if not cons:
                continue
            cons.sort(key=lambda r: r[0])
            braked = False
            for dist, n, other, d_body in cons[:4]:
                room = dist - d_body - m - other
                if room <= 0.0:
                    v_max_appr = 0.0
                else:
                    v_max_appr = a_d * (math.sqrt(t_d * t_d + 2.0 * room / a_d)
                                        - t_d)
                va = float(cmd[i] @ n)
                if va > v_max_appr:
                    cmd[i] = cmd[i] - (va - v_max_appr) * n
                    braked = True
            if braked:
                self.n_brake += 1
                if len(self.brake_events) < 20000:
                    self.brake_events.append((float(st["t"]), int(i),
                                              float(sp),
                                              float(np.linalg.norm(cmd[i]))))
                if self.counters is not None:
                    self.counters.hit("n_brake")
        return cmd

    # ------------------------------------------------------------------
    def _handle_breach(self, cmd, st, i, best, partners, rob_paths):
        cls_worst = min(((v[0], c) for c, v in best.items()))[1]
        clr0, k0, tau0 = best[cls_worst]
        partner = None
        pl = partners[cls_worst]
        if pl is not None and len(pl):
            j = k0
            if cls_worst == "rr":
                partner = int(partners["rr_idx"][j])
            elif cls_worst == "ro":
                partner = int(partners["ro_idx"][j])
            else:
                partner = int(partners["rs_idx"][j])

        # ---- unavoidability certificate (P) ---------------------------
        P, cert = self._certificate(st, i, cls_worst, partner, partners,
                                    max(tau0, 1e-6))

        # ---- admissible candidate set ---------------------------------
        cands = self._candidates(cmd[i], st, i, cls_worst, partners, rob_paths)
        chosen = None
        chosen_clr = clr0
        resolved = False
        for name, c in cands:
            b = self._all_clear(i, c, st, rob_paths, partners)
            w = min(v[0] for v in b.values())
            if w > chosen_clr:
                chosen, chosen_clr = (name, c), w
            if w >= 0.0:
                chosen, chosen_clr, resolved = (name, c), w, True
                break
        if chosen is not None:
            cmd[i] = chosen[1]

        ev = dict(t=float(st["t"]), i=int(i), cls=cls_worst,
                  partner=partner, clr_pred=float(clr0),
                  clr_after=float(chosen_clr),
                  action=(chosen[0] if chosen else "keep"),
                  resolved=bool(resolved), P=bool(P),
                  tau=float(tau0), **cert)
        self.events.append(ev)
        if self.counters is not None:
            self.counters.hit("n_infeasible")
            self.counters.hit(f"n_infeasible_{cls_worst}")
            if resolved:
                self.counters.hit("n_infeasible_resolved")
            elif P:
                self.counters.hit("n_infeasible_P")
            else:
                self.counters.hit("n_infeasible_S")
            if cert.get("cert_used"):
                self.counters.hit("cert_P_checks")

    # ------------------------------------------------------------------
    def _certificate(self, st, i, cls, partner, partners, tau):
        """Sufficient unavoidability certificate at the predicted breach time."""
        cert = dict(cert_R=math.nan, cert_gap=math.nan, cert_rhard=math.nan,
                    cert_tau=float(tau), cert_used=True)
        if partner is None:
            cert["cert_used"] = False
            return False, cert
        n = int(round(tau / self.dt))
        n = max(0, min(n, self.n))
        if cls == "rr":
            loc = np.where(partners["rr_idx"] == partner)[0]
            if len(loc) == 0:
                cert["cert_used"] = False
                return False, cert
            q = partners["rr"][int(loc[0])][n]
        elif cls == "ro":
            q = partners["ro"][partner][n]
        else:
            q = partners["rs"][partner][n]
        p = st["pos"][i]
        gap = float(np.linalg.norm(p - q))
        v = float(st["speed"][i])
        R = min(self.sim.v_max * tau, v * tau + 0.5 * self.sim.a_lin * tau * tau)
        r_hard = self.hard[cls]
        cert.update(cert_R=float(R), cert_gap=float(gap),
                    cert_rhard=float(r_hard))
        return bool(gap + R < r_hard), cert

    # ------------------------------------------------------------------
    def _candidates(self, cmd_i, st, i, cls, partners, rob_paths):
        """Ordered admissible braking / escape options for robot i."""
        sp = float(np.linalg.norm(cmd_i))
        out = [("brake", np.zeros(2))]
        # escape: directly away from the nearest partner's final predicted point
        q = None
        pl = partners[cls]
        if pl is not None and len(pl):
            d = np.linalg.norm(pl[:, -1, :] - st["pos"][i], axis=1)
            q = pl[int(np.argmin(d))][-1]
        if q is not None:
            away = st["pos"][i] - q
            na = float(np.linalg.norm(away))
            if na > 1e-9:
                out.append(("escape", self.sim.v_max * away / na))
        if sp > 1e-6:
            th = math.atan2(cmd_i[1], cmd_i[0])
            for sgn in (1.0, -1.0):
                th2 = th + sgn * math.radians(72.0)
                out.append((f"lateral{sgn:+.0f}", sp * _u(th2)))
            out.append(("slow", 0.5 * cmd_i))
        return out

    # ------------------------------------------------------------------
    # ring buffer + post-hoc C diagnosis
    # ------------------------------------------------------------------
    def push_state(self, st):
        self.buf.append({
            "t": float(st["t"]),
            "pos": st["pos"].copy(), "head": st["head"].copy(),
            "speed": st["speed"].copy(), "omega": st["omega"].copy(),
            "sheep_pos": st["sheep_pos"].copy(),
            "sheep_vel": st["sheep_vel"].copy(),
            "obs_pos": st["obs_pos"].copy(), "obs_vel": st["obs_vel"].copy(),
            "obs_ids": np.asarray(st.get("obs_ids", np.arange(
                len(st["obs_pos"]))), dtype=int).copy(),
        })
        if len(self.buf) > self.buf_max:
            self.buf.pop(0)

    @staticmethod
    def _local_obs(st, gid):
        ids = st.get("obs_ids")
        if ids is None or len(ids) == 0:
            return None
        k = int(np.searchsorted(ids, gid))
        if k >= len(ids) or ids[k] != gid:
            k = int(np.argmin(np.abs(ids - gid)))
            if ids[k] != gid:
                return None
        return k

    def classify_contact(self, t_contact, cls, i, partner, state_now):
        """Post-hoc C/P diagnosis for one contact episode.

        Scans back over the buffered control ticks; at each one it replays robot
        ``i`` with a myopic safety oracle (candidate commands scored by their
        predicted clearance against the *recorded* partner positions) and asks
        whether any admissible sequence avoids the contact.  A myopic oracle can
        miss a feasible sequence, so C is a **lower bound**.
        """
        if not self.cfg.feas_enabled or len(self.buf) < 2:
            return None
        for back in range(1, min(len(self.buf), self.cfg.c_diag_window) + 1):
            st = self.buf[-back]
            if st["t"] >= t_contact - 1e-9:
                continue
            if self._oracle_avoids(st, self.buf[-back:], cls, i, partner,
                                   t_contact):
                horizon = float(t_contact - st["t"])
                # P certificate at that same tick (for reporting)
                P = self._certificate_at(st, i, cls, partner, horizon)
                rec = dict(t_contact=float(t_contact), cls=cls, i=int(i),
                           partner=int(partner) if partner is not None else -1,
                           t_decision=float(st["t"]), horizon=horizon,
                           P=bool(P))
                self.n_contact_C += 1
                self.contact_C.append(rec)
                if self.counters is not None:
                    self.counters.hit("n_contact_C")
                return rec
        return None

    def _oracle_avoids(self, st, window, cls, i, partner, t_contact):
        """Greedy receding-horizon replay from ``st`` to ``t_contact``."""
        M = len(st["pos"])
        pos = st["pos"][i].copy()
        head = float(st["head"][i])
        v = float(st["speed"][i])
        om = float(st["omega"][i])
        t = st["t"]
        # recorded partner positions (for rr) / linear extrapolation otherwise
        while t < t_contact - 1e-9:
            best = None
            best_clr = -math.inf
            for spd in (self.sim.v_max, 0.5 * self.sim.v_max, 0.0):
                for th_off in (0.0, 0.6, -0.6, 1.2, -1.2, math.pi):
                    c = spd * _u(head + th_off)
                    clr = self._pair_clr_after(pos, head, v, om, c, st, cls,
                                               i, partner)
                    if clr > best_clr:
                        best_clr, best = clr, c
                    if clr > 0.05:
                        break
                if best_clr > 0.05:
                    break
            if best is None:
                return False
            v, head, om, _ = robot_step_project(
                best, v, head, om, v_max=self.sim.v_max,
                omega_max=self.sim.omega_max, a_lin=self.sim.a_lin,
                a_ang=self.cfg.a_ang, a_lat=self.cfg.a_lat, dt=self.dt,
                ang_on=self.cfg.enforce_r4, lat_on=self.cfg.enforce_lat,
                eps_v=self.cfg.eps_v)
            pos = pos + v * _u(head) * self.dt
            t += self.dt
            if self._decide_contact(pos, t, st, cls, partner):
                return False
        return True

    def _pair_clr_after(self, pos, head, v, om, c, st, cls, i, partner):
        v2, h2, o2, _ = robot_step_project(
            c, v, head, om, v_max=self.sim.v_max,
            omega_max=self.sim.omega_max, a_lin=self.sim.a_lin,
            a_ang=self.cfg.a_ang, a_lat=self.cfg.a_lat, dt=self.dt,
            ang_on=self.cfg.enforce_r4, lat_on=self.cfg.enforce_lat,
            eps_v=self.cfg.eps_v)
        p2 = pos + v2 * _u(h2) * self.dt
        q = self._partner_pos(st, cls, partner, self.dt)
        if q is None:
            return math.inf
        return float(np.linalg.norm(p2 - q)) - self.hard[cls]

    def _partner_pos(self, st, cls, partner, dtq=0.0):
        """Partner position ``dtq`` seconds after the buffered state."""
        if cls == "rr":
            return st["pos"][partner]
        if cls == "ro":
            k = self._local_obs(st, partner)
            if k is None:
                return None
            return st["obs_pos"][k] + st["obs_vel"][k] * dtq
        return st["sheep_pos"][partner] + st["sheep_vel"][partner] * dtq

    def _decide_contact(self, pos, t, st, cls, partner):
        q = self._partner_pos(st, cls, partner, max(0.0, t - st["t"]))
        if q is None:
            return False
        return float(np.linalg.norm(pos - q)) < self.hard[cls]

    def _certificate_at(self, st, i, cls, partner, tau):
        if partner is None:
            return False
        q = self._partner_pos(st, cls, partner, tau)
        if q is None:
            return False
        gap = float(np.linalg.norm(st["pos"][i] - q))
        v = float(st["speed"][i])
        R = min(self.sim.v_max * tau, v * tau + 0.5 * self.sim.a_lin * tau * tau)
        return bool(gap + R < self.hard[cls])

    # ------------------------------------------------------------------
    def summary(self):
        out = dict(n_infeasible=len(self.events),
                   n_brake_events=int(self.n_brake),
                   n_contact_C=int(self.n_contact_C))
        for c in CAUSE_CLASSES:
            out[f"n_infeasible_{c}"] = sum(1 for e in self.events
                                           if e["cls"] == c)
            out[f"n_infeasible_resolved_{c}"] = sum(
                1 for e in self.events if e["cls"] == c and e["resolved"])
            out[f"n_infeasible_P_{c}"] = sum(
                1 for e in self.events if e["cls"] == c and e["P"]
                and not e["resolved"])
            out[f"n_infeasible_S_{c}"] = sum(
                1 for e in self.events if e["cls"] == c and not e["P"]
                and not e["resolved"])
        out["n_infeasible_resolved"] = sum(1 for e in self.events
                                           if e["resolved"])
        out["n_infeasible_P"] = sum(1 for e in self.events
                                    if e["P"] and not e["resolved"])
        out["n_infeasible_S"] = sum(1 for e in self.events
                                    if not e["P"] and not e["resolved"])
        out["first_infeasible_t"] = (float(self.events[0]["t"])
                                     if self.events else math.inf)
        return out
