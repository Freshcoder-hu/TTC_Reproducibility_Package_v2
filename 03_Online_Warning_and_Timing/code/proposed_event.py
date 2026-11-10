"""Actionable-event controller: bounded horizon, stable passage and threat-specific side."""
import numpy as np
from proposed_improved import HandoverProposed
from robots import flock_geom, task_direction, drive_command, drive_command_mixed, divert_target, _norm1
_EPS = 1e-6

class EventProposed(HandoverProposed):
    name = 'Ours_event'

    def _ttc(self, view, cfg):
        _, _, matrix = super()._ttc(view, cfg)
        if matrix is None:
            return np.inf, None, None
        matrix = matrix.copy()
        # Keep every value relevant to the recovery exit test.  The shorter
        # actionable horizon below filters candidate EVENTS, not safety risk.
        risk_horizon = max(self.t_pre + 4.0, self.t_rec + self.hyst)
        matrix[matrix > risk_horizon] = np.inf
        if not np.isfinite(matrix).any():
            return np.inf, None, matrix
        i, k = np.unravel_index(np.argmin(matrix), matrix.shape)
        return float(matrix[i,k]), (int(i),int(k)), matrix

    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        ttc_min, pair, Tmat = self._ttc(view, cfg)
        self._ttc_last = ttc_min

        # Select a threat only inside the actionable warning horizon.
        # A momentary infinite TTC does not mean the current encounter has passed.
        cur = getattr(self, "_threat_obj", None)
        cand = (view.obstacles_active[pair[1]]
                if pair is not None and ttc_min <= self.t_pre + 4.0 else None)
        ttc_cur = np.inf
        if cur in view.obstacles_active and Tmat is not None:
            k = view.obstacles_active.index(cur)
            ttc_cur = float(np.min(Tmat[:, k]))
        cleared_cur = self._clearance_info(view, cfg, cur)[0]
        if cur is None or cur not in view.obstacles_active:
            self._threat_obj = cand
        elif cand is not None and cand is not cur:
            if (cleared_cur or ttc_min <= self.t_divert or
                    (np.isfinite(ttc_cur) and ttc_min < 0.6 * ttc_cur)):
                self._threat_obj = cand
        threat = getattr(self, "_threat_obj", None)

        # ---- 风险事件管理：事件切换时只重置"避让侧"；**不重置角色集合**，
        #      由 _select_primary 按集合比较自然更新（避免把事件切换
        #      误计为角色切换，压掉 role_switches 的虚高计数）。----
        if threat is not self._event_threat:
            self._event_threat = threat
            self.event_side = None
            self.divert_sign = None
            if threat is not None:
                s_c, _, _ = view.path.closest(c)
                tang = view.path.tangent_at(s_c)
                nrm = np.array([-tang[1], tang[0]])
                _, lat_o, _ = view.path.closest(threat.pos)
                source_side = float(lat_o) if abs(float(lat_o)) > 0.2 else -float(threat.vel @ nrm)
                self.event_side = -1.0 if source_side > 0 else 1.0
        if threat is None or not threat.is_alive():
            self.event_side = None
            self._t_infeasible = None

        cleared, _, _ = self._clearance_info(view, cfg, threat)
        state = self.state
        t = getattr(view, "t", 0.0)
        # 风险是否仍在逼近（用于抑制 PRE 的边界抖动：只在风险变紧迫时进入 PRE）
        approaching = (not np.isfinite(self._ttc_prev)) or \
            (not np.isfinite(ttc_min)) or (ttc_min <= self._ttc_prev + 0.05)
        self._ttc_prev = ttc_min

        # ---------------- 状态机（含风险抢占 4.13 + 最小驻留） ----------------
        if state == "normal":
            if ttc_min <= self.t_divert:
                self._set_mode("divert", t, force=True)
            elif ttc_min <= self.t_pre and approaching:
                self._set_mode("pre", t)
        elif state == "pre":
            if ttc_min <= self.t_divert:
                self._set_mode("divert", t, force=True)
            elif ttc_min > self.t_pre + self.hyst:
                self._set_mode("normal", t)
        elif state == "divert":
            if cleared and ttc_min > self.t_divert + self.hyst:
                self._set_mode("recover", t)
        elif state == "recover":
            # 风险抢占：新风险优先级 DIVERT > PRE > RECOVER
            if ttc_min <= self.t_divert:
                self._set_mode("divert", t, force=True)
            elif ttc_min <= self.t_pre:
                self._set_mode("pre", t)
            else:
                frac = view.path.inside_frac(view.sheep_pos) \
                    if len(view.sheep_pos) else 0.0
                _, lat_c, _ = view.path.closest(c)
                # 风险解除判定使用独立阈值 T_REC（协议 §4.1：T_REC > T_PRE）
                if (frac >= self.recover_rho and
                        abs(float(lat_c)) <= self.recover_lat and
                        ttc_min > self.t_rec + self.hyst):
                    self._set_mode("normal", t)

        # ---------------- 控制指令 ----------------
        if self.state == "pre":
            # PRE（4.3 节，v4.3 最终设计）= **小幅度横向预置**：
            #   单一风险时，主避让机器人提前把羊群整体向避让侧平移约 45% 避让量，
            #   保持队形凝聚（不驱散），使 DIVERT 阶段只需完成剩余机动；
            #   多风险并存时不做预置（各风险要求的避让方向可能冲突），
            #   仅维持正常推进，避免把羊群推入另一处风险。
            o = threat
            # 多风险门控：若另有障碍物也处于风险视界内，则**不做横向预置**
            # （各风险的避让方向可能冲突，预置会把羊群推向另一处危险）。
            others_risky = False
            if o is not None and Tmat is not None and o in view.obstacles_active:
                k_self = view.obstacles_active.index(o)
                for k in range(Tmat.shape[1]):
                    if k == k_self:
                        continue
                    col = Tmat[:, k]
                    if np.isfinite(col).any() and \
                            float(np.min(col)) < self.t_pre + 5.0:
                        others_risky = True
                        break
            if o is not None and o.is_alive() and not others_risky:
                primary = self._select_primary(view, cfg, o, pair)
                full = max(self.divert_shift,
                           o.radius + cfg.r_sheep + 1.0)
                if self.event_side is None:
                    _, self.event_side, _ = divert_target(
                        view, cfg, c, R_fl, o, shift=full, lookahead=4.0,
                        max_lateral_ratio=self.divert_max_lateral_ratio)
                d_task, _, _ = divert_target(
                    view, cfg, c, R_fl, o,
                    shift=self.pre_shift_ratio * full, lookahead=4.0,
                    max_lateral_ratio=self.divert_max_lateral_ratio * 0.6,
                    side=self.event_side)
                d_normal, _ = task_direction(view, self.pre_lookahead)
                cmd, slots, _ = drive_command_mixed(
                    view, cfg, c, R_fl, d_normal, d_task, primary, "pre",
                    compact=self.pre_compact, cmd_scale=self.pre_cmd_scale)
                cmd = self._apply_transition(cmd, slots, view, cfg, c, R_fl)
                return cmd, "pre"
            d, _ = task_direction(view, self.lookahead)
            cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "pre")
            return cmd, "pre"

        if self.state == "divert" and threat is not None \
                and threat.is_alive():
            primary = self._select_primary(view, cfg, threat, pair)
            # 避让余量随障碍物尺寸自适应：至少让出 (R_o + R_sheep + 1 m)
            shift = max(self.divert_shift,
                        threat.radius + cfg.r_sheep + 1.0)
            d_task, side, _ = divert_target(
                view, cfg, c, R_fl, threat, shift=shift,
                max_lateral_ratio=self.divert_max_lateral_ratio,
                side=self.event_side)
            if self.event_side is None:
                self.event_side = side
            if self.divert_sign is None:
                self.divert_sign = side
            d_normal, _ = task_direction(view, self.lookahead)
            cmd, slots, _ = drive_command_mixed(
                view, cfg, c, R_fl, d_normal, d_task, primary, "divert",
                cmd_scale=self.divert_cmd_scale)
            cmd = self._apply_transition(cmd, slots, view, cfg, c, R_fl)
            return cmd, "divert"

        if self.state == "recover":
            s, _, _ = view.path.closest(c)
            s_a = float(np.clip(s + self.recover_lookahead, 0.0,
                                view.path.total))
            d = _norm1(view.path.point_at(s_a) - c)
            if np.linalg.norm(d) < _EPS:
                d = view.path.tangent_at(s)
            cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "recover")
            return cmd, "recover"

        d, _ = task_direction(view, self.lookahead)
        cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "normal")
        return cmd, "normal"
