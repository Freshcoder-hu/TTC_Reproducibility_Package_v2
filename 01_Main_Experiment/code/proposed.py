"""Ours：TTC 驱动的沿路径防碰撞控制器（论文第 4 章）。

控制链：TTC → PRE → DIVERT → RECOVER，并加入：
- 局部风险聚合（仅使用局部观测 ∪ 邻近共享得到的羊群状态，见 simulation.View）；
- 多机器人角色协调（4.9）：只选部分机器人作为主要 DIVERT 执行者，
  其余机器人继续推进/凝聚/路径维持；
- 机器人自身安全（4.10）：机器人—障碍物、机器人—机器人安全优先；
- 风险抢占（4.13）：RECOVER 期间出现新紧迫风险 → DIVERT；PRE 优先于 RECOVER。

状态机只使用 TTC 滞回阈值（不做 jitter/debounce，与初稿一致）。
"""
import numpy as np

from robots import (flock_geom, task_direction, drive_command,
                    drive_command_mixed, divert_target, _norm1)
from ttc import ttc_matrix, collision_point

_EPS = 1e-6


class Proposed:
    name = "Ours"

    def __init__(self, t_pre=12.0, t_divert=6.0, hyst=2.5, lookahead=6.0,
                 pre_lookahead=3.0, pre_compact=0.94, pre_cmd_scale=0.85,
                 pre_shift_ratio=0.45,
                 divert_shift=3.0, divert_cmd_scale=0.85,
                 divert_max_lateral_ratio=0.90, recover_lookahead=5.0,
                 recover_rho=0.88, recover_lat=3.0, n_primary=None,
                 mode_dwell=1.0, role_dwell=4.0,
                 role_transition_speed_scale=0.5,
                 role_transition_time=2.0,
                 detour_margin=0.8, t_rec=None):
        if not (t_pre > t_divert > 0):
            raise ValueError("必须满足 t_pre > t_divert > 0")
        self.t_pre = float(t_pre)
        self.t_divert = float(t_divert)
        # 风险解除阈值 T_REC：独立于 T_PRE（协议 §4.1），用于 RECOVER→NORMAL 判定
        self.t_rec = float(t_rec) if t_rec is not None else float(t_pre) + 3.0
        if not (self.t_rec > self.t_pre):
            raise ValueError("必须满足 t_rec > t_pre")
        self.hyst = float(hyst)
        self.lookahead = float(lookahead)
        self.pre_lookahead = float(pre_lookahead)
        self.pre_compact = float(pre_compact)
        self.pre_cmd_scale = float(pre_cmd_scale)
        self.pre_shift_ratio = float(pre_shift_ratio)
        self.divert_cmd_scale = float(divert_cmd_scale)
        self.divert_shift = float(divert_shift)
        self.divert_max_lateral_ratio = float(divert_max_lateral_ratio)
        self.recover_lookahead = float(recover_lookahead)
        self.recover_rho = float(recover_rho)
        self.recover_lat = float(recover_lat)
        self.n_primary = n_primary          # None -> max(2, M//3)

        self.state = "normal"
        self.switch_count = 0
        self.role_switch_count = 0
        self.divert_sign = None
        self.primary_idx = []
        self._last_threat_id = None
        self.mode_dwell = float(mode_dwell)
        self.role_dwell = float(role_dwell)
        self._t_mode = -1e9
        self._t_role = -1e9
        self._ttc_prev = np.inf             # 用于"风险是否在逼近"的判别
        self._event_threat = None           # 当前风险事件对应的障碍物
        self.event_side = None              # 事件内锁定的避让侧（PRE/DIVERT 一致）
        self._t_infeasible = None
        self.role_replace_delay = 3.0       # 不可行持续多久才替换主避让机器人
        self.role_hard_margin = 1.0         # 硬不可行判据：d_obs < d_ro_safe*margin
        # 角色接替的安全过渡（4.14，支撑机制，不新增核心状态）
        self.role_transition_speed_scale = float(role_transition_speed_scale)
        self.role_transition_time = float(role_transition_time)
        self.detour_margin = float(detour_margin)
        self._trans = {}                    # robot -> 角色变化时刻

    # ------------------------------------------------------------------
    def _apply_transition(self, cmd, slots, view, cfg, c, R_fl):
        """角色接替过渡约束（本轮唯一新增机制）：
        1) 刚换组的机器人限速（v_max 与转向幅度同时下降）；
        2) 若"机器人→新槽位"的直线会穿过羊群核心区域，则先绕外侧中间点，
           禁止从羊群内部直穿（仅作用于换组过渡期，不影响正常 DIVERT）。
        """
        t = getattr(view, "t", 0.0)
        r_core = R_fl + self.detour_margin
        for j, t_chg in list(self._trans.items()):
            if t - t_chg > self.role_transition_time:
                self._trans.pop(j, None)
                continue
            pr = view.robot_pos[j]
            pt = slots[j]
            # 点到线段距离（判断是否穿过羊群核心）
            seg = pt - pr
            L = float(np.linalg.norm(seg))
            crossing = False
            if L > 1e-6:
                u = seg / L
                s = float(np.clip((c - pr) @ u, 0.0, L))
                closest = pr + s * u
                crossing = float(np.linalg.norm(closest - c)) < r_core
            if crossing:
                radial = pr - c
                if float(np.linalg.norm(radial)) < 1e-6:
                    radial = np.array([1.0, 0.0])
                p_way = c + (r_core + 1.0) * (radial /
                                              float(np.linalg.norm(radial)))
                cmd[j] = cfg.kp * (p_way - pr) * \
                    self.role_transition_speed_scale
            else:
                cmd[j] *= self.role_transition_speed_scale
        return cmd

    # ------------------------------------------------------------------
    def _set_mode(self, state, t=0.0, force=False):
        """状态切换（带最小驻留时间 mode_dwell，抑制频繁切换 5.13.8）。"""
        if state == self.state:
            return
        if not force and (t - self._t_mode) < self.mode_dwell:
            return
        self.state = state
        self.switch_count += 1
        self._t_mode = t
        if state != "divert":
            self.divert_sign = None

    def _ttc(self, view, cfg):
        """局部风险聚合：对可用（观测∪共享）羊群状态计算 TTC 矩阵。"""
        if view.obs_active_n == 0 or len(view.sheep_pos) == 0:
            return np.inf, None, None
        Tmat, _, _ = ttc_matrix(
            view.sheep_pos, view.sheep_vel, view.obs_pos, view.obs_vel,
            cfg.r_sheep + cfg.r_obs,
            horizon=max(self.t_pre + 4.0, 10.0),
            a_max=cfg.a_obs_max,
            mode="conservative" if cfg.a_obs_max > 0 else "analytic")
        if not np.isfinite(Tmat).any():
            return np.inf, None, Tmat
        idx = np.unravel_index(np.argmin(Tmat), Tmat.shape)
        return float(Tmat[idx]), (int(idx[0]), int(idx[1])), Tmat

    def _clearance_info(self, view, cfg, threat):
        if threat is None or not threat.is_alive():
            return True, 0.0, None
        c = view.centroid
        uo = threat.vel / (np.linalg.norm(threat.vel) + _EPS)
        proj = (view.sheep_pos - c) @ uo if len(view.sheep_pos) \
            else np.array([0.0])
        ext = float(proj.max()) + threat.radius + cfg.r_sheep + 1.0
        rel = float((threat.pos - c) @ uo)
        return rel >= ext, 0.0, uo

    def _select_primary(self, view, cfg, threat, pair):
        """角色协调（4.9/4.10）：按"到预测碰撞点的距离 + 机动余量"评分，
        选出主要 DIVERT 机器人；不满足自身安全约束者剔除。"""
        M = cfg.n_robots
        k = self.n_primary if self.n_primary else max(2, M // 3)
        if threat is None or pair is None:
            return list(range(M))

        i_sheep, _ = pair
        p_star = None
        if i_sheep < len(view.sheep_pos):
            ttc_val = getattr(self, "_ttc_last", np.inf)
            if np.isfinite(ttc_val):
                p_star = collision_point(view.sheep_pos, view.sheep_vel,
                                         threat.pos, threat.vel,
                                         i_sheep, 0, ttc_val)
        if p_star is None:
            p_star = view.sheep_pos[i_sheep] if i_sheep < len(
                view.sheep_pos) else view.centroid

        dist_to_risk = np.linalg.norm(view.robot_pos - p_star, axis=1)
        # 自身安全：过于靠近障碍物的机器人不得担任主要 DIVERT 角色
        d_obs = np.linalg.norm(view.robot_pos - threat.pos, axis=1)
        # 硬不可行判据：机器人到障碍物**表面**的距离 < role_hard_margin 才判为不可行，
        # 并用较长持续窗口，避免机器人在障碍物附近反复进出角色集合。
        feasible = (d_obs - threat.radius) > self.role_hard_margin

        t = getattr(view, "t", 0.0)
        # 不可行需**持续** role_replace_delay 才替换（避免机器人在障碍物
        # 附近反复进出角色集合导致 role_switches 激增，5.13.9）
        infeasible = [j for j in self.primary_idx if not feasible[j]]
        if infeasible:
            if self._t_infeasible is None:
                self._t_infeasible = t
            elif (t - self._t_infeasible) < self.role_replace_delay:
                return self.primary_idx            # 暂不替换
        else:
            self._t_infeasible = None

        # 事件内冻结：当前主避让机器人仍可行时不再换人
        if self.primary_idx and all(feasible[j] for j in self.primary_idx):
            return self.primary_idx

        order = np.argsort(dist_to_risk)
        chosen = [int(j) for j in order if feasible[j]][:k]
        if not chosen:                      # 全部不可行 -> 退化为全体参与
            chosen = list(range(M))

        # 角色变化滞回：只有当新组合明显更优（平均距离改善 >= 1.5 m）时才换人，
        # 避免机器人频繁横穿羊群造成相互靠近与队形扰动（4.9/5.13.9）。
        if self.primary_idx and (t - self._t_role) >= self.role_dwell:
            if all(feasible[j] for j in self.primary_idx):
                gain = (float(np.mean(dist_to_risk[self.primary_idx])) -
                        float(np.mean(dist_to_risk[chosen])))
                if gain < 1.5:
                    chosen = list(self.primary_idx)

        if set(chosen) != set(self.primary_idx):
            self.role_switch_count += 1
            self._t_role = t
            changed = set(chosen) ^ set(self.primary_idx)
            for j in changed:
                self._trans[int(j)] = t          # 记录换组时刻（过渡限速/绕行）
        self.primary_idx = chosen
        return chosen

    # ------------------------------------------------------------------
    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        ttc_min, pair, Tmat = self._ttc(view, cfg)
        self._ttc_last = ttc_min

        # ---- 威胁对象选择（粘滞 + 抢占）：默认锁定当前威胁，避免多障碍
        #      场景下威胁对象频繁跳变造成状态机抖动；仅当新威胁明显更紧迫
        #      （TTC < 0.6×当前）或当前威胁已解除时才切换。----
        cur = getattr(self, "_threat_obj", None)
        cand = None
        if pair is not None and 0 <= pair[1] < view.obs_active_n:
            cand = view.obstacles_active[pair[1]]
        elif view.obs_active_n > 0:
            distances = [float(np.linalg.norm(c - o.pos))
                         for o in view.obstacles_active]
            cand = view.obstacles_active[int(np.argmin(distances))]

        ttc_cur = np.inf
        if cur is not None and cur.is_alive() and Tmat is not None \
                and cur in view.obstacles_active:
            k = view.obstacles_active.index(cur)
            col = Tmat[:, k]
            ttc_cur = float(np.min(col)) if np.isfinite(col).any() else np.inf
        if cur is None or not cur.is_alive() or np.isinf(ttc_cur) \
                or (cand is not None and ttc_min < 0.6 * ttc_cur):
            self._threat_obj = cand
        threat = getattr(self, "_threat_obj", None)

        # ---- 风险事件管理：事件切换时只重置"避让侧"；**不重置角色集合**，
        #      由 _select_primary 按集合比较自然更新（避免把事件切换
        #      误计为角色切换，压掉 role_switches 的虚高计数）。----
        if threat is not self._event_threat:
            self._event_threat = threat
            self.event_side = None
            self.divert_sign = None
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
