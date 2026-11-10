"""机器人运动学（unicycle, R1-R4）与对比方法控制器（论文 3/4/5.5 节）。

- B1 NormalHerder       : 原始沿路径牧羊（无避障）
- B2 DistanceThreshHerder: 距离阈值反应式避碰（阈值固定标定，不用 TTC）
- B3 CagingHerder       : 动态危险反应式牧羊（RSOS 风格，距离触发驱离）
- Ours                  : 见 proposed.py（TTC→PRE→DIVERT→RECOVER + 角色协调）
- PotentialHerder       : 早期势场式对照（可选，不进入默认对比）

队形：以羊群质心为锚、期望方向 d 为极轴；form='arc' 后驱弧（稳定推进）、
'ring' 平滑径向梯度（快速重组推力）、'cage' 全周等半径（停驻）。
"""
import numpy as np

from sheep import _heading


def _u(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def _norm1(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.zeros(2)


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# --------------------------------------------------------------------------
# 机器人车队（unicycle + R1-R4）与机器人自身安全（4.10/4.14）
# --------------------------------------------------------------------------
class RobotFleet:
    def __init__(self, pos, head=None, cfg=None):
        pos = np.asarray(pos, dtype=float)
        self.pos = pos.copy()
        M = pos.shape[0]
        if head is None:
            head = np.zeros(M)
        self.head = np.asarray(head, dtype=float)
        self.speed = np.zeros(M)

    def step(self, cmd, cfg, obs_pos=None):
        """执行指令；先做机器人自身安全过滤（机器人—机器人/障碍物），
        再按 unicycle + R1-R4 约束积分。"""
        cmd = apply_robot_safety(cmd, self.pos, obs_pos, cfg)
        M = len(self.pos)
        dt = cfg.dt
        for i in range(M):
            ci = cmd[i]
            spd_des = min(float(np.linalg.norm(ci)), cfg.v_max)
            if spd_des > 1e-3:
                th_des = np.arctan2(ci[1], ci[0])
                err = _wrap(th_des - self.head[i])
                turn = float(np.clip(err, -cfg.omega_max * dt,
                                     cfg.omega_max * dt))
                self.head[i] += turn
                align = float(np.cos(_wrap(th_des - self.head[i])))
                vd = spd_des * max(0.0, align)
            else:
                vd = 0.0
            dv = float(np.clip(vd - self.speed[i], -cfg.a_lin * dt,
                               cfg.a_lin * dt))
            self.speed[i] = float(np.clip(self.speed[i] + dv, 0.0, cfg.v_max))
            self.pos[i] += self.speed[i] * _u(self.head[i]) * dt
        return self.pos


def apply_robot_safety(cmd, robot_pos, obs_pos, cfg):
    """机器人自身安全优先级：机器人安全 > 羊群风险 > 路径 > 推进效率（4.10）。

    机器人—机器人采用**预测式投影**：以 pos + v·dt 预测下一时刻间距，
    迭代修正指令，使预测间距不小于 d_rr_safe（比单纯距离斥力更能
    真正保证最小间距，对应 5.13.9 的 dmin_robot_robot）。
    机器人—障碍物：距离 < d_ro_safe 时沿背离方向紧急避让。
    """
    cmd = cmd.copy()
    M = len(robot_pos)
    dt = cfg.dt
    for _ in range(4):
        p_next = robot_pos + cmd * dt
        for i in range(M):
            for j in range(i + 1, M):
                d = p_next[i] - p_next[j]
                dist = float(np.linalg.norm(d))
                if dist < cfg.d_rr_safe:
                    if dist < 1e-6:
                        d = np.array([1.0, 0.0])
                        dist = 1.0
                    corr = _norm1(d) * ((cfg.d_rr_safe - dist) /
                                        (2.0 * dt))
                    cmd[i] += corr
                    cmd[j] -= corr

    # 消除"接近速度分量"：若两机器人仍在相互接近，直接抵消其沿连线方向的
    # 闭合速度（比单纯斥力更有效，直接作用于实际最小间距 rr_min）
    for i in range(M):
        for j in range(i + 1, M):
            n = robot_pos[i] - robot_pos[j]
            dist = float(np.linalg.norm(n))
            if dist < 1e-6:
                continue
            n = n / dist
            closing = float((cmd[i] - cmd[j]) @ n)      # <0 表示在接近
            if dist < cfg.d_rr_safe and closing < 0:
                corr = 0.5 * closing * n
                cmd[i] -= corr
                cmd[j] += corr
    # 机器人—障碍物
    if obs_pos is not None and len(obs_pos):
        for i in range(M):
            d = robot_pos[i] - obs_pos
            dist = np.linalg.norm(d, axis=1)
            k = int(np.argmin(dist))
            if dist[k] < cfg.d_ro_safe:
                cmd[i] += _norm1(d[k]) * (cfg.d_ro_safe - dist[k]) * 2.0
    n = np.linalg.norm(cmd, axis=1)
    over = n > cfg.v_max
    if over.any():
        cmd[over] *= (cfg.v_max / n[over])[:, None]
    return cmd


# --------------------------------------------------------------------------
# 公共几何/队形
# --------------------------------------------------------------------------
def flock_geom(view):
    """羊群质心与队形半径（95% 分位）。view.sheep_pos 为控制器可用状态
    （局部观测 ∪ 邻近共享，见 simulation.View）。"""
    c = view.centroid
    d_to_c = np.linalg.norm(view.sheep_pos - c, axis=1)
    if len(d_to_c) == 0:
        return c, 2.0
    return c, max(1.6, float(np.percentile(d_to_c, 95.0)))


def task_direction(view, lookahead=6.0):
    """正常推进方向：质心指向路径上前方 lookahead 处。"""
    c = view.centroid
    s, _, _ = view.path.closest(c)
    s_a = float(np.clip(s + lookahead, 0.0, view.path.total))
    T = view.path.point_at(s_a)
    d = _norm1(T - c)
    if np.linalg.norm(T - c) < 1e-6:
        d = view.path.tangent_at(s)
    return d, s


SLOT0 = np.pi / 6.0


def slot_positions(c, R_fl, d, cfg, form="arc", f_arc=0.62, spread=0.95,
                   f0=1.05, k_rad=0.52, f_cage=0.72, n_robots=None):
    M = n_robots if n_robots is not None else cfg.n_robots
    perp = np.array([-d[1], d[0]])
    if form == "arc":
        phis = np.pi + np.linspace(-spread, spread, M)
        r = np.full(M, R_fl + f_arc * cfg.r_avo)
    elif form == "cage":
        phis = SLOT0 + 2 * np.pi * np.arange(M) / M
        r = np.full(M, R_fl + f_cage * cfg.r_avo)
    else:  # ring
        phis = SLOT0 + 2 * np.pi * np.arange(M) / M
        fac = np.clip(f0 + k_rad * np.cos(phis), 0.30, 1.70)
        r = R_fl + fac * cfg.r_avo
    slots = c + r[:, None] * (np.cos(phis)[:, None] * d[None, :]
                              + np.sin(phis)[:, None] * perp[None, :])
    # 槽位间距硬约束：保证同一编队内任意两机器人目标间距 >= d_rr_safe+slot_gap，
    # 否则机器人会因目标过近而相互挤压（S1 实测编队最小间距曾低于 d_rr_safe）。
    d_min = cfg.d_rr_safe + cfg.slot_gap
    for _ in range(6):
        moved = False
        for i in range(M):
            for j in range(i + 1, M):
                dv = slots[i] - slots[j]
                dist = float(np.linalg.norm(dv))
                if dist < d_min:
                    if dist < 1e-6:
                        dv = perp.copy()
                        dist = 1.0
                    corr = (dv / dist) * ((d_min - dist) / 2.0)
                    slots[i] += corr
                    slots[j] -= corr
                    moved = True
        if not moved:
            break
    return slots, phis


def slots_for(c, R_fl, d, cfg, idx, **kw):
    """为给定机器人索引子集生成槽位（角色协调用）。"""
    slots, _ = slot_positions(c, R_fl, d, cfg, n_robots=len(idx), **kw)
    out = np.zeros((cfg.n_robots, 2))
    out[np.asarray(idx, dtype=int)] = slots
    return out


def drive_command(view, cfg, c, R_fl, d, tag, form="arc", compact=1.0,
                  cmd_scale=1.0, **kw):
    slots, _ = slot_positions(c, R_fl, d, cfg, form=form, **kw)
    if compact != 1.0:
        slots = c[None, :] + (slots - c[None, :]) * compact
    return _cmd_from_slots(view, cfg, slots, cmd_scale), slots, tag


def drive_command_mixed(view, cfg, c, R_fl, d_normal, d_task, primary_idx,
                        tag, compact=1.0, cmd_scale=1.0, spread_task=1.25):
    """多机器人角色协调（4.9）：primary_idx 中的机器人执行 d_task（避碰），
    其余机器人继续 d_normal（推进/凝聚/路径维持）。
    spread_task 让主避让机器人分布更开，避免相互靠近（5.13.9）。"""
    M = cfg.n_robots
    primary_idx = np.asarray(primary_idx, dtype=int)
    other_idx = np.array([i for i in range(M) if i not in set(primary_idx)],
                         dtype=int)
    slots = np.zeros((M, 2))
    if len(other_idx):
        s_other, _ = slot_positions(c, R_fl, d_normal, cfg,
                                    n_robots=len(other_idx))
        slots[other_idx] = s_other
    if len(primary_idx):
        s_prim, _ = slot_positions(c, R_fl, d_task, cfg,
                                   n_robots=len(primary_idx),
                                   spread=spread_task)
        slots[primary_idx] = s_prim
    if compact != 1.0:
        slots = c[None, :] + (slots - c[None, :]) * compact
    # 混合队形的间距约束（折中方案）：
    #   组内（推动组内部 / 主避让组内部）：d_rr_safe + slot_gap（保持编队安全间距）
    #   组间（推进组 vs 主避让组）：0.8×d_rr_safe + slot_gap（放宽，避免因约束过强
    #     削弱避让时需要的横向压迫力——S2 实测过强约束会把碰撞率抬高）
    prim_set = set(int(i) for i in primary_idx)
    d_in = cfg.d_rr_safe + cfg.slot_gap
    d_cross = 0.8 * cfg.d_rr_safe + cfg.slot_gap
    for _ in range(8):
        moved = False
        for i in range(M):
            for j in range(i + 1, M):
                same = (i in prim_set) == (j in prim_set)
                d_min = d_in if same else d_cross
                dv = slots[i] - slots[j]
                dist = float(np.linalg.norm(dv))
                if dist < d_min:
                    if dist < 1e-6:
                        dv = np.array([-d_task[1], d_task[0]])
                        dist = 1.0
                    corr = (dv / dist) * ((d_min - dist) / 2.0)
                    slots[i] += corr
                    slots[j] -= corr
                    moved = True
        if not moved:
            break
    return _cmd_from_slots(view, cfg, slots, cmd_scale), slots, tag


def _cmd_from_slots(view, cfg, slots, cmd_scale=1.0):
    cmd = cfg.kp * (slots - view.robot_pos) * cmd_scale
    n = np.linalg.norm(cmd, axis=1)
    over = n > cfg.v_max
    if over.any():
        cmd[over] *= (cfg.v_max / n[over])[:, None]
    return cmd


def divert_target(view, cfg, c, R_fl, obs, shift=None, lookahead=4.0,
                  max_lateral_ratio=0.55, side=None):
    """主动避碰目标方向（4.4 节，B2 与 Ours 共用）：
    沿路径前进 + 有限横向偏移。

    side 给定时**锁定避让侧**（同一风险事件内 PRE 与 DIVERT 必须一致，
    且多障碍场景下不逐帧翻转）；side=None 时按"远离活跃障碍物横向重心"
    的规则重新计算。
    """
    s_c, lat_c, _ = view.path.closest(c)
    tang = view.path.tangent_at(s_c)
    nrm = np.array([-tang[1], tang[0]])

    if side is None:
        # 侧向选择：远离"活跃障碍物的横向重心"
        wsum = 0.0
        for o in view.obstacles_active:
            _, lat_o, _ = view.path.closest(o.pos)
            d = float(np.linalg.norm(c - o.pos))
            w = 1.0 / (d + 1e-6)
            if abs(lat_o) > 0.2:
                wsum += w * np.sign(lat_o)
            else:
                # 障碍物几乎在路径中心线上：按其来向的横向分量判断
                wsum += w * (1.0 if o.vel[1] >= 0 else -1.0)
        side = -1.0 if wsum > 0 else 1.0
        if abs(wsum) < 1e-9:
            _, lat_obs, _ = view.path.closest(obs.pos)
            side = -float(np.sign(lat_obs)) if abs(lat_obs) > 0.2 else 1.0

    halfwidth = view.path.halfwidth_at(s_c)
    max_shift = max(1.0, max_lateral_ratio * halfwidth)
    shift = min(shift if shift is not None else 2.4, max_shift)
    dist = float(np.linalg.norm(c - obs.pos))
    if dist < 3.5:
        shift = min(max_shift, max(shift, 3.0))
    s_a = float(np.clip(s_c + lookahead, 0.0, view.path.total))
    p_a = view.path.point_at(s_a)
    tang_a = view.path.tangent_at(s_a)
    norm_a = np.array([-tang_a[1], tang_a[0]])
    target = p_a + side * shift * norm_a
    d = _norm1(target - c)
    if np.linalg.norm(d) < 1e-6:
        d = side * nrm
    return d, side, shift


def threat_obstacle(view, c):
    """当前最近障碍物（距离口径，供 B2/B3 使用）。"""
    if view.obs_active_n == 0:
        return None
    d = [float(np.linalg.norm(c - o.pos)) for o in view.obstacles_active]
    return view.obstacles_active[int(np.argmin(d))]


def min_sheep_obs_dist(view):
    """可用状态下羊—障碍物最小几何距离（B2 距离判据）。"""
    if view.obs_active_n == 0 or len(view.sheep_pos) == 0:
        return np.inf
    best = np.inf
    for o in view.obstacles_active:
        dd = np.linalg.norm(view.sheep_pos - o.pos, axis=1).min()
        best = min(best, float(dd))
    return best


# --------------------------------------------------------------------------
# B1：原始沿路径牧羊
# --------------------------------------------------------------------------
class NormalHerder:
    name = "B1"

    def __init__(self, lookahead=6.0):
        self.lookahead = lookahead

    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        d, _ = task_direction(view, self.lookahead)
        cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "normal")
        return cmd, "normal"


# --------------------------------------------------------------------------
# B2：距离阈值反应式避碰（阈值固定标定，不使用 TTC）
# --------------------------------------------------------------------------
class DistanceThreshHerder(NormalHerder):
    name = "B2"

    def __init__(self, lookahead=6.0, d_trigger=None, hyst=None,
                 max_lateral_ratio=0.55):
        super().__init__(lookahead)
        self.d_trigger = d_trigger
        self.hyst = hyst
        # 横向上限：默认 0.55；公平性单因素检验时可与 Ours 统一为同一值
        self.max_lateral_ratio = float(max_lateral_ratio)
        self.state = "normal"
        self.switch_count = 0

    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        d_trigger = cfg.b2_d_trigger if self.d_trigger is None \
            else self.d_trigger
        hyst = cfg.b2_hyst if self.hyst is None else self.hyst
        dist = min_sheep_obs_dist(view)

        if self.state == "normal":
            if dist <= d_trigger:
                self.state = "divert"
                self.switch_count += 1
        else:
            if dist >= d_trigger + hyst:
                self.state = "normal"
                self.switch_count += 1

        o = threat_obstacle(view, c)
        if self.state == "divert" and o is not None:
            d, _, _ = divert_target(view, cfg, c, R_fl, o,
                                    max_lateral_ratio=self.max_lateral_ratio)
            cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "divert")
            return cmd, "divert"
        d, _ = task_direction(view, self.lookahead)
        cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "normal")
        return cmd, "normal"


# --------------------------------------------------------------------------
# B3：动态危险反应式牧羊（危险检测 + 局部机器人驱离，无 TTC/PRE）
# --------------------------------------------------------------------------
class CagingHerder(NormalHerder):
    name = "B3"

    def __init__(self, lookahead=6.0, r_detect=15.0, cage=0.8):
        super().__init__(lookahead)
        self.r_detect = r_detect
        self.cage = cage
        self.phase = "cage"
        self.repel_obs = None
        self.repel_dir = None
        self.switch_count = 0

    def _passed(self, view, o, R_fl, r_sheep):
        uo = o.vel / (np.linalg.norm(o.vel) + 1e-9)
        rel = float((o.pos - view.centroid) @ uo)
        return rel > R_fl + o.radius + r_sheep + 2.0

    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        if self.phase == "cage":
            danger = None
            for o in view.obstacles_active:
                if np.linalg.norm(c - o.pos) < self.r_detect:
                    danger = o
                    break
            if danger is not None:
                self.phase = "repel"
                self.switch_count += 1
                self.repel_obs = danger
                uo = danger.vel / (np.linalg.norm(danger.vel) + 1e-9)
                nrm = np.array([-uo[1], uo[0]])
                lat = float((c - danger.pos) @ nrm)
                if abs(lat) < 2.0:
                    s, _, _ = view.path.closest(c)
                    self.repel_dir = -view.path.tangent_at(s)
                else:
                    self.repel_dir = np.sign(lat) * nrm
            d, _ = task_direction(view, self.lookahead)
            cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "cage",
                                      compact=self.cage)
            return cmd, "cage"

        o = self.repel_obs
        if o is None or not o.is_alive() or \
                self._passed(view, o, R_fl, cfg.r_sheep):
            self.phase = "cage"
            self.switch_count += 1
            self.repel_obs = None
            d, _ = task_direction(view, self.lookahead)
            cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "cage",
                                      compact=self.cage)
            return cmd, "cage"
        cmd, _, _ = drive_command(view, cfg, c, R_fl, self.repel_dir,
                                  "repel", compact=self.cage)
        return cmd, "repel"


# --------------------------------------------------------------------------
# 可选：早期势场式对照（不进入默认对比）
# --------------------------------------------------------------------------
class PotentialHerder(NormalHerder):
    name = "B2_pot"

    def __init__(self, lookahead=6.0, r_pot=8.0, k_pot=3.0):
        super().__init__(lookahead)
        self.r_pot = r_pot
        self.k_pot = k_pot

    def step(self, view, cfg):
        c, R_fl = flock_geom(view)
        d, _ = task_direction(view, self.lookahead)
        cmd, _, _ = drive_command(view, cfg, c, R_fl, d, "normal")
        for o in view.obstacles_active:
            dd = view.robot_pos - o.pos
            dist = np.linalg.norm(dd, axis=1)
            inside = dist < self.r_pot
            if inside.any():
                w = self.k_pot * (1.0 - dist[inside] / self.r_pot)
                cmd[inside] += (dd[inside] /
                                np.maximum(dist[inside], 1e-6)[:, None]
                                ) * w[:, None]
        n = np.linalg.norm(cmd, axis=1)
        over = n > cfg.v_max
        if over.any():
            cmd[over] *= (cfg.v_max / n[over])[:, None]
        return cmd, "normal"
