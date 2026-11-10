"""外部基线的忠实复现（协议 §5/§6/§7）。

本文件实现 **VH-Path**（Van Havermaet, Khaluf, Simoens, *Sci. Rep.* 14:14915, 2024），
严格按其官方代码 `github.com/stefvh/path_shepherding` 逐分支移植：

- 控制律  `algorithms/path_steering.py::compute_path_steering_action`（P0.0/P0.1/P1.0/CAGING）
- 避碰    `algorithms/base.py::handle_collision_avoidance_{hard,soft,and_caging}`
- 势场    `algorithms/potentials.py::get_follow_path_potential_gradient`
- 梯度    `paths/path.py::get_gradient`（真实边界内取切向；安全边界外取"指向最近中心线点"；
          机器人在安全边界内 → 改用最近羊的梯度）＋ `(1 - safety_margin_multiplier)` 宽度，
          `safety_margin_multiplier = 0.4` → 安全走廊 = 0.6 × 真实走廊
- 参数    `local-run.py`：`max_linear_velocity_herd = 2`、`max_linear_velocity_robot = 4`、
          `zone_width_aversion = 40`（= zI = radius_robot_herd_interaction）、
          `range_detection_robot = 100 = 2.5 zI`、`rate_circular_motion = 0.2`、
          `caging_velocity = 1.0`、`avoidance_boundary_{herd,robot} = 1.0`
- 未移植（本文信息边界所限，已在 BASELINES_SPEC.md 记录）：
  `compute_search_action` 的多机搜索消息协商（本实现退化为"沿路径梯度以最大速度前进"）；
  原文机器人为质点（`get_next_pose_by_direction`，无机器人间硬安全约束），本文统一执行层
  负责 R1–R4 与物理安全（协议 §5.3）。

## 量纲换算（唯一规则）

官方中"距离"与"速度"同单位（cm 与 cm/step，`simulation_frequency = 1` → 1 step = 1 s）。
本文按其**无量纲比值**换算：

    距离：S = zI_m / zI_o = cfg.r_avo / 40.0        [m per official unit]
    速度：U = v_herd_m / v_herd_o = cfg.v_base / 2.0 [m/s per official unit]

- 交互区半径      zI          = cfg.r_avo = 4.0 m          （官方 40）
- 转向半径        bound_steer = zI − 2u  = 3.8 m           （官方 38）
- 保距半径        bound_not_steer = 1.15 zI + 2u = 4.8 m   （官方 48）
- P0.0 触发半径   zI + 2u     = 4.2 m                      （官方 42）
- 误差裕度（ℓ）   2u          = 0.2 m                      （官方 2）
- 邻近误差裕度    err_marg    = 2 × caging_velocity = 0.2 m（官方 2）
- 硬避碰边界      1u          = 0.1 m                      （官方 1）
- 围笼速度        caging_velocity = 1u → 0.5 m/s           （官方 1）
- 圆周运动率      κ           = 0.2 / S = 2.0 m⁻¹          （官方 0.2 per cm）
- 探测半距        det         = 2.5 zI = 10.0 m            （官方 100）

信息边界（协议 §6）：仅使用羊与机器人的位置/速度与已知路径；不含 TTC/PRE/DIVERT/RECOVER。
"""
import numpy as np

Z_OFF = 40.0          # 官方 zI（zone_width_aversion）
VH_OFF = 2.0          # 官方 max_linear_velocity_herd
VR_OFF = 4.0          # 官方 max_linear_velocity_robot


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _unit(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def _avoid_hard(d2, focal, others, boundary):
    """algorithms/base.py::handle_collision_avoidance_hard（官方单位）。"""
    idx = (d2 > 0.0) & (d2 <= boundary ** 2)
    if not np.any(idx):
        return False, 0.0, 0.0
    rel = others[idx, :2] - focal[:2]
    nrm = np.linalg.norm(rel, axis=1, keepdims=True)
    q = -np.sum(rel / np.maximum(nrm, 1e-12), axis=0)
    return True, np.arctan2(q[1], q[0]) % (2 * np.pi), float(np.linalg.norm(q))


def _avoid_caging(d2, focal, others, nearest, boundary, v_max, v_herd,
                  target_distance):
    """base.py::handle_collision_avoidance_and_caging（官方单位）。"""
    idx = (d2 > 0.0) & (d2 <= boundary ** 2)
    if not np.any(idx):
        return False, 0.0, 0.0
    rel = others[idx, :2] - focal[:2]
    nrm = np.linalg.norm(rel, axis=1, keepdims=True)
    d = -np.sum(rel / np.maximum(nrm, 1e-12), axis=0)
    theta = np.arctan2(d[1], d[0]) % (2 * np.pi)
    c = np.cos(nearest[2] - theta)
    eta = -v_herd * c + np.sqrt(max(v_herd ** 2 * (c ** 2 - 1) + v_max ** 2, 0.0))
    # 与"以最近羊为圆心、半径 target_distance 的圆"求交，限制单步前进量
    b_int, pos_int = _line_circle(focal[:2], focal[:2] + v_max * _unit(theta),
                                  nearest[:2], target_distance)
    if b_int:
        eta = min(eta, float(np.linalg.norm(focal[:2] - pos_int)))
    q = v_herd * _unit(nearest[2]) + eta * _unit(theta)
    return True, np.arctan2(q[1], q[0]) % (2 * np.pi), float(np.linalg.norm(q))


def _avoid_soft(focal, nearest, v_max, v_herd):
    """base.py::handle_collision_avoidance_soft（官方单位）。

    注意方向：theta 指向"背离最近羊"的方向 —— 该分支在机器人比目标半径更近时触发，
    因此是"后退让位"而非"冲向羊"。
    """
    d = -(nearest[:2] - focal[:2])
    n = float(np.linalg.norm(d))
    theta = np.arctan2(d[1], d[0]) % (2 * np.pi) if n > 1e-12 else 0.0
    c = np.cos(nearest[2] - theta)
    eta = -v_herd * c + np.sqrt(max(v_herd ** 2 * (c ** 2 - 1) + v_max ** 2, 0.0))
    q = v_herd * _unit(nearest[2]) + eta * _unit(theta)
    return np.arctan2(q[1], q[0]) % (2 * np.pi), float(np.linalg.norm(q))


def _line_circle(p0, p1, c, r):
    """utils/maths.py::line_circle_intersection（取沿 p0→p1 的首个交点）。"""
    d = p1 - p0
    f = p0 - c
    a = float(d @ d)
    if a < 1e-18:
        return 0, np.zeros(2)
    b = 2.0 * float(f @ d)
    cc = float(f @ f) - r * r
    disc = b * b - 4 * a * cc
    if disc < 0:
        return 0, np.zeros(2)
    t = (-b - np.sqrt(disc)) / (2 * a)
    if t < 0:
        t = (-b + np.sqrt(disc)) / (2 * a)
    if t < 0 or t > 1.0:
        return 0, p0 + t * d
    return 1, p0 + t * d


class VHPathHerder:
    """Van Havermaet et al. (2024) 沿动态路径反应式牧羊（忠实移植）。"""

    name = "VH-Path"

    def __init__(self, kappa=None, caging_velocity=1.0, safe_ratio=0.6,
                 avoidance_boundary_herd=1.0, avoidance_boundary_robot=1.0,
                 det_range=None, rotate_herd_velocity=None):
        self.kappa_off = 0.2 if kappa is None else float(kappa)
        self.caging_velocity_off = float(caging_velocity)
        self.safe_ratio = float(safe_ratio)
        self.avoid_h_off = float(avoidance_boundary_herd)
        self.avoid_r_off = float(avoidance_boundary_robot)
        self.det_range = det_range            # None -> 2.5 * cfg.r_avo
        self.v_herd = rotate_herd_velocity    # None -> cfg.v_base
        self._circ = None                     # 官方 current_circular_direction（跨步保持）

    # ------------------------------------------------------------------
    def _grad_one(self, view, p):
        """paths/path.py::get_gradient(x, y)。

        返回 (gradient, is_in_safety_margin)：
          - 真实边界内且安全边界内 → 梯度 = 路径切向，且在安全走廊内（True）；
          - 真实边界内、安全边界外 → 指向最近中心线点（False）；
          - 真实边界外 → 指向最近中心线点（False）。
        """
        path = view.path
        s, lat, _ = path.closest(p)
        tang = path.tangent_at(s)
        half_real = path.halfwidth_at(s)
        inside_real = abs(lat) <= half_real
        inside_safe = abs(lat) <= self.safe_ratio * half_real
        if inside_real and inside_safe:
            return tang, True
        n0 = path.point_at(s)
        d = n0 - p
        nd = float(np.linalg.norm(d))
        return (d / nd if nd > 1e-9 else tang), False

    def _path_gradient(self, view, p_robot, p_herd_nearest):
        """potentials.py::get_follow_path_potential_gradient。
        机器人梯度与最近羊梯度各 0.5 加权；机器人在安全边界内时改用羊群梯度。"""
        g_robot, in_safe = self._grad_one(view, p_robot)
        g_herd, _ = self._grad_one(view, p_herd_nearest)
        if in_safe:
            return g_herd
        g = 0.5 * g_robot + 0.5 * g_herd
        n = float(np.linalg.norm(g))
        return (g / n) if n > 1e-9 else g_herd

    # ------------------------------------------------------------------
    def step(self, view, cfg):
        M = cfg.n_robots
        cmd = np.zeros((M, 2))
        if self._circ is None or len(self._circ) != M:
            self._circ = np.ones(M)

        # 量纲换算（见模块 docstring）
        S = cfg.r_avo / Z_OFF                      # m per official distance unit
        U = (self.v_herd if self.v_herd is not None else cfg.v_base) / VH_OFF
        zI = cfg.r_avo
        ell = 2.0 * S                              # 单步羊位移裕度 ℓ
        bound_steer = zI - 2.0 * S
        bound_not_steer = 1.15 * zI + 2.0 * S
        target_default = zI + 2.0 * S
        err_marg = 2.0 * self.caging_velocity_off * S
        kappa = self.kappa_off / S
        avoid_h = self.avoid_h_off * S
        avoid_r = self.avoid_r_off * S
        det = self.det_range if self.det_range is not None else 2.5 * zI
        V_max_o = cfg.v_max / U                    # 官方单位下的机器人最大速度

        sheep = view.sheep_pos
        n_s = len(sheep)
        if n_s == 0:                               # 无可用羊群信息 -> 沿路径搜索
            return self._search(view, cfg, M), "search"
        svel = view.sheep_vel if len(view.sheep_vel) == n_s else np.zeros_like(sheep)
        spd = np.linalg.norm(svel, axis=1)
        head_s = np.where(spd > 1e-6,
                          np.arctan2(svel[:, 1], svel[:, 0]),
                          np.arctan2(sheep[:, 1] - sheep[:, 1].mean(),
                                     sheep[:, 0] - sheep[:, 0].mean()))
        v_herd = float(spd.mean()) if len(spd) else cfg.v_base
        v_herd_o = max(v_herd / U, 1e-6)           # avg_linear_velocity_herd（官方单位）
        if v_herd < 1e-6:
            v_herd_o = 0.0
        robots = view.robot_pos
        b_steer_available = view.path is not None

        for i in range(M):
            p = robots[i]
            d2_all = np.sum((sheep - p) ** 2, axis=1)
            near = d2_all <= det ** 2
            if not np.any(near):
                cmd[i] = self._search_one(view, cfg, p)
                continue
            # --- 局部羊群（原文 herd_poses 近邻筛选） ---
            sh = sheep[near]
            d2 = d2_all[near]
            k = int(np.argmin(d2))
            nearest = np.array([sh[k, 0], sh[k, 1], head_s[near][k]])
            d_i = float(np.sqrt(d2[k]))

            # --- 近邻机器人（0 < d <= det） ---
            r2_all = np.sum((robots - p) ** 2, axis=1)
            rp = (r2_all > 0.0) & (r2_all <= det ** 2)
            others = robots[rp]
            r2 = r2_all[rp]

            # --- (1) STEERING：任务方向 = 路径梯度 ---
            grad = self._path_gradient(view, p, nearest[:2])
            d_theta = np.arctan2(grad[1], grad[0]) % (2 * np.pi)
            b_steering = b_steer_available

            # --- P0.0 与其它机器人避碰 ---
            if d_i >= target_default:
                b_coll, th, v = _avoid_caging(r2, p, others, nearest, avoid_r,
                                              V_max_o, v_herd_o, target_default)
            else:
                b_coll, th, v = _avoid_hard(r2, p, others, avoid_r)
            if b_coll:
                cmd[i] = (v * U) * _unit(th)
                continue

            # --- P0.1 与羊群避碰 ---
            b_coll, th, v = _avoid_hard(d2, p, sh, avoid_h)
            if b_coll:
                cmd[i] = (v * U) * _unit(th)
                continue

            # --- 转向状态判定（原文 (1) STEERING 段） ---
            heading = np.arctan2(p[1] - nearest[1], p[0] - nearest[0])
            sign_heading = 1.0 if heading >= 0 else -1.0
            target = bound_not_steer
            b_active = False
            if b_steering:
                close = d2 <= (zI + 2.0 * S) ** 2
                if np.any(close):
                    gammas = np.arctan2(p[1] - sh[close, 1], p[0] - sh[close, 0])
                    b_active = bool(np.all(np.abs(_wrap(d_theta - gammas)) > np.pi / 2))
                    if b_active:
                        target = bound_steer
                else:
                    if abs(_wrap(d_theta - heading)) > np.pi / 2:
                        target = bound_steer
                        b_active = True

            # --- P1.0 距目标半径过近 -> 退让（原文 soft avoidance） ---
            if target - d_i >= ell:
                th, v = _avoid_soft(p, nearest, V_max_o, v_herd_o)
                cmd[i] = (v * U) * _unit(th)
                continue

            # --- (2) CAGING ---
            circ = self._circ[i]
            if len(others):
                rel = others[:, :2] - nearest[:2]
                ang = heading - np.arctan2(rel[:, 1], rel[:, 0])
                above = (0.0 < sign_heading * ang) & (sign_heading * ang <= np.pi)
                d_above = d_under = 2.0 * zI - err_marg
                if np.any(above):
                    d_above = float(np.sqrt(np.min(r2[above])))
                if np.any(~above):
                    d_under = float(np.sqrt(np.min(r2[~above])))
                if abs(d_above - d_under) <= err_marg / 2.0:
                    if 0.0 <= d_i - target <= ell:
                        # 两侧均衡且已在目标半径 -> 复制羊群速度与方向
                        cmd[i] = (v_herd_o * U) * _unit(nearest[2])
                        continue
                else:
                    circ = sign_heading if d_above < d_under else -sign_heading
            caging_angle = heading + circ * (
                np.pi / 2.0 + np.arctan(kappa * (d_i - target)))
            caging_v_o = self.caging_velocity_off
            if b_active and (d_i - target) > ell:
                # 原文："move to steering-radius as fast as possible"
                c = np.cos(nearest[2] - caging_angle)
                caging_v_o = -v_herd_o * c + np.sqrt(max(
                    v_herd_o ** 2 * (c ** 2 - 1) + V_max_o ** 2, 0.0))
            q = v_herd_o * _unit(nearest[2]) + caging_v_o * _unit(caging_angle)
            n = float(np.linalg.norm(q))
            cmd[i] = (n * U) * _unit(np.arctan2(q[1], q[0])) if n > 1e-12 \
                else (v_herd_o * U) * _unit(nearest[2])
            self._circ[i] = circ
        return cmd, "caging"

    # ------------------------------------------------------------------
    def _search_one(self, view, cfg, p):
        """搜索分支的简化：沿路径梯度以最大速度前进（原文 compute_search_action
        的多机搜索消息协商未移植，见模块 docstring）。"""
        if view.path is not None:
            g, _ = self._grad_one(view, p)
            n = float(np.linalg.norm(g))
            if n > 1e-9:
                return cfg.v_max * (g / n)
        return np.zeros(2)

    def _search(self, view, cfg, M):
        return np.array([self._search_one(view, cfg, view.robot_pos[i])
                         for i in range(M)])


# ======================================================================
# VH-Danger：Van Havermaet et al. 的"危险区"反应式牧羊（RSOS 2023）
# 官方代码 github.com/stefvh/shepherding :: experiments/steering_2d.py
# ======================================================================
class VHDangerHerder:
    """危险区（danger zone）反应式牧羊（忠实移植 `steering_2d_model`）。

    原文参数（`utils/settings.py` 无调参项 → 取函数默认值）：
        mu = 19（羊群回避区宽，等同本文 cfg.r_avo；原文 school 模型以 zox = mu 传入）
        v_a_max = 6, v_h_max = 2, v_h_avg = 2, eta = 0.5, tau = 0.1
        delta = 2 mu = 38, zoi_a = 1.5 delta = 57（感知半径）
        e_star = 2 v_h_max + eta = 4.5, r_star = mu + e_star = 23.5,
        R_star = r_star + mu/2 = 33, e_steer = v_h_max = 2,
        eps_a = v_h_max = 2, eps_h = 3 v_h_max = 6, zor_h = zor_a = 1

    与 VH-Path 的差异（均为原文差异，不是移植简化）：
      - 无"绕最近个体围笼"的固定 caging 半径，改为两个可控边界
        d_bound ∈ {mu − e_steer, R_star}；
      - 羊群前馈速度取固定 v_h_max（而非实测平均速度）；
      - 有独立的"不得进入羊群危险区"分支（d_star − d_bound ≤ −e_star）。
    同 VH-Path：未移植 searching_model 的多机搜索协商（退化为沿路径梯度前进）；
    机器人执行（转向速率、加减速、物理安全）由本文统一执行层负责。

    ## 任务方向 d_theta（原文有三种 scenario，本实现提供三种 task_mode）
      原文 `steering_2d_model` 的 d_theta 只用于**决定机器人是否为"推压方"**
      （判据：机器人是否位于"任务方向上羊群的后方"），围笼运动本身恒为绕羊群切向。
      - `"path"`：d_theta = 路径梯度（原文 scenario == "path"）；
      - `"danger"`：d_theta = 探测到的**危险块（danger patch）**方向：
        `patch_potential_gradient` 给出 q = −Σ(危险块 − 机器人)（**背离危险**的质心方向），
        机器人在 zoi_a 内**一跳共享**各自的危险梯度（原文 steer_msg 机制），
        取其和的方向；无任何危险信息时 b_steering = False（不推压、仅保距 R_star）；
      - `"mixed"`（本文主实验默认）：探测到危险时用 `"danger"`，否则用 `"path"`。
        **这是本文的显式组合（原文一次只跑一个 scenario），已在 BASELINES_SPEC.md 记录**：
        理由是本文任务是"沿路径牧羊 + 动态避障"，若只给 danger 模式则基线不执行任务、
        若只给 path 模式则基线的危险响应机制完全不生效，两种都不可比。

    量纲换算（同 VH-Path 规则）：S = cfg.r_avo / mu（距离），
    U = cfg.v_base / v_h_max（速度）。**本类内部一律在官方单位下计算**：
    位置乘 invS = 1/S，速度除以 U，最后把 |q| 乘回 U。
    """

    name = "VH-Danger"

    def __init__(self, mu_off=19.0, v_a_max_off=6.0, v_h_max_off=2.0,
                 eta_off=0.5, tau_off=0.1, task_mode="mixed", det_range=None,
                 rotate_herd_velocity=None):
        self.mu_off = float(mu_off)
        self.v_a_max_off = float(v_a_max_off)
        self.v_h_max_off = float(v_h_max_off)
        self.eta_off = float(eta_off)
        self.tau_off = float(tau_off)
        assert task_mode in ("path", "danger", "mixed")
        self.task_mode = task_mode
        self.det_range = det_range
        self.v_herd = rotate_herd_velocity
        self._var = None                      # 官方跨步保持的 var_theta

    # ------------------------------------------------------------------
    def _grad_one(self, view, p):
        """"path" 场景的任务方向：走廊内取路径切向，走廊外指向最近中心线点。"""
        path = view.path
        s, lat, _ = path.closest(p)
        tang = path.tangent_at(s)
        if abs(lat) <= path.halfwidth_at(s):
            return tang
        d = path.point_at(s) - p
        nd = float(np.linalg.norm(d))
        return (d / nd) if nd > 1e-9 else tang

    def _search_one(self, view, cfg, p):
        if view.path is not None:
            g = self._grad_one(view, p)
            n = float(np.linalg.norm(g))
            if n > 1e-9:
                return cfg.v_max * (g / n)
        return np.zeros(2)

    # ------------------------------------------------------------------
    def step(self, view, cfg):
        M = cfg.n_robots
        cmd = np.zeros((M, 2))
        if self._var is None or len(self._var) != M:
            self._var = np.ones(M)

        invS = self.mu_off / cfg.r_avo            # 1/S：本文单位 → 官方单位
        U = (self.v_herd if self.v_herd is not None else cfg.v_base) \
            / self.v_h_max_off                    # m/s per official speed unit

        # ---- 原文派生量（全部为官方单位） ----
        v_h = self.v_h_max_off                    # v_h_hat（固定，取 v_h_max）
        V_a = cfg.v_max / U                       # v_a_max
        e_star = 2.0 * v_h + self.eta_off
        r_star = self.mu_off + e_star
        R_star = r_star + self.mu_off / 2.0
        e_steer = v_h
        eps_a = v_h
        eps_h = 3.0 * v_h
        zoi_a = 3.0 * self.mu_off                 # 1.5 * (2 mu)
        d_bound_lo = self.mu_off - e_steer
        close_r = self.mu_off + v_h
        det_o = (self.det_range * invS) if self.det_range is not None else zoi_a

        sheep = view.sheep_pos
        n_s = len(sheep)
        if n_s == 0:
            return np.array([self._search_one(view, cfg, view.robot_pos[i])
                             for i in range(M)]), "search"
        svel = view.sheep_vel if len(view.sheep_vel) == n_s else np.zeros_like(sheep)
        spd = np.linalg.norm(svel, axis=1)
        head_s = np.where(spd > 1e-6, np.arctan2(svel[:, 1], svel[:, 0]),
                          np.arctan2(sheep[:, 1] - sheep[:, 1].mean(),
                                     sheep[:, 0] - sheep[:, 0].mean()))
        sheep_o = sheep * invS
        robots_o = view.robot_pos * invS
        b_path = view.path is not None

        # ---- 危险块（danger patch）探测与一跳共享：原文 patch_potential_gradient ----
        # 每台机器人各自的"背离危险"梯度；再在 zoi_a 内一跳共享（原文 steer_msg 机制）。
        zoi_d = 1.25 * zoi_a
        g_danger = np.full((M, 2), np.nan)
        obs = view.obs_pos if view.obs_active_n else np.zeros((0, 2))
        if len(obs):
            obs_o = obs * invS
            for k in range(M):
                dd = np.sum((obs_o - robots_o[k]) ** 2, axis=1)
                sel = dd <= zoi_d ** 2
                if np.any(sel):
                    q = -np.sum(obs_o[sel] - robots_o[k], axis=0)
                    nn = float(np.linalg.norm(q))
                    if nn > 1e-9:
                        g_danger[k] = q / nn

        for i in range(M):
            p = robots_o[i]
            d2_all = np.sum((sheep_o - p) ** 2, axis=1)
            hit = d2_all <= det_o ** 2
            if not np.any(hit):
                cmd[i] = self._search_one(view, cfg, view.robot_pos[i])
                continue
            d2 = d2_all[hit]
            sh = sheep_o[hit]
            k = int(np.argmin(d2))
            d_star = float(np.sqrt(d2[k]))
            theta_star = float(head_s[hit][k])
            h_star = sh[k]

            a2_all = np.sum((robots_o - p) ** 2, axis=1)
            ap = (a2_all > 0.0) & (a2_all <= det_o ** 2)

            # ---- 任务方向 d_theta（原文三种 scenario，本实现由 task_mode 选择） ----
            nb = ap & (a2_all <= zoi_a ** 2)           # 一跳通信邻居（含自身）
            gsum = g_danger[nb]
            gsum = gsum[np.isfinite(gsum[:, 0])]
            b_danger = len(gsum) > 0
            if b_danger and self.task_mode in ("danger", "mixed"):
                s = gsum.sum(axis=0)
                d_theta = np.arctan2(s[1], s[0]) % (2 * np.pi)
                b_steering = True
            elif self.task_mode in ("path", "mixed") and b_path:
                g = self._grad_one(view, view.robot_pos[i])
                d_theta = np.arctan2(g[1], g[0]) % (2 * np.pi)
                b_steering = True
            else:
                # danger 模式且无任何危险信息 -> 不推压（b_steering = False）
                d_theta = 0.0
                b_steering = False

            # ---- (0) 与羊群硬避碰（zor_h = 1） ----
            col = d2 <= 1.0
            if np.any(col):
                th, v = _away_dir(p, sh[col])
                cmd[i] = (v * U) * _unit(th)
                continue

            # ---- (1) 转向状态与边界 d_bound ----
            gamma_o = np.arctan2(p[1] - h_star[1], p[0] - h_star[0])
            beta = 1.0 if gamma_o >= 0 else -1.0
            d_bound = R_star
            b_active = False
            if b_steering:
                close = d2 <= close_r ** 2
                if np.any(close):
                    gs = np.arctan2(p[1] - sh[close, 1], p[0] - sh[close, 0])
                    b_active = bool(np.all(np.abs(_wrap(d_theta - gs)) > np.pi / 2))
                    if b_active:
                        d_bound = d_bound_lo
                else:
                    if abs(_wrap(d_theta - gamma_o)) > np.pi / 2:
                        d_bound = d_bound_lo
                        b_active = True

            # ---- (0) 不得进入羊群危险区：后退让位 ----
            if d_star - d_bound <= -e_star:
                th, v = _back_off(p, h_star, theta_star, r_star, V_a, v_h,
                                  clamp=True)
                cmd[i] = (v * U) * _unit(th)
                continue

            # ---- (0) 与其它机器人避碰（仅当 d_star ≥ d_bound） ----
            if d_star >= d_bound:
                col_a = (a2_all > 0.0) & (a2_all <= 1.0)      # zor_a = 1
                if np.any(col_a):
                    th_a, _ = _away_dir(p, robots_o[col_a])
                    th, v = _back_off(p, h_star, theta_star, r_star, V_a, v_h,
                                      clamp=True, direction=th_a)
                    cmd[i] = (v * U) * _unit(th)
                    continue

            # ---- (2) CAGING ----
            var = self._var[i]
            if np.any(ap):
                oth = robots_o[ap]
                r2 = a2_all[ap]
                rel = oth - h_star
                gam = gamma_o - np.arctan2(rel[:, 1], rel[:, 0])
                bar = (0.0 < beta * gam) & (beta * gam <= np.pi)
                under = ~bar
                d_bar = d_under = 2.0 * self.mu_off - eps_a
                if np.any(bar):
                    d_bar = float(np.sqrt(np.min(r2[bar])))
                if np.any(under):
                    d_under = float(np.sqrt(np.min(r2[under])))
                if abs(d_bar - d_under) <= eps_a / 2.0:
                    if 0.0 <= d_star - d_bound <= eps_h:
                        # 复制羊群朝向与速度（原文 d_v = v_h_hat）
                        cmd[i] = (v_h * U) * _unit(theta_star)
                        continue
                else:
                    var = beta if d_bar < d_under else -beta
                    self._var[i] = var
            theta = gamma_o + var * (np.pi / 2.0
                                     + np.arctan(self.tau_off * (d_star - d_bound)))
            eta_o = self.eta_off                       # 原文默认切向速度 0.5
            if b_active and (d_star - d_bound) > eps_h:
                c = np.cos(theta_star - theta)
                eta_o = -v_h * c + np.sqrt(max(v_h ** 2 * (c ** 2 - 1)
                                               + V_a ** 2, 0.0))
            q = v_h * _unit(theta_star) + eta_o * _unit(theta)
            n = float(np.linalg.norm(q))
            cmd[i] = (n * U) * _unit(np.arctan2(q[1], q[0])) if n > 1e-12 \
                else (v_h * U) * _unit(theta_star)
        return cmd, "caging"


def _away_dir(p, others_o):
    """hard avoidance 的等价形式（官方单位，速度 = |q|）。"""
    rel = others_o[:, :2] - p[:2]
    nrm = np.linalg.norm(rel, axis=1, keepdims=True)
    q = -np.sum(rel / np.maximum(nrm, 1e-12), axis=0)
    return np.arctan2(q[1], q[0]) % (2 * np.pi), float(np.linalg.norm(q))


def _back_off(p, h_star, theta_star, r_star, V_a, v_h, clamp=False,
              direction=None):
    """"后退让位/机器人避碰"分支（官方单位）：
    θ 指向背离最近羊（或给定方向），
    eta = −v·cos(θ*−θ) + sqrt(v²(cos²−1) + V²)，q = v·u(θ*) + eta·u(θ)；
    clamp 时把 eta 限制在"以最近羊为圆心、半径 r_star 的圆"内。"""
    if direction is None:
        d = -(h_star[:2] - p[:2])
        nn = float(np.linalg.norm(d))
        th = np.arctan2(d[1], d[0]) % (2 * np.pi) if nn > 1e-12 else 0.0
    else:
        th = float(direction)
    c = np.cos(theta_star - th)
    eta = -v_h * c + np.sqrt(max(v_h ** 2 * (c ** 2 - 1) + V_a ** 2, 0.0))
    if clamp:
        b_int, pos_int = _line_circle(p[:2], p[:2] + V_a * _unit(th),
                                      h_star[:2], r_star)
        if b_int:
            eta = min(eta, float(np.linalg.norm(p[:2] - pos_int)))
    q = v_h * _unit(theta_star) + eta * _unit(th)
    return np.arctan2(q[1], q[0]) % (2 * np.pi), float(np.linalg.norm(q))
