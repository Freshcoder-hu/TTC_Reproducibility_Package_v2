"""B-Outmost：Zhang et al., *A Distributed Outmost Push Approach for Multirobot
Herding*, IEEE T-RO 40(3):1705-1722, 2024（DOI 10.1109/TRO.2024.3359528）。

**复现等级：基于原文公式的机制级忠实复现**（作者未开源代码，公开渠道无仓库）。
论文中必须写明："由于原作者未提供开源代码，本文基于原文控制律与公式进行统一仿真
平台适配复现。"

## 原文控制律（逐式移植）

机器人（herder）动力学为一阶积分器，控制器为三力之和（式 4）：

    ẋ_i = u_i = k_o·f_i^obs + k_v·f_i^ove + k_s·f_i^ste                     (4)
    f_i^ste = −(x_i − s_i)                                                  (5)
    s_i = s_out + δR_e·(s_out − c)/‖s_out − c‖                              (6)

* **最外逃逸者（outmost evader）** s_out：机器人感知范围内（半径 R_h）**距目标中心 c
  最远**的逃逸者（原文 §V-A："the most distant evader from the goal center in the
  field of view of a herder"）。并列时原文"randomly chosen"，本实现取索引最小者（确定性）。
* **推压点** s_i：位于最外羊**背离目标**一侧、距离 δR_e 处（原文称 push range），
  即机器人绕到羊群后方把最外羊往目标方向推。0 < δ < 1。
* **f_i^ove**（机器人间重叠规避）：与逃逸者色散项同式（原文："the overlap avoidance
  force for herders ... has the same form as the one mentioned for evaders"），即
  式 (2) 把逃逸者位置换成机器人位置：
      f^ove = (1/N_i) Σ_{j∈N_i, j≠i} (x_i − x_j)/‖x_i − x_j‖³
* **f_i^obs**（障碍规避）：与逃逸者障碍规避同式（式 (3)）：
      f^obs = (1/O_i) Σ_{l∈O_i} (x_i − z_l)/‖x_i − z_l‖³
  原文假设障碍为**质点**、影响半径 R_e。
* **分布式 / 信息边界**：无通信、无共享、不需要全局信息（原文明确"no need to obtain
  global information about the evaders or herders (e.g., the GCM)"）；只用机器人
  自身感知半径内的羊、机器人与障碍 + 预先给定的目标区域中心 c。
* **不含** TTC / PRE / DIVERT / RECOVER / 专用安全屏障 —— 本文未添加任何此类逻辑。

## 参数来源（原文可获取的显式数值）

原文 §VII 物理实验段给出：`R_e = 800 mm`、`R_h = 1000 mm`、`δ = 0.3`、
`r = 500 mm`（目标区域半径）、`k_a = 3, k_d = 1, k_e = 4, k_o = 1, k_s = 3`。
仿真用 Table II 的数值**未出现在可获取的 PDF 文本中**（表格未随文本导出），故一律采用
上述显式数值，并逐项记录下列**未充分规定项**的处理（见 BASELINES_SPEC.md §7）：
  1. `k_v` 未给数值 → 取 `k_v = k_d = 1`（原文称重叠规避与色散同式，k_d 是色散参考强度）；
  2. `v_max^h / v_max^e` 未给数值 → 采用原文稳定性条件给出的临界值
     `v_max^e = k_s δ R_e = 0.72` 作为**速度锚点**（原文条件：`0 < v_max^e < k_s δR_e ≤ v_max^h`）；
     机器人速度上限由本文统一执行层 `cfg.v_max` 承担（不另设上限）；
  3. 机器人感知半径内无羊时原文未定义（假设 2 保证初始条件成立）→ 本实现按式 (4) 保留
     `f^ove + f^obs` 两项、`f^ste = 0`，**不额外引入搜索逻辑**；
  4. 障碍为圆（半径 r_obs）而非质点 → 距离与幂律取**到障碍表面的距离**
     `d = max(‖x−z‖ − r_obs, ε)`，方向仍沿 `x − z`；
  5. 目标中心 c：本文任务为"沿路径牧羊"，取**路径 92% 弧长点**（与 success 判据一致）；
  6. 原文的封闭软边界（arena boundary）不适用于本文场景，未实现。

## 量纲换算（与 BASELINES_SPEC §1 同一规则：长度锚 R_e→r_avo，速度锚 v_max^e→v_base）

    S = cfg.r_avo / R_e_official = 4.0/0.8 = 5.0      [本文 m per 原文 m]
    U = cfg.v_base / (k_s·δ·R_e)  = 1.0/0.72          [本文 m/s per 原文 m/s]

由此：`R_h = 5.0 m`、障碍影响半径 `R_e = 4.0 m`、推压距离 `δR_e = 1.2 m`；
系数按量纲缩放（f^ove/f^obs 的单位是 1/m²，f^ste 的单位是 m）：

    k_o, k_v  [原文 m³/s] → ×U·S² ；k_s [原文 1/s] → ×U/S

于是 k_o = k_v = 34.72 m³/s、k_s = 0.8333 s⁻¹。该缩放下有一个可核验的不变量：
**机器人距推压点 δR_e 处的指令速度恰等于羊群速度 `cfg.v_base`**（原文：该处速度 = k_s·δR_e = v_max^e）。
"""
import numpy as np

RE_OFF = 0.8        # 原文 R_e = 800 mm（逃逸者感知/影响半径）
RH_OFF = 1.0        # 原文 R_h = 1000 mm（机器人感知半径）
DELTA_OFF = 0.3     # 原文 δ（push range = δR_e）
KO_OFF = 1.0        # 原文 k_o
KV_OFF = 1.0        # k_v 未给数值，取 k_d = 1（见 docstring 第 1 条）
KS_OFF = 3.0        # 原文 k_s


class BOutmostHerder:
    """Distributed outmost push（机制级忠实复现，无通信、纯局部）。"""

    name = "B-Outmost"

    def __init__(self, Re=RE_OFF, Rh=RH_OFF, delta=DELTA_OFF, ko=KO_OFF,
                 kv=KV_OFF, ks=KS_OFF, goal_from_path=True, goal_s=None):
        self.Re_off = float(Re)
        self.Rh_off = float(Rh)
        self.delta = float(delta)
        self.ko_off = float(ko)
        self.kv_off = float(kv)
        self.ks_off = float(ks)
        self.goal_from_path = bool(goal_from_path)
        self.goal_s = goal_s          # None -> cfg.goal_frac * path.total
        self._c = None                # 目标中心（只取一次，视为预先给定的任务信息）
        self.n_cycles = 0             # 诊断：控制周期数
        self.n_noview = 0             # 诊断：感知半径内无羊的机器人-周期数

    # ------------------------------------------------------------------
    def _goal_center(self, view, cfg):
        """任务给定量：目标区域中心 c（本文取路径 goal_frac 弧长点）。"""
        if self._c is not None:
            return self._c
        if self.goal_s is not None:
            s = float(self.goal_s)
        elif view.path is not None:
            s = float(cfg.goal_frac) * float(view.path.total)
        else:
            s = 0.0
        self._c = (view.path.point_at(s) if view.path is not None
                   else np.zeros(2))
        return self._c

    # ------------------------------------------------------------------
    def step(self, view, cfg):
        M = cfg.n_robots
        cmd = np.zeros((M, 2))
        sheep = view.sheep_pos
        robots = view.robot_pos
        c = self._goal_center(view, cfg)

        # ---- 量纲换算（S 长度、U 速度；见模块 docstring） ----
        S = cfg.r_avo / self.Re_off
        v_e_off = self.ks_off * self.delta * self.Re_off     # k_s·δ·R_e（原文临界速度）
        U = (cfg.v_base / v_e_off) if v_e_off > 1e-12 else 1.0
        R_h = self.Rh_off * S                                # 机器人感知半径
        R_e = self.Re_off * S                                # 障碍影响半径
        push = self.delta * self.Re_off * S                  # δR_e
        k_o = self.ko_off * U * S ** 2
        k_v = self.kv_off * U * S ** 2
        k_s = self.ks_off * U / S

        obs = view.obs_pos if view.obs_active_n else np.zeros((0, 2))
        obs_r = np.array([o.radius for o in view.obstacles_active], dtype=float) \
            if view.obs_active_n else np.zeros(0)

        self.n_cycles += M

        for i in range(M):
            x = robots[i]
            u = np.zeros(2)

            # ---- f^ste（式 5-6）：最外羊 = 感知范围内距目标中心最远者 ----
            if len(sheep):
                d_s = np.linalg.norm(sheep - x, axis=1)
                vis = d_s <= R_h
                if not np.any(vis):
                    self.n_noview += 1
                if np.any(vis):
                    y = sheep[vis]
                    k = int(np.argmax(np.linalg.norm(y - c, axis=1)))   # 并列取最小索引
                    s_out = y[k]
                    g = s_out - c
                    ng = float(np.linalg.norm(g))
                    if ng > 1e-9:
                        s_i = s_out + push * (g / ng)
                        u += k_s * (s_i - x)                            # f^ste = s_i − x_i
                    # ng ≈ 0（最外羊已在目标中心）时无有效推压方向，f^ste 取 0

            # ---- f^ove（式 2 同式，位置换成机器人） ----
            if M > 1:
                rel = robots - x
                d_r = np.linalg.norm(rel, axis=1)
                sel = (d_r > 1e-9) & (d_r <= R_h)
                if np.any(sel):
                    dd = d_r[sel]
                    u += k_v * np.mean(rel[sel] / (dd ** 3)[:, None], axis=0)

            # ---- f^obs（式 3 同式；圆障碍取表面距离） ----
            if len(obs):
                vec = x - obs
                d_c = np.linalg.norm(vec, axis=1)
                d_surf = np.maximum(d_c - obs_r, 1e-6)
                sel = d_surf <= R_e
                if np.any(sel):
                    dirs = vec[sel] / np.maximum(d_c[sel], 1e-12)[:, None]
                    u += k_o * np.mean(dirs / (d_surf[sel] ** 2)[:, None], axis=0)

            cmd[i] = u
        return cmd, "outmost"
