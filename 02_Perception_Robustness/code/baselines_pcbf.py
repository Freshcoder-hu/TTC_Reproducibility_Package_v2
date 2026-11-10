"""B-PCBF：Nguyen et al., *Decentralized Multi-Robot Herding via Partial Containment
Strategy*, IEEE RA-L 11(5), 2026（IEEE doc 11455914）。

**复现等级：基于原文公式的机制级忠实复现**（作者未开源代码；论文中须声明）。
**不得**把"通用避碰 QP"当作该文方法 —— 本实现严格按原文五组公式构造：

## 原文机制（逐式）
1. **机器人动力学（式 1）**：双积分器 `ṗ_i = v_i, v̇_i = u_i`，`‖u_i‖ ≤ a_max^r`。
2. **CBF（式 7）**：`û = argmin ½‖u − u_nom‖² + p_ω(ω−ω_0)² + p_δδ²`
   s.t. 硬约束 `L_f h_r + L_g h_r u + ω α_r(h_r) ≥ 0`，
   软约束 `L_f h_k + L_g h_k u + ω α_k(h_k) ≥ δ`，`‖u‖ ≤ ‖u_max‖`。
3. **最小距离 CBF（式 8–13）**：速度障碍锥 `VO_ij`；`r_ij` 为 `Δp_ij` 对锥切向边界的
   负矢量投影（式 9，`‖r_ij‖ = d_min`）；`h_ij,min = r_ij^T Δv_ij`（式 10）；
   分布式半责任分解后写成 `A_ij,min u ≤ b_ij,min`（式 13）：
       `A = [ −r_ij^T , 0 , −½ α_min (r_ij^T Δv_ij) ]`，`b = ½ (ṙ_ij^T Δv_ij)`
   —— **硬约束，不加松弛**（原文：最小距离是关键约束）。
4. **最大距离 CBF（式 14–16）**：`h_ij,max = √(4a_max^r(d_max−‖Δp_ij‖)) − (Δp_ij^T Δv_ij)/‖Δp_ij‖`；
   写成 `A_ij,max u ≤ ½ b_ij,max`（式 16）：
       `A = [ Δp_ij^T , −1 , −½ (α_max h_ij,max³) ‖Δp_ij‖ ]`，
       `b = (Δv^TΔp)²/‖Δp‖² − ‖Δv‖² − 2a_max^r(Δv^TΔp)/√(4a_max^r(d_max−‖Δp‖))`
   —— **软约束**（原文：最大距离非关键，允许多机器人接近时的临时编队调整）。
5. **包围目标（式 18–19）**：`f(x) = (1 − e^{R_inf−x})³`，
   `u_i,s = −Σ_{j∈N_i} f(‖Δp_ij‖)·Δp_ij/‖Δp_ij‖ − k_t·(1/N_i)Σ_{j∈N_i} Δv_ij`。
6. **驱动目标（式 20–21）**：`Δp̄_ig = (1/N_i)Σ_j(p_j − g)`、`v̄_i = (1/N_i)Σ_j v_j`、
   `Δφ_i = arccos( v̄_i·Δp̄_ig /(‖v̄_i‖‖Δp̄_ig‖) )`，
   `u_i,d = −( v_i − Δφ_i·R(clamp(−Δφ_i, −π/2, π/2))·Δp̄_i/‖Δp̄_i‖ )`。
7. **相位切换（原文第 IV-C 节）**：`u_i,nom = k_s u_i,s + k_d u_i,d`；
   **包围相位**（k_s=1, k_d=0）直到 `min_j‖Δp_ij‖ ≤ R_inf` → 切换到**驱动相位**（k_s=0, k_d=1），
   直到离开影响区（`min_j‖Δp_ij‖ > R_inf`）再切回。
8. **信息边界**：仅用局部感知半径内的机器人与动物相对位置/速度 + 目标区域中心 g
   （原文"uses only locally available relative positions and velocities of sensed robots
   and animals within R^r_sensing"）；无 TTC / PRE / DIVERT / RECOVER。

## 原文未给数值的项（逐条记录，见 BASELINES_SPEC §8）
原文仿真参数表未随 PDF 文本导出（仅硬件实验给出无人机 0.5 m/s、地面机器人 0.1 m/s），
因此下列取值按**平台锚点或 O(1) 规范值**确定，并做敏感性诊断，**不得静默调参**：
| 项 | 取值 | 依据 |
|---|---|---|
| `R_inf`（影响半径） | `cfg.r_avo = 4.0 m` | 锚点：动物对"捕食者"的回避触发距离（与 VH 系列同一锚点规则） |
| `R^r_sensing` | `cfg.r_sense = 10.0 m` | 平台给定；满足原文条件 `R^r_sensing ≥ R_inf` |
| `d_min`（硬，最小间距） | `cfg.d_rr_safe = 2.0 m` | 语义 = 平台临界机器人间距 |
| `d_max`（软，最大间距） | `cfg.r_sense = 10.0 m` | 语义 = 编队连通/保持在感知范围内 |
| `a_max^r` | `cfg.a_lin` | 平台加速度上限（统一执行层 R3） |
| `α_min`/`α_max` | 1.0（线性 class-K） | 原文未给数值 → 线性 class-K 规范值 |
| `ω_0`, `p_ω`, `p_δ` | 1.0 / 1.0 / 1.0 | 同上（与 `½‖u−u_nom‖²` 同量级） |
| `k_t`（速度跟踪增益） | 1.0 | 与 `f(x)` 同量级（O(1)） |
| 目标 `g` | 路径 92% 弧长点 | 本文任务目标（与 success 判据一致） |

## 其余已记录的实现约定
* **式 (16) 的 δ 符号**：原文把 δ 的系数印为 `+1`（`LHS + δ ≤ RHS`），但该符号使 δ 变成"收紧"变量，
  与原文"非关键约束用 δ 松弛"的语义矛盾（且最优解恒为 δ=0，松弛机制形同虚设）。
  本实现取**使松弛语义成立**的符号（δ 移到右端，等价左端系数 `−1`），并在文档中如实记录该处原文符号存疑。
* **`ṙ_ij`**：用中心差分 `(r(Δp+hΔv) − r(Δp−hΔv))/(2h)`（`h = ½·ctrl_period·dt`）数值近似 Lie 导数。
* **`h_ij,max` 在 `‖Δp‖ ≥ d_max` 时**：`4a_max(d_max−‖Δp‖) < 0` 使根号无定义；此时取 `h=0`
  并保留约束（等价于把两人拉近），原因记录在文档。
* **加速度 → 统一执行层**：`v_cmd = v̂_i + u_i·(ctrl_period·dt)`，其中 `v̂_i` 由 `view.robot_pos`
  中心差分得到（机器人对自身速度的估计）。**该步只做运动学换算，不添加任何避障逻辑**。
* **QP 失败兜底**：以 `scipy.optimize.minimize(SLSQP)` 求解；若失败或不可行，
  取"名义控制裁剪到加速度球内 + δ=0 + ω=ω_0"（最近可行控制的投影），
  **不引入本文的避障逻辑**，并把每次失败计入诊断计数 `n_qp_fail`。
"""
import numpy as np

try:
    from scipy.optimize import minimize as _sp_minimize
    _HAS_SCIPY = True
except Exception:                                   # pragma: no cover
    _HAS_SCIPY = False

DEF_ALPHA = 1.0       # α_min / α_max（线性 class-K）
DEF_OMEGA0 = 1.0      # ω_0
DEF_PW = 1.0          # p_ω
DEF_PD = 1.0          # p_δ
DEF_KT = 1.0          # k_t


def _rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _r_ij(dp, d_min, theta=None):
    """式 (9)：Δp 对 VO 切向边界的负矢量投影，‖r‖ = d_min（‖Δp‖ ≥ d_min 时有定义）。"""
    n = float(np.linalg.norm(dp))
    if n < 1e-9:
        return np.zeros(2)
    if theta is None:
        s = min(d_min / max(n, d_min), 1.0)
        theta = float(np.arcsin(s))
    v = _rot(theta) @ dp
    return (float(dp @ v) / (n ** 2)) * v - dp


def _f_shaping(x, R_inf):
    """式 (18)：f(x) = (1 − e^{R_inf − x})³。"""
    return (1.0 - np.exp(R_inf - x)) ** 3


class BPCBHerder:
    """Partial Containment + CBF-QP 牧羊（机制级忠实复现）。"""

    name = "B-PCBF"

    def __init__(self, alpha=DEF_ALPHA, omega0=DEF_OMEGA0, pw=DEF_PW,
                 pd=DEF_PD, kt=DEF_KT, d_max_mult=None, goal_from_path=True,
                 R_inf_mult=1.0):
        self.alpha = float(alpha)
        self.omega0 = float(omega0)
        self.pw = float(pw)
        self.pd = float(pd)
        self.kt = float(kt)
        self.d_max_mult = d_max_mult        # None -> d_max = cfg.r_sense
        self.goal_from_path = bool(goal_from_path)
        self.R_inf_mult = float(R_inf_mult)  # 仅用于敏感性诊断（默认 1.0 = cfg.r_avo）
        self._phase = None                  # 1 = driving, 0 = surrounding（逐机器人）
        self._v = None                      # 机器人速度估计（中心差分）
        self._prev_pos = None
        self._c = None
        # 诊断（不作为结果指标）
        self.n_qp = 0
        self.n_qp_fail = 0
        self.n_fallback = 0   # 实际**使用兜底控制**的次数（与 n_qp_fail 区分）
        self.n_slack = 0
        self.n_switch = 0

    # ------------------------------------------------------------------
    def _goal_center(self, view, cfg):
        if self._c is not None:
            return self._c
        s = float(cfg.goal_frac) * float(view.path.total) \
            if view.path is not None else 0.0
        self._c = view.path.point_at(s) if view.path is not None else np.zeros(2)
        return self._c

    # ------------------------------------------------------------------
    def _solve_qp(self, u_nom, A, b, a_max):
        """式 (22)：4 变量 (u_x,u_y,δ,ω) 的 CBF-QP。返回 (u, ok, slack_active)。"""
        A = np.asarray(A, dtype=float).reshape(-1, 4) if len(A) else np.zeros((0, 4))
        b = np.asarray(b, dtype=float).reshape(-1) if len(b) else np.zeros(0)

        def cost(z):
            du = z[:2] - u_nom
            return 0.5 * float(du @ du) + self.pw * (z[3] - self.omega0) ** 2 \
                + self.pd * z[2] ** 2

        def grad(z):
            g = np.zeros(4)
            g[:2] = z[:2] - u_nom
            g[2] = 2.0 * self.pd * z[2]
            g[3] = 2.0 * self.pw * (z[3] - self.omega0)
            return g

        def bal(z):
            return a_max ** 2 - float(z[0] ** 2 + z[1] ** 2)

        def bal_jac(z):
            return np.array([-2 * z[0], -2 * z[1], 0.0, 0.0])

        cons = [{"type": "ineq", "fun": bal, "jac": bal_jac},
                {"type": "ineq", "fun": lambda z: z[2]}]      # δ ≥ 0
        for k in range(A.shape[0]):
            cons.append({"type": "ineq",
                         "fun": (lambda z, Ak=A[k], bk=b[k]: bk - float(Ak @ z)),
                         "jac": (lambda z, Ak=A[k]: -Ak)})
        self.n_qp += 1
        z0 = np.array([u_nom[0], u_nom[1], 0.0, self.omega0])
        if _HAS_SCIPY:
            try:
                res = _sp_minimize(cost, z0, jac=grad, constraints=cons,
                                   method="SLSQP",
                                   options=dict(maxiter=60, ftol=1e-6))
                if res.success and np.all(np.isfinite(res.x)):
                    z = res.x
                    slack = float(z[2]) > 1e-6
                    return z[:2], True, slack
            except Exception:
                pass
        self.n_qp_fail += 1
        # 兜底：名义控制投影到加速度球（不添加任何避障逻辑）
        # 注意：该控制**不保证 CBF 约束继续成立**；violation_rate 按实际下发
        # 控制量（含兜底）统计，实测为 0 只说明本样本未观测到违例，
        # 不构成兜底机制具有理论安全保证。
        self.n_fallback += 1
        n = float(np.linalg.norm(u_nom))
        u = u_nom * (a_max / n) if n > a_max else u_nom.copy()
        return u, False, False

    # ------------------------------------------------------------------
    def step(self, view, cfg):
        M = cfg.n_robots
        cmd = np.zeros((M, 2))
        if self._phase is None or len(self._phase) != M:
            self._phase = np.zeros(M, dtype=int)
            self._prev_pos = view.robot_pos.copy()
            self._v = np.zeros((M, 2))

        # 平台锚点（见模块 docstring 的参数表）
        R_inf = self.R_inf_mult * cfg.r_avo
        R_sense = cfg.r_sense
        d_min = cfg.d_rr_safe
        d_max = (self.d_max_mult * d_min) if self.d_max_mult else R_sense
        a_max = float(getattr(cfg, "a_lin", 1.0))
        T = cfg.ctrl_period * cfg.dt
        g = self._goal_center(view, cfg)

        # 机器人自身速度估计（中心差分，机器人可自测）
        pos = view.robot_pos
        self._v = (pos - self._prev_pos) / T if T > 0 else np.zeros_like(pos)
        self._prev_pos = pos.copy()

        sheep = view.sheep_pos
        svel = view.sheep_vel if len(view.sheep_vel) == len(sheep) \
            else np.zeros_like(sheep)

        for i in range(M):
            p_i, v_i = pos[i], self._v[i]
            # ---- 局部集合（原文：R^r_sensing 内） ----
            if len(sheep):
                d_s = np.linalg.norm(sheep - p_i, axis=1)
                inN = d_s <= R_sense
            else:
                d_s, inN = np.zeros(0), np.zeros(0, dtype=bool)
            d_r = np.linalg.norm(pos - p_i, axis=1)
            inM = (d_r > 1e-9) & (d_r <= R_sense)

            # ---- 相位切换（原文第 IV-C 节） ----
            dmin_sheep = float(d_s[inN].min()) if np.any(inN) else np.inf
            if self._phase[i] == 0:
                if dmin_sheep <= R_inf:
                    self._phase[i] = 1
                    self.n_switch += 1
            else:
                if not np.any(inN) or dmin_sheep > R_inf:
                    self._phase[i] = 0
                    self.n_switch += 1

            # ---- 名义控制（式 19 / 式 21） ----
            # 原文约定：Δp_ij = p_i − p_j、Δv_ij = v_i − v_j（机器人 − 动物）
            if self._phase[i] == 0:
                u_nom = np.zeros(2)
                if np.any(inN):
                    dp = p_i - sheep[inN]
                    dn = np.maximum(np.linalg.norm(dp, axis=1), 1e-9)
                    fx = _f_shaping(dn, R_inf)
                    dv = v_i - svel[inN]
                    u_nom = -np.sum((fx / dn)[:, None] * dp, axis=0) \
                        - self.kt * dv.mean(axis=0)
            else:
                u_nom = np.zeros(2)
                if np.any(inN):
                    pa = sheep[inN].mean(axis=0)
                    va = svel[inN].mean(axis=0)                 # v̄_i
                    dpa = p_i - pa                              # Δp̄_i
                    # 原文式(20) 印为 Δp̄_ig = mean(p_j) − g（由目标指向群体）。该符号下
                    # "已对准目标" ⇔ Δφ = π，会使驱动力在**无需修正时最大**，与原文所述
                    # 目标（"minimize the angular deviation between the animals' average
                    # heading and the goal direction"）相反。本实现按**原文陈述的语义**
                    # 取"由群体指向目标"的方向（g − p̄），使 Δφ = 0 ⇔ 已对准（此歧义已在
                    # BASELINES_SPEC §8 记录）。
                    pig = g - pa                                # 由群体指向目标
                    na, nb = float(np.linalg.norm(va)), float(np.linalg.norm(pig))
                    if na > 1e-9 and nb > 1e-9 and np.linalg.norm(dpa) > 1e-9:
                        cosphi = float(np.clip(va @ pig / (na * nb), -1.0, 1.0))
                        dphi = float(np.arccos(cosphi))
                        ang = float(np.clip(-dphi, -np.pi / 2, np.pi / 2))
                        v_des = dphi * (_rot(ang) @ (dpa / np.linalg.norm(dpa)))
                        u_nom = -(v_i - v_des)

            # ---- 约束装配（式 13 硬 / 式 16 软） ----
            A_list, b_list = [], []
            for j in np.where(inM)[0]:
                dpij = p_i - pos[j]
                dvij = v_i - self._v[j]
                nij = float(np.linalg.norm(dpij))
                if nij < 1e-9:
                    continue
                # 式 (13)：最小距离（硬）
                r = _r_ij(dpij, d_min)
                h = 0.5 * T
                rdot = (_r_ij(dpij + h * dvij, d_min)
                        - _r_ij(dpij - h * dvij, d_min)) / (2 * h)
                A_list.append(np.array([-r[0], -r[1], 0.0,
                                        -0.5 * self.alpha * float(r @ dvij)]))
                b_list.append(0.5 * float(rdot @ dvij))
                # 式 (16)：最大距离（软）
                arg = max(4.0 * a_max * (d_max - nij), 0.0)
                hmax = np.sqrt(arg) - float(dpij @ dvij) / nij
                hmax = max(hmax, 0.0) if arg <= 0.0 else hmax
                bb = (float(dpij @ dvij) ** 2) / (nij ** 2) - float(dvij @ dvij)
                if arg > 0:
                    bb -= 2.0 * a_max * float(dvij @ dpij) / np.sqrt(arg)
                A_list.append(np.array([dpij[0], dpij[1], -1.0,
                                        -0.5 * self.alpha * (hmax ** 3) * nij]))
                b_list.append(0.5 * bb)

            u, ok, slack = self._solve_qp(u_nom, A_list, b_list, a_max)
            self.n_slack += int(slack)
            # ---- 双积分器加速度 → 统一执行层的速度指令（纯运动学换算） ----
            cmd[i] = v_i + u * T
        return cmd, "pcbf"
