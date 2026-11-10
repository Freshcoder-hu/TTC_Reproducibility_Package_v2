"""全局仿真参数配置（论文第 2 章建模 + 5.2/5.3 实验环境 + 附录 A）。

新增（对应初稿第 5 章）：
- 局部有限观测（半径 r_sense、半角 phi_sense）与邻近机器人有限状态共享（r_share）
- 机器人自身安全约束（机器人—障碍物、机器人—机器人）
- B2 距离阈值（按代表性相对速度固定标定，运行中不再调整）
- 机器人—羊群比例 η = Nr/Ns 相关默认值
"""
from dataclasses import dataclass, replace


@dataclass
class SimCfg:
    # ---------- 时间 ----------
    dt: float = 0.05
    ctrl_period: int = 2          # 控制周期 0.1 s

    # ---------- 羊群（Couzin + C1-C5） ----------
    n_sheep: int = 36
    r_sheep: float = 0.35
    v_base: float = 1.0           # C1 下限
    v_flee: float = 1.5           # C1 上限
    turn_max: float = 3.0         # C2
    accel_max: float = 4.0        # C3
    r_rep: float = 1.1
    r_ori: float = 3.5
    r_att: float = 7.0
    r_avo: float = 4.0            # 羊对机器人规避距离（C4 上限由 avo_k 体现）
    avo_k: float = 2.2
    goal_bias: float = 0.25
    react_obs: bool = False       # 方案 A：羊对障碍物无自主逃逸
    r_avo_obs: float = 2.5
    avo_obs_k: float = 2.0

    # ---------- 机器人（unicycle + R1-R4） ----------
    n_robots: int = 6
    v_max: float = 2.6            # R1
    omega_max: float = 3.5        # R2
    a_lin: float = 3.0            # R3
    a_ang: float = 5.0            # R4
    kp: float = 2.0

    # ---------- 局部观测与状态共享（5.2/5.3） ----------
    r_sense: float = 10.0         # 机器人局部观测半径 Rs
    phi_sense: float = 1.40       # 扇形半角 φs [rad]（约 80°）
    r_share: float = 25.0         # 邻近机器人状态共享半径（覆盖整个机器人队）
    use_local_obs: bool = True    # 关闭则退化为全局观测（对照/调试用）

    # ---------- 机器人自身安全约束（4.10） ----------
    # d_rr_safe 必须与正常编队几何相容：先由 S1（无风险）实测编队最小间距，
    # 再取略低于该值的下限（见 verify 脚本 rr 标定输出；默认 2.0 m）。
    d_ro_safe: float = 1.2        # 机器人—障碍物最小安全距离（紧急避让）
    d_rr_safe: float = 2.0        # 机器人—机器人最小安全距离（硬约束优先）
    slot_gap: float = 0.3         # 编队槽位最小间距余量（槽位间距 >= d_rr_safe+slot_gap）
    rr_gain: float = 8.0          # 机器人间排斥强度

    # ---------- 离群归因（5.4） ----------
    # 用"客观风险状态"判定 out_risk / out_normal，而不是控制器内部标签：
    # risk_active = 存在障碍物且 TTC_min < risk_ref_ttc
    risk_ref_ttc: float = 12.0

    # ---------- 路径 ----------
    path_halfwidth: float = 6.0

    # ---------- 障碍物（O1-O2） ----------
    r_obs: float = 1.2
    v_obs_max: float = 6.0
    a_obs_max: float = 0.0        # >0 时启用保守 TTC

    # ---------- B2 距离阈值（5.5：按场景代表性相对速度固定标定） ----------
    # d_trigger = v_rel_ref(场景) × t_equiv，t_equiv 与 Ours 的 T_DIVERT 一致，
    # 使"距离法"与"TTC 法"具有同等标称预警时间，比较才公平。
    b2_t_equiv: float = 6.0
    b2_v_rel_ref: float = 2.5     # 未标定时的兜底值
    b2_d_trigger: float = 10.0    # 未标定时的兜底阈值
    b2_hyst: float = 2.0

    # ---------- 目标/几何/分析 ----------
    goal_frac: float = 0.92
    flock_radius_perc: float = 0.95
    cohesion_r: float = 3.0       # Cflock 连通判定半径
    viol_tol: float = 0.02

    # ---------- 终止条件（5.14） ----------
    terminate_on_collision: bool = False   # 默认把碰撞作为事件统计（便于观察恢复）
    stray_radius: float = 12.0             # 离队判定：距主要连通群超过该距离


def default_cfg(**overrides) -> SimCfg:
    return replace(SimCfg(), **overrides)
