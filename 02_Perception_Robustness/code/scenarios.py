"""实验场景（论文 5.7-5.11 节）。

S1   无动态风险沿路径牧羊
S1N  S1-2：窄路径 + 急转弯（困难路径任务）
S2   单动态障碍物斜向冲突（核心闭环 NORMAL→PRE→DIVERT→RECOVER）
S2R3/S2R2/S2R1  不同初始风险等级（初始预测 TTC0 ≈ 3.5/2.5/1.5 s）
S3   连续动态风险（三个障碍物先后形成风险，新风险抢占 RECOVER）
S4S/S4M/S4L      机器人—羊群规模变化（η=Nr/Ns 相近的小/中/大组）
"""
import numpy as np

from config import default_cfg
from path import straight_path, bent_path
from obstacle import Obstacle


def scenario_1():
    """S1-1 正常路径（无动态风险）。"""
    return dict(name="S1_normal_path",
                path=straight_path(0, 70, halfwidth=6.0),
                obstacles=[], cfg=default_cfg(n_sheep=36, n_robots=12),
                T_max=130.0, s0=8.0)


def scenario_1n():
    """S1-2 窄路径 + 急转弯（无动态风险）。"""
    path = bent_path(0, 50.0, bend_at=28.0, bend_angle=1.31, halfwidth=4.0)
    return dict(name="S1N_narrow_turn", path=path, obstacles=[],
                cfg=default_cfg(n_sheep=36, n_robots=12),
                T_max=150.0, s0=6.0)


def calibrate_b2(cfg, obstacles, path=None, t_equiv=None, v_flock_ref=1.0):
    """按场景代表性相对速度固定标定 B2 的距离阈值（论文 5.5 节）。

    d_trigger = v_rel_ref(场景) × t_equiv，t_equiv 取 Ours 的 T_DIVERT，
    使"距离阈值法"与"TTC 法"具有**同等标称预警时间**；运行中不再调整。
    """
    t_equiv = cfg.b2_t_equiv if t_equiv is None else t_equiv
    if obstacles:
        rels = []
        for o in obstacles:
            v = np.asarray(o.vel, dtype=float)
            if path is not None:
                s, _, _ = path.closest(np.asarray(o.start, dtype=float))
                tang = path.tangent_at(s)
            else:
                tang = np.array([1.0, 0.0])
            rels.append(float(np.linalg.norm(v - v_flock_ref * tang)))
        cfg.b2_v_rel_ref = float(np.mean(rels))
    cfg.b2_d_trigger = cfg.b2_v_rel_ref * t_equiv
    return cfg


def scenario_2():
    """S2 单动态障碍物斜向冲突。"""
    cfg = default_cfg(n_sheep=36, n_robots=12)
    path = straight_path(0, 70, halfwidth=6.0)
    obs = [Obstacle([58.0, 18.0], [-0.8, -0.45], radius=cfg.r_obs,
                    life=130.0, t_spawn=0.0, a_max=cfg.a_obs_max)]
    calibrate_b2(cfg, obs, path)
    # 标定集(100-104)确定的场景级参数：单次冲突可用较大避让偏移量
    return dict(name="S2_oblique_conflict", path=path, obstacles=obs,
                cfg=cfg, T_max=125.0, s0=8.0,
                params=dict(divert_shift=3.0))


# --------------------------------------------------------------------------
# S2-R：不同初始风险等级（5.9 节）
# 与 S2 使用**完全相同的入口几何**（起点 58,18、方向 (-0.8,-0.45) 归一化），
# 仅改变障碍物**接近速度**倍数，使初始预测 TTC0 落在不同水平：
#   R3 慢（TTC0 大） -> R2 中 -> R1 快（TTC0 小）
# 这样既保证"仅改变初始位置或相对速度"，又能完整走 NORMAL→PRE→DIVERT。
# --------------------------------------------------------------------------
_S2_BASE_START = (58.0, 18.0)
_S2_BASE_DIR = np.array([-0.8, -0.45])
_S2_BASE_SPEED = float(np.linalg.norm(_S2_BASE_DIR))

_S2R_PARAMS = {
    "S2R3": dict(speed_factor=1.5),
    "S2R2": dict(speed_factor=2.2),
    "S2R1": dict(speed_factor=3.0),
}


def scenario_s2r(key):
    p = _S2R_PARAMS[key]
    cfg = default_cfg(n_sheep=36, n_robots=12)
    path = straight_path(0, 70, halfwidth=6.0)
    u = _S2_BASE_DIR / _S2_BASE_SPEED
    vel = u * (_S2_BASE_SPEED * p["speed_factor"])
    obs = [Obstacle(list(_S2_BASE_START), list(vel), radius=cfg.r_obs,
                    life=130.0, t_spawn=0.0, a_max=cfg.a_obs_max)]
    calibrate_b2(cfg, obs, path)
    return dict(name=f"S2_R_speedX{p['speed_factor']:.1f}", path=path,
                obstacles=obs, cfg=cfg, T_max=125.0, s0=8.0)


def scenario_3():
    """S3 连续动态风险（5.10 节）：三个风险事件**按固定时刻释放**，
    使 risk1 → risk2 → risk3 的顺序在不同 seed 下保持稳定。

    设计方法（不人为操控羊群，只固定障碍物释放）：
      羊群以 v_f ≈ 1.0 m/s 沿路径前进；障碍物从横向 14 m 处以 ~0.7 m/s
      斜向进入路径，横向行程约 20 s。因此把障碍物 i 的释放时刻设为
      t_i = {8, 42, 76} s、并在"预计羊群 20 s 后所处弧长"处进入，
      事件间隔 34 s ≫ 种子间到达时间波动（±3 s），保证时序稳定。
      障碍物半径 2.2 m，保证在 12 m 宽通道内**可规避**。
    """
    cfg = default_cfg(n_sheep=36, n_robots=12)
    path = straight_path(0, 110, halfwidth=6.0)
    v_flock = 1.0
    # ① 遭遇几何与 S2 一致（斜向擦边：只暴露羊群前缘，3 m 让道可规避），
    #    但按固定时刻释放三次，保证 risk1 -> risk2 -> risk3 顺序稳定。
    #    S2 标定关系：障碍以 (-0.8,-0.45) 从横向 18 m 接近，约 27.8 s 后与
    #    羊群在 x ≈ 8+1.0*27.8 处擦边相遇 => x0 = 8 + t_release + 50。
    t_release = [2.0, 30.0, 58.0]
    obs = []
    for i, t_r in enumerate(t_release):
        x0 = 8.0 + v_flock * t_r + 50.0
        y0 = 18.0 if i % 2 == 0 else -18.0
        v_y = -0.45 if y0 > 0 else 0.45
        obs.append(Obstacle([x0, y0], [-0.8, v_y], radius=cfg.r_obs,
                            life=60.0, t_spawn=t_r, a_max=cfg.a_obs_max))
    calibrate_b2(cfg, obs, path)
    # 标定集(100-104)结论：divert_shift 2.4 vs 3.0 逐 seed 仅 2/5 占优，
    # 且 ρ/max_dev 系统性更差 -> **不采用场景级差异**，统一 3.0
    return dict(name="S3_sequential_risk", path=path, obstacles=obs, cfg=cfg,
                T_max=200.0, s0=8.0, params=dict(divert_shift=3.0))


# --------------------------------------------------------------------------
# S4：机器人与羊群规模变化（η = Nr/Ns 相近，仅验证基本规模适应性）
# --------------------------------------------------------------------------
_S4_GROUPS = {
    "S4S": dict(n_sheep=18, n_robots=6),
    "S4M": dict(n_sheep=36, n_robots=12),
    "S4L": dict(n_sheep=60, n_robots=20),
}


def scenario_s4(key):
    g = _S4_GROUPS[key]
    ns, nr = g["n_sheep"], g["n_robots"]
    # 路径宽度与障碍物初始位置随羊群规模按 √Ns 缩放，保持相近的相对几何
    f = float(np.sqrt(ns / 36.0))
    hw = max(4.0, 6.0 * f)
    cfg = default_cfg(n_sheep=ns, n_robots=nr)
    path = straight_path(0, 70, halfwidth=hw)
    # 中规模组与 S2 使用相同规模，但障碍物初值不同（避免与 S2 数据重复）
    dx = 6.0 if key == "S4M" else 0.0
    obs = [Obstacle([58.0 * f + dx, 18.0 * f], [-0.8, -0.45],
                    radius=cfg.r_obs, life=130.0, t_spawn=0.0,
                    a_max=cfg.a_obs_max)]
    calibrate_b2(cfg, obs, path)
    return dict(name=f"S4_scale_{key[2:]}_Ns{ns}_Nr{nr}", path=path,
                obstacles=obs, cfg=cfg, T_max=160.0, s0=8.0)


# --------------------------------------------------------------------------
# 动态相对速度梯度场景（论文"TTC 适用条件"实验）
# 控制变量：保持**遭遇点固定**（路径 x=35 m），只改变障碍物速度倍数 f。
#   障碍物沿固定方向 u 从 E - u*(R0*f) 出发，速度 |v| = |v_base|*f，
#   因此遭遇点不变、而接近时间按 1/f 缩短 —— 即"相对接近速度"是唯一变量。
#   f=1.0 时与 S2 的遭遇强度相当。
# --------------------------------------------------------------------------
_VEL_DIR = np.array([-0.8, -0.45])
_VEL_DIR = _VEL_DIR / np.linalg.norm(_VEL_DIR)
_VEL_SPEED = float(np.linalg.norm([-0.8, -0.45]))
_VEL_E = np.array([35.0, 0.0])      # 固定遭遇点（路径中心线上）
_VEL_R0 = 24.8                      # f=1.0 时的初始斜距


def scenario_velocity(factor, lateral_offset=0.0):
    """动态相对速度梯度场景（factor 为速度倍数）。

    lateral_offset：遭遇点的横向偏移。为 0 时障碍物穿过羊群中心（接近
    "必撞"、方法差异被饱和掩盖）；取 3.6 m 时为**擦边式遭遇**（只威胁
    羊群上缘），避让在几何上可行，才能区分不同判据的预警能力。
    """
    cfg = default_cfg(n_sheep=36, n_robots=12)
    path = straight_path(0, 70, halfwidth=6.0)
    E = _VEL_E + np.array([0.0, lateral_offset])
    start = E - _VEL_DIR * (_VEL_R0 * factor)
    vel = _VEL_DIR * (_VEL_SPEED * factor)
    t_travel = (_VEL_R0 * factor) / (_VEL_SPEED * factor)
    obs = [Obstacle(list(start), list(vel), radius=cfg.r_obs,
                    life=3.0 * t_travel + 10.0, t_spawn=0.0,
                    a_max=cfg.a_obs_max)]
    calibrate_b2(cfg, obs, path)
    tag = "graze" if lateral_offset else "center"
    return dict(name=f"S_V_{tag}_speedX{factor:.1f}", path=path, obstacles=obs,
                cfg=cfg, T_max=125.0, s0=8.0,
                params=dict(divert_shift=3.0))


VELOCITY_LEVELS = [0.6, 1.0, 1.6, 2.4]
VELOCITY_LEVELS_GRAZE = [1.0, 1.6, 2.4]
GRAZE_LATERAL = 3.6

SCENARIOS = {
    "S1": scenario_1,
    "S1N": scenario_1n,
    "S2": scenario_2,
    "S2R3": lambda: scenario_s2r("S2R3"),
    "S2R2": lambda: scenario_s2r("S2R2"),
    "S2R1": lambda: scenario_s2r("S2R1"),
    "S3": scenario_3,
    "S4S": lambda: scenario_s4("S4S"),
    "S4M": lambda: scenario_s4("S4M"),
    "S4L": lambda: scenario_s4("S4L"),
    "V06": lambda: scenario_velocity(0.6),
    "V10": lambda: scenario_velocity(1.0),
    "V16": lambda: scenario_velocity(1.6),
    "V24": lambda: scenario_velocity(2.4),
}

# 快速运行（覆盖 RQ1-RQ4）；论文正式实验用 MAIN_SCENARIOS 全部场景
QUICK_SCENARIOS = ["S1", "S1N", "S2", "S3"]
MAIN_SCENARIOS = ["S1", "S1N", "S2", "S2R3", "S2R2", "S2R1", "S3",
                  "S4S", "S4M", "S4L"]
