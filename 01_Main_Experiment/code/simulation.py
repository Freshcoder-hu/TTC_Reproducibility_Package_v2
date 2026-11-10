"""仿真编排（论文 5.2 环境 / 5.13 指标 / 5.14 终止条件）。

- 局部有限观测：机器人扇形观测（半径 Rs、半角 φs）+ 邻近机器人有限状态共享；
  控制器只能使用"观测 ∪ 共享"得到的羊群状态（全局真值仅用于实验评价）。
- 机器人自身安全：机器人—机器人、机器人—障碍物安全检查（所有方法一致）。
- 逐控制周期记录：ρ、路径偏离、凝聚率、三类最小距离、TTC、感知覆盖、
  模式标签、控制变化幅度 Δu、角色切换次数等。
"""
import time
import numpy as np

from sheep import init_flock, advance_sheep
from robots import RobotFleet, slot_positions
from obstacle import Obstacle
from ttc import ttc_matrix


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def largest_cluster(pos, r_conn):
    """返回最大连通分量掩码（连通判定半径 r_conn）。"""
    n = len(pos)
    mask = np.zeros(n, dtype=bool)
    if n == 0:
        return mask
    seen = np.zeros(n, dtype=bool)
    best = np.zeros(n, dtype=bool)
    for i in range(n):
        if seen[i]:
            continue
        comp = [i]
        seen[i] = True
        stack = [i]
        while stack:
            k = stack.pop()
            d = np.linalg.norm(pos - pos[k], axis=1)
            for j in np.where((~seen) & (d <= r_conn))[0]:
                seen[j] = True
                stack.append(int(j))
                comp.append(int(j))
        if len(comp) > best.sum():
            best = np.zeros(n, dtype=bool)
            best[comp] = True
    return best


class View:
    """控制器只读视图（局部观测 ∪ 邻近共享后的羊群状态）。"""

    def __init__(self):
        self.t = 0.0
        self.path = None
        self.sheep = None
        self.sheep_pos = np.zeros((0, 2))       # 可用状态（观测∪共享）
        self.sheep_vel = np.zeros((0, 2))
        self.sheep_pos_global = np.zeros((0, 2))  # 全局真值（仅评价用）
        self.sheep_vel_global = np.zeros((0, 2))
        self.mask_direct = np.zeros(0, dtype=bool)
        self.mask_shared = np.zeros(0, dtype=bool)
        self.robot_pos = np.zeros((0, 2))
        self.obstacles_active = []
        self.obs_pos = np.zeros((0, 2))
        self.obs_vel = np.zeros((0, 2))
        self.obs_active_n = 0

    @property
    def centroid(self):
        return self.sheep_pos.mean(axis=0) if len(self.sheep_pos) \
            else np.zeros(2)

    def refresh(self, sheep, robots, cfg, obstacles):
        self.sheep = sheep
        self.robot_pos = robots.pos.copy()
        act = [o for o in obstacles if o.is_alive()]
        self.obstacles_active = act
        if act:
            self.obs_pos = np.array([o.pos for o in act]).reshape(-1, 2)
            self.obs_vel = np.array([o.vel for o in act]).reshape(-1, 2)
        else:
            self.obs_pos = np.zeros((0, 2))
            self.obs_vel = np.zeros((0, 2))
        self.obs_active_n = len(act)

        alive = sheep.alive
        pos_g = sheep.pos[alive]
        vel_g = sheep.vel[alive]
        self.sheep_pos_global = pos_g
        self.sheep_vel_global = vel_g
        n = len(pos_g)
        if n == 0:
            self.mask_direct = np.zeros(0, dtype=bool)
            self.mask_shared = np.zeros(0, dtype=bool)
            self.sheep_pos = np.zeros((0, 2))
            self.sheep_vel = np.zeros((0, 2))
            return

        if not cfg.use_local_obs:
            m = np.ones(n, dtype=bool)
            self.mask_direct = m
            self.mask_shared = m
            self.sheep_pos = pos_g
            self.sheep_vel = vel_g
            return

        # ---- 机器人局部观测 ----
        #   mask_fan : 扇形（半径 Rs、半角 phi）—— 论文 5.3 的"局部有效影响/观测范围"，
        #              用于统计感知覆盖率 Csense（5.13.11）；
        #   mask_omni: 邻近半径观测 —— 用于控制器输入（风险预测/编队几何），
        #              避免因扇形朝向而漏掉真正处于危险中的个体，使 TTC 评估保守。
        M = len(robots.pos)
        fan = np.zeros((M, n), dtype=bool)
        omni = np.zeros((M, n), dtype=bool)
        for j in range(M):
            rel = pos_g - robots.pos[j]
            dist = np.linalg.norm(rel, axis=1)
            head = np.array([np.cos(robots.head[j]), np.sin(robots.head[j])])
            cosang = (rel @ head) / np.maximum(dist, 1e-6)
            ang = np.arccos(np.clip(cosang, -1.0, 1.0))
            fan[j] = (dist <= cfg.r_sense) & (ang <= cfg.phi_sense)
            omni[j] = dist <= cfg.r_sense
        mask_direct = fan.any(axis=0)

        # ---- 邻近机器人有限状态共享（r_share 内的观测并集）----
        mask_shared = omni.any(axis=0)
        for j in range(M):
            for k in range(j + 1, M):
                if np.linalg.norm(robots.pos[j] - robots.pos[k]) <= cfg.r_share:
                    mask_shared |= (omni[j] | omni[k]) | (fan[j] | fan[k])
        if not mask_shared.any():               # 兜底：避免控制器无输入
            mask_shared = np.ones(n, dtype=bool)

        self.mask_direct = mask_direct
        self.mask_shared = mask_shared
        self.sheep_pos = pos_g[mask_shared]
        self.sheep_vel = vel_g[mask_shared]


def _init_robots_ring(view, cfg, s0, form="ring", R_fl=3.0):
    c = view.centroid if len(view.sheep_pos) else view.path.point_at(s0)
    tang = view.path.tangent_at(s0)
    slots, _ = slot_positions(c, R_fl, tang, cfg, form=form)
    fleet = RobotFleet(slots, cfg=cfg)
    fleet.head[:] = np.arctan2(tang[1], tang[0])
    return fleet


def run_scene(cfg, path, obstacles, controller, seed=0, T_max=180.0,
              s0=8.0, trace=0.0, init=None):
    """`init` 可覆盖初始编队（默认 None = 冻结协议：form="ring", R_fl=3.0）。
    仅用于**方法适配性核验**（例如外部论文自身要求的初始条件），
    默认路径与冻结实验完全一致。"""
    init = init or {}
    rng = np.random.default_rng(seed)
    flock = init_flock(cfg.n_sheep, path.point_at(s0), radius=2.8, rng=rng)
    view = View()
    view.path = path
    view.sheep = flock
    view.refresh(flock, RobotFleet(np.zeros((cfg.n_robots, 2))), cfg,
                 obstacles)
    robots = _init_robots_ring(view, cfg, s0, form=init.get("form", "ring"),
                               R_fl=init.get("R_fl", 3.0))

    obs = [Obstacle(o.start.copy(), o.vel.copy(), o.radius, o.life,
                    o.t_spawn, o.a_max) for o in obstacles]

    n0 = cfg.n_sheep
    lost = 0
    collided = np.zeros(n0, dtype=bool)
    ctrl_ms_acc = 0.0
    ctrl_n = 0
    step = 0
    logs = []
    success_t = None
    timeout = False
    terminated_by = "time"
    t = 0.0
    dt = cfg.dt
    cp = cfg.ctrl_period
    cmd = np.zeros((cfg.n_robots, 2))
    prev_cmd = None
    tag = "init"

    tol = cfg.viol_tol
    robot_viol = sheep_viol = n_dt = 0
    dmin_ro = np.inf
    dmin_rr = np.inf
    rr_info = (np.inf, 0.0, "?", -1, -1)     # (距离, 时刻, 模式, 机器人i, 机器人j)
    dmin_so = np.inf
    cvar_acc = 0.0
    cvar_n = 0
    out_normal = set()
    out_risk = set()
    seen_out = set()

    while t < T_max:
        view.t = t
        view.refresh(flock, robots, cfg, obs)

        if step % cp == 0:
            t0 = time.perf_counter()
            cmd, tag = controller.step(view, cfg)
            ctrl_ms_acc += (time.perf_counter() - t0) * 1e3
            ctrl_n += 1
            if prev_cmd is not None:
                cvar_acc += float(np.linalg.norm(
                    cmd - prev_cmd, axis=1).mean())
                cvar_n += 1
            prev_cmd = cmd.copy()

        prev_head = robots.head.copy()
        prev_shead = flock.head[flock.alive].copy()
        robots.step(cmd, cfg, view.obs_pos if view.obs_active_n else None)
        advance_sheep(flock, cfg, robots.pos, path)

        # ---- 约束检查（实际状态，5.5.4 / 4.14）----
        if np.any(robots.speed > cfg.v_max * (1 + tol)):
            robot_viol += 1
        dth = _wrap(robots.head - prev_head)
        if np.any(np.abs(dth) / dt > cfg.omega_max * (1 + tol)):
            robot_viol += 1
        spd = flock.speed[flock.alive]
        if spd.size:
            if np.any(spd > cfg.v_flee * (1 + tol)) or \
                    np.any(spd < cfg.v_base * (1 - tol)):
                sheep_viol += 1
            dth_s = _wrap(flock.head[flock.alive] - prev_shead)
            if np.any(np.abs(dth_s) / dt > cfg.turn_max * (1 + tol)):
                sheep_viol += 1
        n_dt += 1

        # ---- 机器人自身安全距离统计（4.10 / 5.13.9）+ 最小值定位诊断 ----
        if cfg.n_robots > 1:
            for i in range(cfg.n_robots):
                for j in range(i + 1, cfg.n_robots):
                    dd = float(np.linalg.norm(robots.pos[i] - robots.pos[j]))
                    if dd < dmin_rr:
                        dmin_rr = dd
                        rr_info = (dd, t, tag, i, j)
        for o in obs:
            if o.is_alive():
                dmin_ro = min(dmin_ro, float(np.linalg.norm(
                    robots.pos - o.pos, axis=1).min()))
                if flock.alive.any():
                    dd = np.linalg.norm(flock.pos[flock.alive] - o.pos,
                                        axis=1).min()
                    dmin_so = min(dmin_so, float(dd))

        for o in obs:
            o.update(t + dt)

        # ---- 碰撞检测（羊—障碍物；事件统计，不删除个体）----
        view.refresh(flock, robots, cfg, obs)
        if view.obs_active_n and flock.alive.any():
            for i in np.where(flock.alive)[0]:
                p = flock.pos[i]
                hit = any(np.linalg.norm(p - o.pos) < cfg.r_sheep + o.radius
                          for o in view.obstacles_active)
                if hit and not collided[i]:
                    collided[i] = True
                    lost += 1

        # ---- 日志 ----
        if step % cp == 0:
            n_alive = int(flock.alive.sum())
            pos = view.sheep_pos_global
            ttc_min = np.inf
            if n_alive:
                inside = path.inside_frac(pos)
                rho = float(inside * n_alive / n0)
                s_c, lat_c, _ = path.closest(pos.mean(axis=0))
                cl = largest_cluster(pos, cfg.cohesion_r)
                cflock = float(cl.sum()) / n_alive
                # 离群归因用**客观风险状态**（而非控制器内部标签）：
                # risk_active = 存在活跃障碍物且当前 TTC_min < risk_ref_ttc
                risk_active = bool(view.obs_active_n) and \
                    np.isfinite(ttc_min) and (ttc_min < cfg.risk_ref_ttc)
                for k in np.where(~cl)[0]:
                    if k not in seen_out:
                        seen_out.add(int(k))
                        if risk_active:
                            out_risk.add(int(k))
                        else:
                            out_normal.add(int(k))
                dmin_s = np.inf
                for o in view.obstacles_active:
                    dmin_s = min(dmin_s, float(np.linalg.norm(
                        pos - o.pos, axis=1).min()))
                if np.isfinite(dmin_s):
                    dmin_so = min(dmin_so, dmin_s)
                T, _, _ = ttc_matrix(pos, view.sheep_vel_global,
                                     view.obs_pos, view.obs_vel,
                                     cfg.r_sheep + cfg.r_obs, horizon=12.0)
                ttc_min = float(np.min(T)) if np.isfinite(T).any() else np.inf
            else:
                rho, s_c, lat_c, cflock, dmin_s = 0.0, 0.0, 0.0, 0.0, np.inf
                ttc_min = np.inf
            cov_dir = float(view.mask_direct.mean()) if len(
                view.mask_direct) else 0.0
            cov_sha = float(view.mask_shared.mean()) if len(
                view.mask_shared) else 0.0
            # ---- 逐障碍物 TTC（索引与场景障碍物列表对齐，用于风险事件时序）----
            K = len(obs)
            ttc_each = np.full(K, np.inf)
            if K and n_alive:
                p_all = np.array([o.pos if o.is_alive() else o.start
                                  for o in obs], dtype=float)
                v_all = np.array([o.vel for o in obs], dtype=float)
                Tall, _, _ = ttc_matrix(pos, view.sheep_vel_global, p_all,
                                        v_all, cfg.r_sheep + cfg.r_obs,
                                        horizon=12.0)
                for k, o in enumerate(obs):
                    col = Tall[:, k]
                    ttc_each[k] = float(np.min(col)) \
                        if (o.is_alive() and np.isfinite(col).any()) else np.inf
            # ---- Nguyen 2026 附录乙表指标（逐控制周期记录）----
            #   d_goal    : ‖羊群质心 − g‖（g = 路径 goal_frac 弧长点，全方法共用）
            #   d_range   : 分布范围 = 质心到最远羊的距离
            #   head_err  : 朝向误差 Δφ（度）= 羊群平均速度方向 与 "质心→目标"方向 的夹角
            #   nguyen_ok : 是否**同时**满足原文成功判据（d_goal ≤ 2.0 m 且 d_range ≤ 12 m）
            c_goal = view.path.point_at(cfg.goal_frac * view.path.total)
            if n_alive:
                cnt = pos.mean(axis=0)
                d_goal = float(np.linalg.norm(cnt - c_goal))
                d_range = float(np.max(np.linalg.norm(pos - cnt, axis=1)))
                vbar = view.sheep_vel_global.mean(axis=0)
                gdir = c_goal - cnt
                ng, nv = float(np.linalg.norm(gdir)), float(np.linalg.norm(vbar))
                head_err = (float(np.degrees(np.arccos(np.clip(
                    float(vbar @ gdir) / max(ng * nv, 1e-9), -1.0, 1.0))))
                    if (ng > 1e-9 and nv > 1e-9) else np.nan)
                nguyen_ok = bool(d_goal <= 2.0 and d_range <= 12.0)
            else:
                d_goal = d_range = np.inf
                head_err = np.nan
                nguyen_ok = False
            logs.append(dict(t=t, rho=rho, lat=lat_c, ttc=ttc_min,
                             ttc_each=ttc_each.tolist(),
                             dmin=(dmin_s if np.isfinite(dmin_s) else np.inf),
                             cflock=cflock, cov_direct=cov_dir,
                             cov_shared=cov_sha, lost=lost, tag=tag,
                             d_goal=d_goal, d_range=d_range,
                             head_err=head_err, nguyen_ok=nguyen_ok))

        # ---- 终止条件（5.14）----
        if flock.alive.any():
            s_c, _, _ = path.closest(flock.pos[flock.alive].mean(axis=0))
            if s_c >= cfg.goal_frac * path.total:
                success_t = t
                terminated_by = "goal"
                break
        if cfg.terminate_on_collision and lost > 0:
            terminated_by = "collision"
            break
        if trace > 0 and step % max(1, int(trace / dt)) == 0:
            c = flock.pos[flock.alive].mean(axis=0) if flock.alive.any() \
                else np.zeros(2)
            s_c, lat_c, _ = path.closest(c)
            print(f"t={t:6.1f} s={s_c:6.2f} lat={lat_c:6.2f} "
                  f"rho={logs[-1]['rho']:.2f} lost={lost} tag={tag}",
                  flush=True)
        t += dt
        step += 1
    else:
        timeout = True

    ctrl_ms = ctrl_ms_acc / max(ctrl_n, 1)
    return dict(
        cfg=cfg, path=path, logs=logs, lost=lost, n0=n0,
        success=success_t is not None, success_t=success_t,
        timeout=bool(timeout), terminated_by=terminated_by,
        ctrl_ms=ctrl_ms, controller=type(controller).__name__,
        controller_obj=controller,
        violation_rate=(robot_viol + sheep_viol) /
        max(n_dt * (cfg.n_robots + cfg.n_sheep), 1),
        dmin_robot_obs=dmin_ro, dmin_robot_robot=dmin_rr,
        rr_min_t=rr_info[1], rr_min_tag=rr_info[2],
        rr_min_pair=(rr_info[3], rr_info[4]),
        dmin_sheep_obs=dmin_so,
        control_var=cvar_acc / max(cvar_n, 1),
        out_normal=len(out_normal), out_risk=len(out_risk),
        role_switches=getattr(controller, "role_switch_count", np.nan),
    )
