"""碰撞时间（TTC）计算模块（对应论文 3.2-3.4 节）。

- 常速度解析式 TTC（附录 C 推导的封闭解）；
- 含加速度的迭代预测（障碍物加速度上界 O2 -> 保守 TTC 区间）；
- 碰撞点预测与受威胁个体筛选。
"""
import numpy as np

_EPS = 1e-9


def _ttc_pair_analytic(r, v_rel, r_sum):
    """单对常速度 TTC 封闭解。
    r: 相对位置（羊 - 障碍物）；v_rel: 相对速度（羊 - 障碍物）。
    返回 TTC>=0 或 inf。"""
    rr = float(r @ r)
    rv = float(r @ v_rel)
    vv = float(v_rel @ v_rel)
    if vv < _EPS:
        return np.inf
    # 最近距离
    dmin2 = rr - rv * rv / vv
    rs2 = r_sum * r_sum
    if rv >= 0.0 or dmin2 > rs2:
        return np.inf
    return -rv / vv - np.sqrt(max(rs2 - dmin2, 0.0)) / np.sqrt(vv)


def _ttc_pair_iterative(pos, vel, o_pos, o_vel, r_sum, horizon,
                        a_obs=np.zeros(2), dtp=0.04):
    """数值迭代预测（含障碍物常加速度 a_obs）：
    对时间轴扫描，返回首次侵入时刻（TTC）。羊按当前速度直线外推。"""
    t = 0.0
    while t <= horizon:
        # 羊：x0 + v*t；障碍物：o0 + v_o*t + 0.5*a*t^2
        d = (pos + vel * t) - (o_pos + o_vel * t + 0.5 * a_obs * t * t)
        if np.linalg.norm(d) <= r_sum:
            return t
        t += dtp
    return np.inf


def ttc_matrix(sheep_pos, sheep_vel, obs_pos, obs_vel, r_sum,
               horizon=12.0, a_max=0.0, mode="analytic"):
    """对全部 羊×障碍物 对计算 TTC。

    mode='analytic'：常速度封闭解；
    mode='conservative'：a_max>0 时按障碍物最坏情况加速度（指向羊群、
    增大闭合速度）做数值搜索，返回保守（更小）TTC；a_max==0 时不进入
    搜索，直接调用封闭解（与 mode='analytic' 逐位相同）。

    返回 (T, i_min, j_min)，T 形状 (N,K)，inf 表示无碰撞风险。
    """
    N = sheep_pos.shape[0]
    K = len(obs_pos)
    T = np.full((N, K), np.inf)
    for i in range(N):
        p = sheep_pos[i]; v = sheep_vel[i]
        for j in range(K):
            o = obs_pos[j]; ov = obs_vel[j]
            if mode == "analytic":
                ttc = _ttc_pair_analytic(p - o, v - ov, r_sum)
            else:
                # 最坏情形（对羊最不利）：障碍物以 a_max 沿视线方向朝羊加速，
                # 即增大闭合速度（r = p_羊 - p_障碍 由障碍指向羊，故取 +rhat）。
                r = p - o
                if np.linalg.norm(r) > _EPS and a_max > 0:
                    rhat = r / np.linalg.norm(r)
                    vhat = rhat             # 指向羊 = 增大闭合速度（保守）
                    ttc = _ttc_pair_iterative(p, v, o, ov, r_sum, horizon,
                                              a_obs=vhat * a_max)
                else:
                    ttc = _ttc_pair_analytic(p - o, v - ov, r_sum)
            T[i, j] = ttc
    idx = np.unravel_index(np.argmin(T), T.shape) if np.isfinite(T).any() \
        else (0, 0)
    return T, int(idx[0]), int(idx[1])


def collision_point(sheep_pos, sheep_vel, o_pos, o_vel, i, j, ttc):
    """预测第 i 只羊与第 j 个障碍物在 ttc 时刻的碰撞点。"""
    ttc = float(ttc)
    if not np.isfinite(ttc):
        return None
    return sheep_pos[i] + sheep_vel[i] * ttc
