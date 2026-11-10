"""羊群动力学（对应论文 2.1 节）。

Couzin 三区（排斥/对齐/吸引）+ 对机器人的规避反应（Aversion）
+ 可选的对障碍物自主逃逸（论文 2.1.3 方案 B，默认关闭）。
行为约束 C1-C3（速度范围、转向率上限）在积分步内强制执行；
C4（推力上限）体现在规避强度系数 avo_k / 斥力强度上限。
"""
import numpy as np


def _heading(angle):
    return np.stack([np.cos(angle), np.sin(angle)], axis=1)


class SheepState:
    def __init__(self, pos, head, speed):
        self.pos = np.asarray(pos, dtype=float)      # (N,2)
        self.head = np.asarray(head, dtype=float)    # (N,)
        self.speed = np.asarray(speed, dtype=float)  # (N,)
        self.alive = np.ones(len(pos), dtype=bool)

    @property
    def vel(self):
        return self.speed[:, None] * _heading(self.head)

    @property
    def n(self):
        return self.pos.shape[0]


def init_flock(n, center=(8.0, 0.0), radius=3.0, rng=None, v0=1.0):
    """在圆盘内撒 n 只羊，初始方向朝 +x。"""
    rng = rng if rng is not None else np.random.default_rng(0)
    ang = rng.uniform(0, 2 * np.pi, n)
    rad = np.sqrt(rng.uniform(0, 1, n)) * radius
    pos = np.array(center) + rad[:, None] * np.stack(
        [np.cos(ang), np.sin(ang)], axis=1)
    head = rng.uniform(-0.4, 0.4, n)   # 大致朝 +x
    return SheepState(pos, head, np.full(n, v0))


def advance_sheep(st, cfg, robots_pos, path=None):
    """推进羊群一个 dt。robots_pos: (M,2)。
    默认方案 A：羊只对机器人有规避反应，对障碍物无反应。
    path 提供时叠加"趋向路径"弱偏好（goal_bias）。"""
    return _advance_sheep(st, cfg, robots_pos, None, path=path)


def _advance_sheep(st, cfg, robots_pos, obs_pos, path=None):
    """核心积分。obs_pos 非空且 cfg.react_obs=True 时，
    障碍物作为额外斥力源（方案 B），否则忽略。"""
    N = st.n
    pos = st.pos[st.alive]
    head = st.head[st.alive]
    spd = st.speed[st.alive]
    m = len(pos)
    if m == 0:
        return

    d = _heading(head)

    # ---- 羊-羊成对作用（向量化）----
    dpos = pos[:, None, :] - pos[None, :, :]          # (m,m,2)
    dist = np.linalg.norm(dpos, axis=2)
    np.fill_diagonal(dist, np.inf)

    rep_m = (dist < cfg.r_rep)
    ali_m = (dist >= cfg.r_rep) & (dist < cfg.r_ori)
    att_m = (dist >= cfg.r_ori) & (dist < cfg.r_att)

    rep_dir = np.zeros((m, 2))
    rep_n = np.zeros(m)
    with np.errstate(invalid="ignore", divide="ignore"):
        u = dpos / np.maximum(dist[..., None], 1e-6)      # 指向邻居
    rep_dir = np.where(rep_m[..., None], u, 0.0).sum(axis=1)
    rep_n = rep_m.sum(axis=1)

    # ---- 斥力源集合（机器人，可选障碍物）----
    src_pos = [robots_pos]
    src_range = [cfg.r_avo]
    src_k = [cfg.avo_k]
    if cfg.react_obs and obs_pos is not None and len(obs_pos):
        src_pos.append(obs_pos)
        src_range.append(cfg.r_avo_obs)
        src_k.append(cfg.avo_obs_k)

    rob_dir = np.zeros((m, 2))
    rob_n = np.zeros(m)
    for sp, sr, sk in zip(src_pos, src_range, src_k):
        dsrc = pos[:, None, :] - sp[None, :, :]
        dsrc_d = np.linalg.norm(dsrc, axis=2)
        src_m = dsrc_d < sr
        for i in range(m):
            idx = np.where(src_m[i])[0]
            if len(idx):
                dd = dsrc_d[i, idx]
                w = sk * (1.0 - dd / sr)
                rob_dir[i] += (dsrc[i, idx] / np.maximum(dd, 1e-6)[:, None]
                               * w[:, None]).sum(axis=0)
                rob_n[i] += len(idx)

    flee = (rob_n > 0)

    # ---- 对齐 / 吸引（仅当无排斥性邻居时生效）----
    steer = np.zeros((m, 2))
    if (~flee).any():
        nf = ~flee
        # 对齐：邻居速度方向平均
        dd = d
        ali_d = np.where(ali_m[..., None], dd[None, :, :], 0.0).sum(axis=1)
        # 吸引：指向邻居质心
        att_cnt = np.where(att_m[..., None], pos[None, :, :], 0.0).sum(axis=1)
        att_num = att_m.sum(axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            att_v = att_cnt / np.maximum(att_num[:, None], 1e-6) - pos
        steer = 0.6 * ali_d + 0.4 * att_v
        # 趋向路径弱偏好（每只羊沿其所在弧长的路径切向缓行）
        if path is not None and cfg.goal_bias > 0:
            s_i, _, _ = path.closest(pos)
            for i in np.where(nf)[0]:
                steer[i] = steer[i] + cfg.goal_bias * path.tangent_at(s_i[i])

    # ---- 合成期望方向 ----
    want = np.zeros((m, 2))
    rep_act = (rep_n > 0) | flee                     # Couzin：排斥主导
    rep_total = rep_dir + rob_dir                    # 羊排斥 + 机器人规避
    rn = np.linalg.norm(rep_total, axis=1)
    use = rep_act & (rn > 1e-6)
    want[use] = rep_total[use] / np.maximum(rn[use], 1e-6)[:, None]
    other = ~use
    sn = np.linalg.norm(steer, axis=1)
    good = other & (sn > 1e-6)
    want[good] = steer[good] / sn[good, None]
    # 仍无方向者保持原方向
    idle = ~(use | good)
    want[idle] = d[idle]

    # ---- 转向率上限（C2）----
    cur = head
    target = np.arctan2(want[:, 1], want[:, 0])
    dth = np.angle(np.exp(1j * (target - cur)))
    dth = np.clip(dth, -cfg.turn_max * cfg.dt, cfg.turn_max * cfg.dt)
    head = cur + dth

    # ---- 速度（C1）：逃逸加速，否则基准速度 ----
    spd = np.where(flee, cfg.v_flee, cfg.v_base)
    # C3：速度变化率限制
    spd = np.clip(spd, cfg.v_base, cfg.v_flee)

    # ---- 位置积分 ----
    pos = pos + spd[:, None] * _heading(head) * cfg.dt

    st.pos[st.alive] = pos
    st.head[st.alive] = head
    st.speed[st.alive] = spd


def advance_sheep_with_obs(st, cfg, robots_pos, obs_pos):
    """方案 B：羊对障碍物也有自主逃逸反应（鲁棒性实验用）。"""
    _advance_sheep(st, cfg, robots_pos, obs_pos)
