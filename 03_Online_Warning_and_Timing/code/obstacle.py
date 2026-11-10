"""动态障碍物模型（对应论文 2.3 节）。

圆形障碍物沿直线匀速运动；到达终点自动消失。
支持两种出现方式：
- 定时出现：t_spawn 指定；
- 触发式出现：trigger=(s_lo,s_hi)，当羊群质心弧长进入区间时立即出现
  （模拟"车辆驶近才被感知到"，对所有算法公平且保证遭遇）。
非合作假设：不通信、不配合规避。
"""
import numpy as np


class Obstacle:
    def __init__(self, start, vel, radius=1.2, life=None, t_spawn=0.0,
                 a_max=0.0, trigger=None):
        self.start = np.asarray(start, dtype=float).copy()
        self.vel = np.asarray(vel, dtype=float)
        self.radius = radius
        self.life = life
        self.t_spawn = t_spawn
        self.a_max = a_max
        self.trigger = trigger          # (s_lo, s_hi) 或 None
        self.pos = self.start.copy()
        self.t0_active = None
        self.active = False
        self._triggered = False

    # ---- 生命周期 ----
    def _activate(self, t):
        self.active = True
        self.t0_active = t
        self.pos = self.start.copy()

    def check_trigger(self, s_c, t):
        """羊群质心弧长进入触发带 -> 立即出现。"""
        if self.trigger is not None and not self._triggered:
            lo, hi = self.trigger
            if lo <= s_c <= hi:
                self._triggered = True
                self._activate(t)

    def update(self, t):
        if self.trigger is None and not self.active and t >= self.t_spawn:
            self._activate(t)
        if self.active:
            self.pos = self.start + self.vel * (t - self.t0_active)
            if self.life is not None and (t - self.t0_active) >= self.life:
                self.active = False

    def is_alive(self):
        return self.active


def crossing_obstacle(cfg, x_cross, y_from=-16.0, speed=4.0, y_to=16.0,
                      t_spawn=0.0, trigger=None, radius=None):
    """自 (x_cross,y_from) 沿 +y 穿越路径的障碍物。"""
    radius = radius if radius is not None else cfg.r_obs
    life = (y_to - y_from) / speed
    return Obstacle([x_cross, y_from], [0.0, speed], radius=radius,
                    life=life, t_spawn=t_spawn, a_max=cfg.a_obs_max,
                    trigger=trigger)


def along_path_obstacle(cfg, s_start, direction, speed, path, y_lateral=0.0,
                        t_spawn=0.0, trigger=None, radius=None):
    """沿路径方向 (direction=+1 同向超车 / -1 对向) 运动的障碍物。"""
    radius = radius if radius is not None else cfg.r_obs
    p = path.point_at(float(s_start))
    tang = path.tangent_at(float(s_start))
    if y_lateral != 0.0:
        norm = np.array([-tang[1], tang[0]])
        p = p + norm * y_lateral
    s_end = float(np.clip(s_start + direction * 200.0, 0.0, path.total))
    life = abs(s_end - s_start) / max(abs(speed), 1e-6)
    v = tang * (direction * speed)
    return Obstacle(p, v, radius=radius, life=life, t_spawn=t_spawn,
                    a_max=cfg.a_obs_max, trigger=trigger)
