"""路径模型（对应论文 2.4 节）。

路径 = 折线中心线 {P_m} + 逐顶点半宽（宽度沿弧长线性插值，
支持用高斯凸包模拟"可变宽度"，对应论文 3 的 GMM 宽度思想）。
"""
import numpy as np


class Path:
    def __init__(self, verts, halfwidths=None, halfwidth=6.0):
        """verts: (K,2) 中心线顶点（按行进顺序）；
        halfwidths: 每顶点半宽（标量或长度 K 数组）。"""
        self.verts = np.asarray(verts, dtype=float)
        segs = np.diff(self.verts, axis=0)                 # (K-1,2)
        self.seg_len = np.linalg.norm(segs, axis=1)        # (K-1,)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total = float(self.cum[-1])
        # 每段单位方向与其法向
        ok = self.seg_len > 1e-9
        self.tang = np.zeros_like(segs)
        self.tang[ok] = segs[ok] / self.seg_len[ok, None]
        self.norm = np.stack([-self.tang[:, 1], self.tang[:, 0]], axis=1)
        if halfwidths is None:
            halfwidths = halfwidth
        hw = np.asarray(halfwidths, dtype=float)
        if hw.ndim == 0:
            hw = np.full(len(verts), float(hw))
        self.halfwidth = np.asarray(hw, dtype=float)       # (K,)

    # ---------- 几何查询 ----------
    def closest(self, pts):
        """pts:(...,2) -> (s, lateral, dist)。s 为弧长（夹在 [0,total]），
        lateral 为到中心线的有符号距离（法向左侧为正）。"""
        pts = np.asarray(pts, dtype=float)
        shape = pts.shape
        p = pts.reshape(-1, 2)
        n = p.shape[0]
        K = len(self.verts)
        s_best = np.zeros(n); lat_best = np.zeros(n); d_best = np.full(n, np.inf)
        for k in range(K - 1):
            a = self.verts[k]; b = self.verts[k + 1]
            d = b - a; L2 = float(d @ d)
            if L2 < 1e-12:
                continue
            t = np.clip(((p - a) @ d) / L2, 0.0, 1.0)
            q = a[None, :] + t[:, None] * d[None, :]
            diff = p - q
            dist = np.linalg.norm(diff, axis=1)
            lat = diff @ self.norm[k]                       # 有符号横向
            s = self.cum[k] + t * self.seg_len[k]
            upd = dist < d_best
            d_best[upd] = dist[upd]
            s_best[upd] = s[upd]
            lat_best[upd] = lat[upd]
        # 端点之外按最近端点处理
        return (s_best.reshape(shape[:-1]), lat_best.reshape(shape[:-1]),
                d_best.reshape(shape[:-1]))

    def point_at(self, s):
        s = float(np.clip(s, 0, self.total))
        k = int(np.searchsorted(self.cum, s, side="right") - 1)
        k = min(max(k, 0), len(self.verts) - 2)
        t = (s - self.cum[k]) / max(self.seg_len[k], 1e-9)
        return self.verts[k] + t * (self.verts[k + 1] - self.verts[k])

    def tangent_at(self, s):
        s = float(np.clip(s, 0, self.total))
        k = int(np.searchsorted(self.cum, s, side="right") - 1)
        k = min(max(k, 0), len(self.verts) - 2)
        return self.tang[k]

    def halfwidth_at(self, s):
        s = float(np.clip(s, 0, self.total))
        k = int(np.searchsorted(self.cum, s, side="right") - 1)
        k = min(max(k, 0), len(self.verts) - 2)
        t = (s - self.cum[k]) / max(self.seg_len[k], 1e-9)
        return float(self.halfwidth[k] * (1 - t) + self.halfwidth[k + 1] * t)

    def inside_frac(self, pts):
        """返回给定个体中位于路径边界内（含宽度约束）的比例 ρ。
        调用方应传入存活个体（丢失个体视为永久离径，在仿真层计入分母）。"""
        pts = np.asarray(pts, dtype=float)
        if pts.ndim == 1:
            pts = pts[None, :]
        if len(pts) == 0:
            return 0.0
        s, lat, _ = self.closest(pts)
        inside = np.abs(lat) <= np.array([self.halfwidth_at(ss) for ss in s])
        return float(inside.sum()) / len(pts)


# --------------------------------------------------------------------------
# 路径构造辅助
# --------------------------------------------------------------------------
def straight_path(x_start=0.0, x_end=70.0, halfwidth=6.0, n_verts=8):
    xs = np.linspace(x_start, x_end, n_verts)
    return Path(np.stack([xs, np.zeros_like(xs)], axis=1),
                halfwidth=np.full(n_verts, halfwidth))


def bent_path(x_start=0.0, x_end=60.0, bend_at=30.0, bend_angle=np.pi / 2,
              halfwidth=6.0, n_verts=None):
    """直行后在 bend_at 处弯折 bend_angle 的可变宽度路径。"""
    # 第一段沿 +x 到 bend_at，然后转向 bend_angle
    p0 = np.array([x_start, 0.0])
    p1 = np.array([bend_at, 0.0])
    dir2 = np.array([np.cos(bend_angle), np.sin(bend_angle)])
    L2 = 30.0
    p2 = p1 + dir2 * L2
    n1 = 6
    seg = np.linspace(p0, p1, n1)
    if n_verts is None:
        n_verts = 6
    seg2 = np.linspace(p1, p2, n_verts)
    verts = np.vstack([seg[:-1], seg2])
    return Path(verts, halfwidth=np.full(len(verts), halfwidth))
