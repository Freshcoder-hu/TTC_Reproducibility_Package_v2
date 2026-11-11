#!/usr/bin/env python3
"""Graphical abstract (CEA): scene -> method -> result, one row of three panels.

Drawn physically at 13.0 cm x 6.6 cm, the size the journal reproduces it, so
every font size below is its final size.  All geometry is specified in
millimetres on one axes that spans the whole canvas (1 unit = 1 mm); every
arrow tail and head is computed from the border coordinates of the element it
leaves or enters, so the arrows are exactly horizontal/vertical and snapped.

Panels:
  (a) scene   -- top view of the corridor, the 36-animal flock, the 12 robots,
                 one moving obstacle with a velocity arrow, and the animal-to-
                 obstacle first-contact lines.
  (b) method  -- observed animals + obstacle -> flock-level T(t) -> four-state
                 response -> action-conditioned rollout -> robot action, over
                 the threshold structure of Eq. (2) in s3.tex.
  (c) result  -- contacted-animal fraction under exact observation for the
                 proposed controller and the two guidance baselines, with
                 goal-and-cohesion completion beside each bar.

Numbers (provenance in GRAPHICAL_ABSTRACT_REPORT.md):
  0.0546 / 0.2486 / 0.2444 and 54/60 / 51/60 / 49/60  -> s5.tex:13-15, 44-46
  (T_PRE, T_DIVERT, T_REC, h) = (12, 8, 15, 3) s        -> s3.tex:57-58
  states NORMAL, PRE, DIVERT, RECOVER                   -> s3.tex:25-27, 32-39
  H = 12 s horizon, replanned every 1.0 s               -> s3.tex:123-128
  36 animals, 12 robots                                 -> s3.tex:83

Every mark is drawn here: no bitmap, no stock art, no generated imagery.

Reproducible: SOURCE_DATE_EPOCH is pinned below, so re-running this file
rewrites both outputs byte for byte.
"""
import os

# Pin the PDF creation date before matplotlib is imported (reproducible build).
os.environ.setdefault("SOURCE_DATE_EPOCH", "1793000000")

import argparse
import sys

sys.stdout.reconfigure(encoding="utf-8")

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon, Rectangle

MM = 1.0 / 25.4  # mm -> inch

# ----------------------------------------------------------------- canvas
W_MM, H_MM = 130.0, 66.0

# ----------------------------------------------------------------- palette
# Okabe-Ito plus neutral greys for structure.  Nothing is encoded by hue alone:
# every colour is repeated by a shape, a position or a printed number.
INK = "#1A1A1A"
GREY = "#4D4D4D"
LGREY = "#B0B0B0"
PALE = "#D9D9D9"
BLUE = "#0072B2"
SKY = "#56B4E9"
ORANGE = "#E69F00"
PURPLE = "#CC79A7"

# ----------------------------------------------------------------- type scale
FS_TITLE = 8.0
FS_PANEL = 7.8
FS_CHAIN = 7.2
FS_LABEL = 7.4
FS_SMALL = 7.2
FS_VALUE = 7.8

# ----------------------------------------------------------------- layout
LEFT, RIGHT = 2.2, 127.8
TITLE_Y = 63.8
HEAD_Y = 59.6

PA = (2.2, 38.32)        # panel (a) x range, 30 % of the row
PB = (40.92, 89.08)      # panel (b), 40 %
PC = (91.68, 127.8)      # panel (c), 30 %

CORR = dict(x0=4.9, x1=35.7, y0=5.2, y1=42.6)    # corridor walls
CX = 0.5 * (CORR["x0"] + CORR["x1"])             # centreline
FLOCK = dict(cx=CX, cy=21.0, a=9.8, b=7.6, r=0.76, n=36)
ROBOT = dict(y1=7.0, y2=9.7, r=0.97)
OBST = dict(cx=CX - 5.4, cy=37.0, r=2.15)

# (b) chain: five stops, equal pitch, labels alternate above / below the spine
SPINE_Y = 45.2
PITCH = 8.2
STOP0 = 7.3
LAB_UP = 2.2
LAB_DN = 2.2

# (b) risk strip: T decreasing to the right
STRIP = dict(y0=19.6, y1=25.4)
T_HI, T_LO = 22.0, 2.0

# (c) bars
BAR_X0_LOC = 13.0
BAR_SCALE = 40.0          # mm per unit of contacted fraction (0.30 -> 12.0 mm)
BAR_W = 5.2
ROWS = (43.5, 32.5, 21.5)

FORBIDDEN = ("collision-free", "collision free", "collisionless",
             "guarantee", "guaranteed", "field validated", "field-validated",
             "welfare", "no collisions", "safe in all", "always avoids")


# ===================================================================== drawing
class Sheet:
    """One axes in millimetres, with a text registry for the QA gate."""

    def __init__(self, w_mm, h_mm):
        self.fig = plt.figure(figsize=(w_mm * MM, h_mm * MM), dpi=600)
        self.ax = self.fig.add_axes([0.0, 0.0, 1.0, 1.0])
        self.ax.set_xlim(0.0, w_mm)
        self.ax.set_ylim(0.0, h_mm)
        self.ax.set_aspect("equal")
        self.ax.axis("off")
        self.texts = []          # (artist, panel, kind)

    def text(self, x, y, s, size=FS_SMALL, colour=INK, weight="normal",
             ha="center", va="center", panel=None, kind="free", style="normal",
             zorder=6, **kw):
        t = self.ax.text(x, y, s, fontsize=size, color=colour, fontweight=weight,
                         ha=ha, va=va, family="Arial", fontstyle=style,
                         zorder=zorder, **kw)
        self.texts.append((t, panel, kind))
        return t

    def rect(self, x0, y0, w, h, fc="none", ec=INK, lw=0.8, zorder=2, **kw):
        p = Rectangle((x0, y0), w, h, facecolor=fc, edgecolor=ec, linewidth=lw,
                      zorder=zorder, **kw)
        self.ax.add_patch(p)
        return p

    def line(self, x0, y0, x1, y1, colour=INK, lw=0.8, zorder=3, **kw):
        ln = Line2D([x0, x1], [y0, y1], color=colour, linewidth=lw, zorder=zorder,
                    solid_capstyle="butt", **kw)
        self.ax.add_line(ln)
        return ln

    def disc(self, x, y, r, fc, ec=INK, lw=0.6, zorder=4):
        c = Circle((x, y), r, facecolor=fc, edgecolor=ec, linewidth=lw,
                   zorder=zorder)
        self.ax.add_patch(c)
        return c

    def arrow(self, x0, x1, y, lw=0.9, head=1.6, half=0.8, colour=INK, zorder=5):
        """Horizontal arrow: tail exactly at x0, tip exactly at x1."""
        assert abs(x1 - x0) > head + 0.05
        s = 1.0 if x1 > x0 else -1.0
        hx = x1 - s * head
        self.line(x0, y, hx, y, colour=colour, lw=lw, zorder=zorder)
        self.ax.add_patch(Polygon([[x1, y], [hx, y + half], [hx, y - half]],
                                  closed=True, facecolor=colour,
                                  edgecolor="none", zorder=zorder))
        return x1

    def tbox(self, t):
        """Rendered bbox of a text artist, in mm."""
        r = self.fig.canvas.get_renderer()
        bb = t.get_window_extent(renderer=r)
        inv = self.ax.transData.inverted()
        p0 = inv.transform((bb.x0, bb.y0))
        p1 = inv.transform((bb.x1, bb.y1))
        return min(p0[0], p1[0]), min(p0[1], p1[1]), max(p0[0], p1[0]), max(p0[1], p1[1])


def flock_points(cx, cy, a, b, n, gap=2.30, iters=400):
    """Exactly n animals inside an ellipse, evenly but not mechanically spaced.

    Seeded random start (fixed seed 7, so the result is bit-identical on every
    run), then a separation pass: no two discs touch, and the outline stays
    organic instead of looking like a lattice.
    """
    rng = np.random.RandomState(7)
    u = []
    while len(u) < n:
        p = rng.uniform(-1.0, 1.0, 2)
        if p[0] ** 2 + p[1] ** 2 <= 0.90:
            u.append(p)
    pts = np.array([[cx + a * p[0], cy + b * p[1]] for p in u])
    for _ in range(iters):
        moved = False
        for i in range(n):
            d = pts - pts[i]
            dist = np.hypot(d[:, 0], d[:, 1])
            dist[i] = 1e9
            for j in np.nonzero(dist < gap)[0]:
                step = (gap - dist[j]) / 2.0
                v = d[j] / max(dist[j], 1e-6)
                pts[i] -= step * v
                pts[j] += step * v
                moved = True
        ex = (pts[:, 0] - cx) / a
        ey = (pts[:, 1] - cy) / b
        r = np.hypot(ex, ey)
        over = r > 0.96
        ex[over] *= 0.96 / r[over]
        ey[over] *= 0.96 / r[over]
        pts = np.column_stack([cx + a * ex, cy + b * ey])
        if not moved:
            break
    return pts


def strip_x(x0, x1, T):
    """Data-to-mm map of the risk axis: T_HI at x0, T_LO at x1."""
    return x0 + (T_HI - T) / (T_HI - T_LO) * (x1 - x0)


# ===================================================================== (a)
def panel_scene(s):
    c = CORR

    # corridor: a pale floor between two solid boundaries, dashed centreline
    s.rect(c["x0"], c["y0"], c["x1"] - c["x0"], c["y1"] - c["y0"], fc="#F4F4F4",
           ec="none", zorder=0)
    for wx in (c["x0"], c["x1"]):
        s.line(wx, c["y0"], wx, c["y1"], colour=INK, lw=1.1, zorder=3)
    s.line(CX, c["y0"], CX, c["y1"], colour="#8C8C8C", lw=0.6, zorder=2,
           linestyle=(0, (2.6, 2.0)))

    # flock
    pts = flock_points(FLOCK["cx"], FLOCK["cy"], FLOCK["a"], FLOCK["b"],
                       FLOCK["n"])
    for px, py in pts:
        s.disc(px, py, FLOCK["r"], BLUE, ec="#08405F", lw=0.35, zorder=4)

    # robots: 5 in the front row, 7 behind, both centred on the corridor axis
    for off in (-5.0, -2.5, 0.0, 2.5, 5.0):
        s.disc(CX + off, ROBOT["y2"], ROBOT["r"], "white", ec=INK, lw=0.8,
               zorder=4)
    for off in (-7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5):
        s.disc(CX + off, ROBOT["y1"], ROBOT["r"], "white", ec=INK, lw=0.8,
               zorder=4)

    # obstacle and its velocity arrow (crosses the corridor, leftwards)
    ox, oy, orr = OBST["cx"], OBST["cy"], OBST["r"]
    s.arrow(c["x1"] - 2.4, c["x0"] + 1.6, oy, lw=0.9, head=1.6, half=0.8,
            zorder=3)
    s.disc(ox, oy, orr, PURPLE, ec=INK, lw=0.9, zorder=5)

    # first-contact lines from the four animals closest to the obstacle
    order = np.argsort(pts[:, 1])[::-1][:4]
    for idx in order:
        ax_, ay_ = pts[idx]
        vx, vy = ox - ax_, oy - ay_
        d = np.hypot(vx, vy)
        s.line(ax_, ay_, ox - vx / d * orr, oy - vy / d * orr, colour=GREY,
               lw=0.55, zorder=3, linestyle=(0, (1.6, 1.2)))
    s.text(CX + 3.8, 31.6, "T(t)", size=FS_SMALL, colour=GREY, ha="left",
           style="italic", panel="a")

    # key at the top of the panel
    ky = (54.6, 50.8, 47.0)
    mx, tx = 4.2, 6.4
    s.disc(mx, ky[0], 0.80, BLUE, ec="#08405F", lw=0.35, zorder=6)
    s.text(tx, ky[0], "flock, 36 animals", size=FS_LABEL, ha="left", panel="a")
    s.disc(mx, ky[1], 0.97, "white", ec=INK, lw=0.8, zorder=6)
    s.text(tx, ky[1], "12 robots", size=FS_LABEL, ha="left", panel="a")
    s.disc(mx, ky[2], 1.50, PURPLE, ec=INK, lw=0.9, zorder=6)
    s.text(tx, ky[2], "moving obstacle", size=FS_LABEL, ha="left", panel="a")


# ===================================================================== (b)
def panel_method(s):
    x0, x1 = PB

    # ------------------------------------------------------------- chain
    stops = [x0 + STOP0 + i * PITCH for i in range(5)]
    stages = [
        ("observed", "animals +", "obstacle"),
        ("flock T(t)", "first contact"),
        ("four-state", "response"),
        ("rollout", "H = 12 s", "replan 1.0 s"),
        ("coordinated", "robot action"),
    ]
    r_dot = 0.75
    s.line(stops[0], SPINE_Y, stops[-1], SPINE_Y, colour=INK, lw=0.9, zorder=3)
    for a, b in zip(stops[:-1], stops[1:]):
        s.arrow(a + r_dot + 0.15, b - r_dot - 0.15, SPINE_Y, lw=0.9, head=1.6,
                half=0.8)
    for k, (st, lines) in enumerate(zip(stops, stages)):
        s.disc(st, SPINE_Y, r_dot - 0.1, INK, ec=INK, lw=0.0, zorder=6)
        if k % 2 == 0:
            s.line(st, SPINE_Y + r_dot, st, SPINE_Y + LAB_UP, colour=INK, lw=0.6,
                   zorder=4)
            s.text(st, SPINE_Y + LAB_UP, "\n".join(lines), size=FS_CHAIN,
                   weight="bold", va="bottom", panel="b", linespacing=1.12)
        else:
            s.line(st, SPINE_Y - r_dot, st, SPINE_Y - LAB_DN, colour=INK, lw=0.6,
                   zorder=4)
            s.text(st, SPINE_Y - LAB_DN, "\n".join(lines), size=FS_CHAIN,
                   weight="bold", va="top", panel="b", linespacing=1.12)

    # --------------------------------------------------------- risk strip
    y0, y1 = STRIP["y0"], STRIP["y1"]
    hh = y1 - y0
    bands = [
        (T_HI, 15.0, "NORMAL", PALE, INK, None),
        (15.0, 12.0, "h", "white", GREY, "////"),
        (12.0, 8.0, "PRE", SKY, INK, None),
        (8.0, T_LO, "DIVERT", BLUE, "white", None),
    ]
    spans = {}
    for ta, tb, name, fc, tc, hatch in bands:
        xa, xb = strip_x(x0, x1, ta), strip_x(x0, x1, tb)
        kw = {}
        ec = INK
        if hatch:
            kw = dict(hatch=hatch)
            ec = LGREY
        s.rect(xa, y0, xb - xa, hh, fc=fc, ec=ec, lw=0.7, zorder=2, **kw)
        spans[name] = (xa, xb)
        s.text(0.5 * (xa + xb), 0.5 * (y0 + y1), name, size=FS_SMALL,
               colour=tc, weight="bold", style=("italic" if name == "h" else "normal"),
               panel="b", kind="inband")

    # thresholds: tick above, number above the tick
    for T, lab in ((18.0, "18"), (15.0, "15"), (12.0, "12"), (8.0, "8")):
        xt = strip_x(x0, x1, T)
        s.line(xt, y1, xt, y1 + 1.3, colour=INK, lw=0.7, zorder=5)
        s.text(xt, y1 + 2.35, lab, size=FS_SMALL, weight="bold", panel="b")

    # hysteresis windows, bracketed under the strip: 15->18 and 8->11
    yb = y0 - 1.4
    for Ta, Tb in ((15.0, 18.0), (8.0, 11.0)):
        xa, xb = strip_x(x0, x1, Tb), strip_x(x0, x1, Ta)
        s.line(xa, yb, xb, yb, colour=INK, lw=0.6, zorder=5)
        s.line(xa, yb, xa, yb + 1.0, colour=INK, lw=0.6, zorder=5)
        s.line(xb, yb, xb, yb + 1.0, colour=INK, lw=0.6, zorder=5)
        s.text(0.5 * (xa + xb), yb - 1.5, "h = 3 s", size=FS_SMALL, colour=GREY,
               panel="b")

    # axis caption + footnotes
    s.text(0.5 * (x0 + x1), y1 + 5.1, "first-contact time T(t), s  "
           "(decreasing \u2192)", size=FS_SMALL, panel="b")
    s.text(x0, 13.6, "T_PRE=12  T_DIVERT=8  T_REC=15 s", size=FS_SMALL,
           ha="left", panel="b")
    s.text(x0, 10.0, "8 < 8+h = 11 < 12 < 15 < 15+h = 18 s", size=FS_SMALL,
           ha="left", panel="b")
    s.text(x0, 6.4, "RECOVER after the pass, exit T > 15 s", size=FS_SMALL,
           ha="left", panel="b")


# ===================================================================== (c)
def panel_result(s):
    x0, x1 = PC
    bx = x0 + BAR_X0_LOC

    s.text(x0, 54.8, "animal\u2013obstacle contact", size=FS_PANEL,
           weight="bold", ha="left", panel="c")
    s.text(x0, 51.4, "risk reduced", size=FS_PANEL, weight="bold", ha="left",
           panel="c")
    s.text(x0, 48.4, "exact observation, 60 runs", size=FS_SMALL,
           colour=GREY, ha="left", panel="c")

    rows = [
        (("Proposed", "controller"), 0.0546, "54/60", BLUE),
        (("Path-only", "guidance"), 0.2486, "51/60", ORANGE),
        (("Distance-", "reactive"), 0.2444, "49/60", SKY),
    ]
    s.line(bx, ROWS[-1] - 4.6, bx, ROWS[0] + 3.1, colour=INK, lw=0.8, zorder=3)
    for (names, val, gcv, col), ry in zip(rows, ROWS):
        s.text(bx - 1.5, ry + 1.2, names[0], size=FS_LABEL, ha="right",
               va="center", panel="c")
        s.text(bx - 1.5, ry - 1.55, names[1], size=FS_SMALL, colour=GREY,
               ha="right", va="center", panel="c")
        w = val * BAR_SCALE
        s.rect(bx, ry - BAR_W / 2, w, BAR_W, fc=col, ec=INK, lw=0.7, zorder=3)
        s.text(bx + w + 1.0, ry + 1.2, "%.4f" % val, size=FS_VALUE,
               weight="bold", ha="left", va="center", panel="c")
        s.text(bx + w + 1.0, ry - 1.55, gcv, size=FS_SMALL, colour=GREY,
               ha="left", va="center", panel="c")

    s.text(x0, 13.4, "bars: contacted animal fraction", size=FS_SMALL,
           ha="left", colour=GREY, panel="c")
    s.text(x0, 9.8, "GC: goal-and-cohesion", size=FS_SMALL, ha="left",
           colour=GREY, panel="c")
    s.text(x0, 6.2, "completion over 60 runs", size=FS_SMALL, ha="left",
           colour=GREY, panel="c")


# ===================================================================== build
def build(out_pdf, out_png):
    plt.rcParams.update({
        "font.family": "Arial",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    s = Sheet(W_MM, H_MM)

    s.text(0.5 * (LEFT + RIGHT), TITLE_Y,
           "Animal contact-risk prediction and action-conditioned rollout "
           "for multi-robot shepherding",
           size=FS_TITLE, weight="bold")

    for (a, b), lab in ((PA, "(a) Scene (top view)"), (PB, "(b) Method"), (PC, "(c) Result")):
        s.text(a, HEAD_Y, lab, size=FS_PANEL, weight="bold", ha="left",
               panel=lab[1])

    panel_scene(s)
    panel_method(s)
    panel_result(s)

    # ------------------------------------------------------------- QA gate
    s.fig.canvas.draw()
    problems = []

    w_mm = s.fig.get_size_inches()[0] * 25.4
    h_mm = s.fig.get_size_inches()[1] * 25.4
    if abs(w_mm - W_MM) > 0.02 or abs(h_mm - H_MM) > 0.02:
        problems.append("canvas %.2f x %.2f mm" % (w_mm, h_mm))

    boxes = []
    fonts = []
    for t, panel, kind in s.texts:
        txt = t.get_text()
        if not txt.strip():
            continue
        fs = t.get_fontsize()
        fonts.append((fs, txt))
        if fs < 7.0 - 1e-6:
            problems.append("font %.2f < 7 pt: %r" % (fs, txt[:24]))
        bb = s.tbox(t)
        if bb[0] < LEFT - 0.01 or bb[2] > RIGHT + 0.01 or bb[1] < 0.6 or bb[3] > H_MM - 0.6:
            problems.append("outside canvas: %r %s" % (txt[:26], np.round(bb, 1)))
        if panel in ("a", "b", "c"):
            px0, px1 = {"a": PA, "b": PB, "c": PC}[panel]
            if bb[0] < px0 - 0.01 or bb[2] > px1 + 0.01:
                problems.append("outside panel %s: %r %.2f..%.2f"
                                % (panel, txt[:26], bb[0], bb[2]))
        boxes.append((txt, bb, kind))

    for i in range(len(boxes)):
        ti, bi, _ = boxes[i]
        for j in range(i + 1, len(boxes)):
            tj, bj, _ = boxes[j]
            ox = min(bi[2], bj[2]) - max(bi[0], bj[0])
            oy = min(bi[3], bj[3]) - max(bi[1], bj[1])
            if ox > 0.2 and oy > 0.2:
                problems.append("text overlap %r / %r (%.1f x %.1f mm)"
                                % (ti[:20], tj[:20], ox, oy))

    # 1.5 mm padding inside every filled band that carries text
    x0b, x1b = PB
    spans = {}
    for ta, tb, name in ((T_HI, 15.0, "NORMAL"), (15.0, 12.0, "h"),
                         (12.0, 8.0, "PRE"), (8.0, T_LO, "DIVERT")):
        spans[name] = (strip_x(x0b, x1b, ta), strip_x(x0b, x1b, tb))
    for t, panel, kind in s.texts:
        if kind != "inband":
            continue
        bb = s.tbox(t)
        xa, xb = spans[t.get_text()]
        pad = min(bb[0] - xa, xb - bb[2], bb[1] - STRIP["y0"], STRIP["y1"] - bb[3])
        if pad < 1.5 - 0.02:
            problems.append("padding %.2f mm < 1.5 mm: %r" % (pad, t.get_text()))

    all_text = " ".join(t.get_text().replace("\n", " ") for t, _, _ in s.texts)
    low = all_text.lower()
    for bad in FORBIDDEN:
        if bad in low:
            problems.append("forbidden phrase: %r" % bad)

    if problems:
        for p in problems:
            print("QA  ", p)
        raise SystemExit("QA failed with %d problem(s)" % len(problems))

    os.makedirs(os.path.dirname(os.path.abspath(out_pdf)), exist_ok=True)
    s.fig.savefig(out_pdf, format="pdf")
    s.fig.savefig(out_png, format="png", dpi=600)
    plt.close(s.fig)

    print("canvas     %.2f x %.2f mm" % (w_mm, h_mm))
    print("text items %d" % len(boxes))
    print("min font   %.2f pt" % min(f for f, _ in fonts))
    print("written    %s" % out_pdf)
    print("written    %s" % out_png)


def main():
    import pkgpaths
    pkg = str(pkgpaths.ROOT)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", default=str(pkgpaths.OUT_GA))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    out_pdf = os.path.join(args.out_dir, "graphical_abstract.pdf")
    out_png = os.path.join(args.out_dir, "graphical_abstract.png")
    if args.dry_run:
        print("PKG    ", pkg)
        print("OUT_PDF", out_pdf)
        print("OUT_PNG", out_png)
        return
    build(out_pdf, out_png)


if __name__ == "__main__":
    main()























