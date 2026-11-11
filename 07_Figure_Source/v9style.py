"""Shared design system for the v9 figures.

One palette (Okabe-Ito), one type scale, one panel-letter position, and a QA
gate that every figure must pass before it is written.
"""
from __future__ import annotations

import os
import re

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.text import Text

MM = 1.0 / 25.4
# Design width of every body figure.  The manuscript is two-column with a
# 522 pt = 183.6 mm text block, and each figure is included at \textwidth, so
# the figure is drawn at exactly that width: no rescaling, and the type sizes
# below are the sizes that appear on the printed page.
W_MM = 183.6
W = W_MM * MM
W190 = W  # legacy name used by the earlier 190 mm drafting

# ------------------------------------------------------------------ palette
# Okabe-Ito. The proposed method is the strongest ink (pure black); the
# baselines keep one fixed colour each in every figure of the paper.
OURS = "#000000"
METHOD = {
    "Ours_rollout": OURS,
    "B1": "#0072B2",          # blue
    "B2": "#56B4E9",          # sky blue
    "VH-Path": "#009E73",     # bluish green
    "VH-Danger": "#E69F00",   # orange
    "VH-Danger-path": "#CC79A7",  # reddish purple
    "B-Outmost": "#D55E00",   # vermillion
    "B-PCBF": "#F0E442",      # yellow (always drawn with a dark edge)
}
LABEL = {
    "Ours_rollout": "Ours",
    "B1": "B1",
    "B2": "B2",
    "VH-Path": "VH-Path",
    "VH-Danger": "VH-Danger",
    "VH-Danger-path": "VH-Danger-path",
    "B-Outmost": "B-Outmost",
    "B-PCBF": "B-PCBF",
}
METHODS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger",
           "VH-Danger-path", "B-Outmost", "B-PCBF"]
SCENARIOS = ["V2S1", "V2S1N", "V2S2", "V2S2R3", "V2S2R2", "V2S2R1",
             "V2S3", "V2S4S", "V2S4M", "V2S4L"]

# neutral greys for structure, never for a method
INK = "#1A1A1A"
GREY_D = "#4D4D4D"
GREY_M = "#8C8C8C"
GREY_L = "#C8C8C8"
GREY_XL = "#E8E8E8"
ACCENT = "#D55E00"
ALLOWED = {c.lower() for c in METHOD.values()} | {
    INK.lower(), GREY_D.lower(), GREY_M.lower(), GREY_L.lower(),
    GREY_XL.lower(), ACCENT.lower(), "#ffffff", "none",
}

# ------------------------------------------------------------------ type scale
# Body text is 11 pt; figure text is one step smaller, never below 7 pt.
FS_TICK = 7.5
FS_LABEL = 8.0
FS_TEXT = 7.5
FS_PANEL = 8.5
HEAVY = 1.6      # linewidth of the proposed method
NORMAL = 1.0


def setup():
    mpl.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": FS_TEXT,
        "axes.labelsize": FS_LABEL,
        "axes.titlesize": FS_TEXT,
        "xtick.labelsize": FS_TICK,
        "ytick.labelsize": FS_TICK,
        "legend.fontsize": FS_TEXT,
        "axes.linewidth": 0.6,
        "axes.edgecolor": INK,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.minor.width": 0.5,
        "ytick.minor.width": 0.5,
        "xtick.major.size": 2.4,
        "ytick.major.size": 2.4,
        "xtick.minor.size": 1.4,
        "ytick.minor.size": 1.4,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "lines.linewidth": NORMAL,
        "lines.markersize": 3.6,
        "patch.linewidth": 0.6,
        "legend.frameon": False,
        "legend.handlelength": 1.5,
        "legend.handletextpad": 0.5,
        "legend.labelspacing": 0.35,
        "legend.borderaxespad": 0.2,
        "grid.color": GREY_XL,
        "figure.dpi": 150,
        "savefig.dpi": 600,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.formatter.use_locale": False,
    })


def panel(ax, letter, dx=-0.02, dy=1.03):
    """Consistent panel letter: bold, same size, same offset everywhere."""
    ax.text(dx, dy, "(%s)" % letter, transform=ax.transAxes,
            fontsize=FS_PANEL, fontweight="bold", va="bottom", ha="left",
            clip_on=False)


def ygrid(ax):
    ax.set_axisbelow(True)
    ax.yaxis.grid(True, color=GREY_XL, lw=0.5)
    ax.xaxis.grid(False)


def xgrid(ax):
    ax.set_axisbelow(True)
    ax.xaxis.grid(True, color=GREY_XL, lw=0.5)
    ax.yaxis.grid(False)


def thin(ax, keep=("top", "right")):
    for side in keep:
        ax.spines[side].set_visible(False)


# ------------------------------------------------------------------ QA gate
CJK = re.compile(r"[\u3000-\u303f\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff"
                 r"\uff00-\uffef]")


def _iter_texts(fig):
    for t in fig.findobj(Text):
        if not t.get_visible():
            continue
        s = t.get_text()
        if not s or not s.strip():
            continue
        if t.get_fontsize() < 6.99:
            yield t, "font %.2f pt < 7 pt" % t.get_fontsize()
        yield t, None


def qa(fig, name, min_font=7.0, check_overlap=True):
    """Assert canvas width, type floor, palette, no CJK and no text collisions."""
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    fw, fh = fig.canvas.get_width_height()
    width_mm = fig.get_size_inches()[0] * 25.4
    assert abs(width_mm - W_MM) < 0.2, "%s: width %.2f mm" % (name, width_mm)

    problems = []
    boxes = []
    for t in fig.findobj(Text):
        if not t.get_visible():
            continue
        s = t.get_text()
        if not s or not s.strip():
            continue
        fs = t.get_fontsize()
        if fs < min_font - 0.01:
            problems.append("font %.2f pt: %r" % (fs, s[:30]))
        if fs > 9.8:
            problems.append("font %.2f pt above scale: %r" % (fs, s[:30]))
        if CJK.search(s):
            problems.append("CJK text: %r" % s[:30])
        bb = t.get_window_extent(renderer=r)
        if bb.width <= 0 or bb.height <= 0:
            continue
        if bb.x0 < -2 or bb.y0 < -2 or bb.x1 > fw + 2 or bb.y1 > fh + 2:
            problems.append("outside canvas: %r at (%.0f,%.0f)-(%.0f,%.0f)"
                            % (s[:40], bb.x0, bb.y0, bb.x1, bb.y1))
        boxes.append((s, bb))

    if check_overlap:
        for i in range(len(boxes)):
            si, bi = boxes[i]
            for j in range(i + 1, len(boxes)):
                sj, bj = boxes[j]
                ox = min(bi.x1, bj.x1) - max(bi.x0, bj.x0)
                oy = min(bi.y1, bj.y1) - max(bi.y0, bj.y0)
                if ox > 0.8 and oy > 0.8:
                    problems.append("overlap %r / %r (%.1fx%.1f px)"
                                    % (si[:24], sj[:24], ox, oy))

    def _ok(c):
        """Palette membership: Okabe-Ito, or a neutral grey (structure only)."""
        try:
            if isinstance(c, str) and c in ("none", "auto"):
                return True
            rgb = mpl.colors.to_rgb(c)
            if abs(rgb[0] - rgb[1]) < 1e-6 and abs(rgb[1] - rgb[2]) < 1e-6:
                return True
            return mpl.colors.to_hex(rgb, keep_alpha=False).lower() in ALLOWED
        except (ValueError, TypeError):
            return True

    for obj in fig.findobj(mpl.lines.Line2D):
        if not obj.get_visible():
            continue
        c = obj.get_color()
        if not _ok(c):
            problems.append("colour %r on line %r" % (c, obj.get_label()[:20]))
    for obj in fig.findobj(mpl.patches.Patch):
        for c in (obj.get_facecolor(), obj.get_edgecolor()):
            if c is None or len(c) == 0:
                continue
            if not _ok(c):
                problems.append("colour %r on patch" % (mpl.colors.to_hex(c),))

    if problems:
        raise AssertionError("%s failed QA: %r" % (name, problems))
    return dict(width_mm=round(width_mm, 2),
                n_text=len(boxes),
                min_font=min(t.get_fontsize() for t in fig.findobj(Text)
                             if t.get_visible() and t.get_text().strip()))


def save(fig, outdir, name, width_mm=W_MM):
    """QA-gate, then write .pdf (vector) and .png (600 dpi), then log the PNG."""
    assert abs(fig.get_size_inches()[0] * 25.4 - width_mm) < 0.2
    stats = qa(fig, name)
    os.makedirs(outdir, exist_ok=True)
    pdf = os.path.join(outdir, name + ".pdf")
    png = os.path.join(outdir, name + ".png")
    fig.savefig(pdf)
    fig.savefig(png, dpi=600)
    import hashlib
    import json
    with open(png, "rb") as fh:
        digest = hashlib.sha256(fh.read()).hexdigest()[:12]
    stats.update(name=name, png_bytes=os.path.getsize(png), sha256=digest)
    stats["height_mm"] = round(fig.get_size_inches()[1] * 25.4, 2)
    with open(os.path.join(outdir, "qa.jsonl"), "a", encoding="utf-8") as fh:
        fh.write(json.dumps(stats) + "\n")
    plt.close(fig)
    print("%-14s %s  texts=%d  min %.1f pt" %
          (name, "%.2f mm" % stats["width_mm"], stats["n_text"], stats["min_font"]))
    return stats
