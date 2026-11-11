"""Body figures 1-8 of the v9 manuscript, rebuilt from the archived results.

Ported from the submission package's ``06_图表源码/fig_body.py``.  The drawing
code is unchanged; ``pkgpaths.py`` maps the submission-package input tree onto
this repository (every input is byte-identical).  ``fig9()`` remains the
Word-era 100 s-scale panel -- the submitted Fig. 9 is built by ``fig_loss.py``.
"""
from __future__ import annotations

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.path as mpath
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle
from matplotlib.ticker import FormatStrFormatter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v9style as S
from v9style import (ACCENT, FS_LABEL, FS_PANEL, FS_TEXT, FS_TICK, GREY_D,
                     GREY_L, GREY_M, GREY_XL, HEAVY, INK, LABEL, METHOD,
                     METHODS, MM, NORMAL, OURS, SCENARIOS, W190, panel,
                     thin, xgrid, ygrid)

# Paths are mapped onto this repository's layout by pkgpaths.py: the archived
# result tables that the submission package read out of its nested archive
# (TTC_Shepherding_Complete_Reproducibility_Package.zip) are shipped here
# directly, byte-identical, under 01_Main_Experiment/, 02_Perception_
# Robustness/ and 03_Online_Warning_and_Timing/.  P100 (the Word-era 100 s
# scale panel behind the old Fig. 9) is not shipped by either tree; set P100
# to a local copy if that panel has to be regenerated.
import pkgpaths  # noqa: E402

PKG = str(pkgpaths.ROOT)
T3 = str(pkgpaths.CURVED)
P100 = str(pkgpaths.P100)
OUT = str(pkgpaths.OUT_BODY)


def V2(*parts):
    return str(pkgpaths.TABLES.joinpath(*parts))


def S3(*parts):
    return str(pkgpaths.STAGE3.joinpath(*parts))


def ROB(*parts):
    return str(pkgpaths.PERCEPTION.joinpath(*parts))


def ONL(*parts):
    return str(pkgpaths.ONLINE.joinpath(*parts))


# ------------------------------------------------------------------ data
def main9():
    return pd.read_csv(V2("table_main9.csv")).set_index("method")


def per_scene():
    return pd.read_csv(V2("table_per_scenario.csv"))


def ablation():
    return pd.read_csv(V2("table_ablation.csv"))


def contrasts():
    return pd.read_csv(V2("table_contrasts.csv"))


def census():
    return pd.read_csv(V2("table_census.csv")).set_index("method")


def observability():
    obs = pd.read_csv(S3("observability", "observable_table_R70.csv"))
    req = pd.read_csv(S3("required_range.csv"))
    return obs, req


def robust_summary():
    return pd.read_csv(ROB("summary.csv"))


def timing():
    return pd.read_csv(ONL("timing_summary.csv"))


def three_turn():
    return pd.read_csv(os.path.join(T3, "detail.csv"))


# ------------------------------------------------------------------ figure 1
def fig1():
    obs, req = observability()
    fig, axes = plt.subplots(1, 2, figsize=(W190, 66 * MM))
    fig.subplots_adjust(left=0.085, right=0.975, bottom=0.155, top=0.92,
                        wspace=0.30)

    ax = axes[0]
    sp = np.sort(obs.speed.unique())
    head = obs[obs.geometry == "head_on"].set_index("speed").loc[sp]
    obl = obs[obs.geometry == "oblique_3"].set_index("speed").loc[sp]
    fam = ["cross_0", "cross_3", "cross_6"]
    perp_mean = np.mean([obs[obs.geometry == g].set_index("speed").loc[
        sp, "T_obs_mean"].to_numpy() for g in fam], axis=0)
    perp_p10 = np.mean([obs[obs.geometry == g].set_index("speed").loc[
        sp, "T_obs_p10"].to_numpy() for g in fam], axis=0)

    series = [(head["T_obs_mean"].to_numpy(), head["T_obs_p10"].to_numpy(),
               METHOD["B1"], "head-on"),
              (obl["T_obs_mean"].to_numpy(), obl["T_obs_p10"].to_numpy(),
               METHOD["VH-Path"], "oblique (S2 direction)"),
              (perp_mean, perp_p10, GREY_D, "perpendicular, three offsets")]
    for mean, p10, col, lab in series:
        ax.plot(sp, mean, color=col, lw=NORMAL, marker="o", ms=3.6, mfc=col,
                mec=col, zorder=3)
        ax.plot(sp, p10, color=col, lw=0.7, ls=(0, (1.2, 1.4)), zorder=2)

    ax.axhline(12.0, color=INK, lw=0.7, ls=(0, (4, 2)), zorder=1)
    ax.axhline(16.0, color=GREY_M, lw=0.7, ls=(0, (1, 1.6)), zorder=1)
    ax.text(3.08, 26.3, "horizontal rules:\n12 s frozen, 16 s alternative",
            fontsize=FS_TEXT, ha="left", va="top", linespacing=1.35)

    handles = [plt.Line2D([], [], color=c, lw=NORMAL, marker="o", ms=3.6,
                          label=l) for _, _, c, l in series]
    handles.append(plt.Line2D([], [], color=GREY_D, lw=0.7,
                              ls=(0, (1.2, 1.4)),
                              label="10th percentile (dotted)"))
    ax.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.0, 0.0))

    ax.set_xlim(2.9, 6.15)
    ax.set_ylim(3.5, 26.5)
    ax.set_xticks([3, 4, 5, 6])
    ax.set_yticks([4, 8, 12, 16, 20, 24])
    ax.set_xlabel("obstacle speed [m/s]")
    ax.set_ylabel("observable warning time [s]")
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    thin(ax)
    panel(ax, "a", dx=-0.115)

    ax = axes[1]
    d = req[req.geometry == "head_on"].set_index("speed").loc[sp]
    specs = [("R_required_T_DIVERT", "8 s", GREY_D, (0, (1, 1.6))),
             ("R_required_T_PRE_frozen", "12 s", METHOD["B1"], "-"),
             ("R_required_T_PRE_16", "16 s", ACCENT, (0, (4, 2)))]
    for col, lab, cc, ls in specs:
        ax.plot(sp, d[col], color=cc, lw=NORMAL, ls=ls, marker="o", ms=3.6,
                mfc=cc, mec=cc, zorder=3)
    ax.axhline(70.0, color=INK, lw=0.7)
    ax.text(6.34, 71.6, r"configured $R$ = 70 m", fontsize=FS_TEXT,
            ha="right", va="bottom")
    for col, lab, cc, ls in specs:
        ax.text(6.07, d[col].iloc[-1], lab, color=cc, fontsize=FS_TEXT,
                ha="left", va="center")
    ax.set_xlim(2.9, 6.4)
    ax.set_ylim(35, 128)
    ax.set_xticks([3, 4, 5, 6])
    ax.set_yticks([40, 60, 80, 100, 120])
    ax.set_xlabel("obstacle speed [m/s]")
    ax.set_ylabel("range needed [m]")
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    thin(ax)
    panel(ax, "b", dx=-0.115)
    S.save(fig, OUT, "Fig1")


# ------------------------------------------------------------------ figure 2
def fig2():
    """Architecture of the coupling.

    Strict orthogonal geometry: one three-column grid, identical box heights,
    arrows that are exactly horizontal or vertical, every endpoint snapped to
    a box border, one arrowhead size and one line width for all connectors.
    """
    W_MM, H_MM = 183.6, 99.0
    fig, ax = plt.subplots(figsize=(W_MM * S.MM, H_MM * S.MM))
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax.set_xlim(0, W_MM)
    ax.set_ylim(0, H_MM)
    ax.set_axis_off()

    PAD = 3.0          # inner padding, mm
    HEAD = 9.0         # header band height, mm
    XS = [2.8, 65.8, 128.8]   # left edge of each column, mm
    BW = 52.0                 # identical box width, mm
    ROW1_Y, ROW1_H = 56.0, 32.0
    ROW2_Y, ROW2_H = 8.0, 28.0
    MID = ROW1_Y + ROW1_H / 2.0        # 72.0 mm, the only y used by the
    TOP = ROW1_Y + ROW1_H              # horizontal connectors
    boxes = []

    def box(x, y, w, h, header, lines, ec=GREY_D, fc="white", lw=0.7):
        ax.add_patch(Rectangle((x, y), w, h, facecolor=fc, edgecolor=ec,
                               lw=lw, zorder=2, joinstyle="miter"))
        hdr = ax.text(x + w / 2, y + h - HEAD / 2, header, fontsize=FS_LABEL,
                      ha="center", va="center", zorder=4)
        ax.plot([x + PAD, x + w - PAD], [y + h - HEAD] * 2, color=ec, lw=0.4,
                zorder=3)
        body = ax.text(x + w / 2, y + (h - HEAD) / 2, "\n".join(lines),
                       fontsize=FS_TEXT, ha="center", va="center",
                       linespacing=1.55, zorder=4)
        boxes.append((x, y, w, h, hdr, body))

    box(XS[0], ROW1_Y, BW, ROW1_H, "flock-level contact risk",
        ["observed animals and obstacles",
         r"$\tau_{io}$: first-contact time of $(i,o)$",
         r"$T(t)=\min_{i,o}\,\tau_{io}$"])
    box(XS[1], ROW1_Y, BW, ROW1_H, "four-state response",
        ["normal guidance, pre-positioning,",
         "diversion, preemptible recovery;",
         "handover of the avoidance slot"], ec=METHOD["B1"])
    box(XS[2], ROW1_Y, BW, ROW1_H, "action-conditioned rollout",
        [r"candidates $\delta=0,\pm1.35,\pm2.4,\pm3.6$ m",
         r"prediction horizon $H=12$ s",
         r"$J(\delta)$: predicted contact,",
         "path, spread, switching"], ec=ACCENT)
    box(XS[0], ROW2_Y, XS[2] + BW - XS[0], ROW2_H, "shared execution layer",
        [r"finite bodies $r=0.45$ m;  $v_{\max}=4.5$ m/s;  "
         r"$a_{\mathrm{lin}}=4.0$, $a_{\mathrm{lat}}=2.0$ m s$^{-2}$",
         r"layered speed policy $v\leq\sqrt{a_{\mathrm{lat}}r_{\mathrm{req}}}$ "
         "in pre-positioning and diversion"], fc=GREY_XL[:7])

    def arrow(p, q, text="", tpos=None, color=INK, ls="-", ha="center",
              va="bottom"):
        """Exactly horizontal or vertical arrow between two named points."""
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=7,
                                     lw=0.75, color=color, linestyle=ls,
                                     shrinkA=0, shrinkB=0, capstyle="butt",
                                     joinstyle="miter", zorder=5))
        if text:
            ax.text(tpos[0], tpos[1], text, fontsize=FS_TEXT, ha=ha, va=va,
                    zorder=6)

    def elbow(points, text, tpos):
        """Dashed feedback path built from right angles only."""
        path = mpath.Path(points, [mpath.Path.MOVETO]
                          + [mpath.Path.LINETO] * (len(points) - 1))
        ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>",
                                     mutation_scale=7, lw=0.75, color=INK,
                                     linestyle=(0, (3, 2)), shrinkA=0,
                                     shrinkB=0, capstyle="butt",
                                     joinstyle="miter", zorder=5))
        ax.text(tpos[0], tpos[1], text, fontsize=FS_TEXT, ha="center",
                va="bottom", zorder=6)

    # row connectors: horizontal, at the row centre, border to border
    arrow((XS[0] + BW, MID), (XS[1], MID), "when", (60.3, MID + 1.1))
    arrow((XS[1] + BW, MID), (XS[2], MID), "offset", (123.3, MID + 1.1))
    # rollout -> execution: a single vertical segment, border to border
    arrow((XS[2] + BW / 2, ROW1_Y), (XS[2] + BW / 2, ROW2_Y + ROW2_H),
          "selected offset", (XS[2] + BW / 2 + 2.2, 46.0), ha="left",
          va="center")
    # execution -> risk: dashed feedback with right angles only
    elbow([(60.0, ROW2_Y + ROW2_H), (60.0, 46.0), (20.0, 46.0),
           (20.0, ROW1_Y)], "executed motion", (40.0, 47.1))
    # annotation over the top row: horizontal, from the centre of the first
    # box to the centre of the last
    arrow((XS[0] + BW / 2, 92.0), (XS[2] + BW / 2, 92.0),
          "the flock-level risk of every candidate is predicted: "
          "selection is by consequence", (91.8, 93.2), color=ACCENT)

    # geometry gate: every text inside its own box with 1.5 mm to spare
    fig.canvas.draw()
    inv = ax.transData.inverted()
    for (x, y, w, h, hdr, body) in boxes:
        for t in (hdr, body):
            bb = t.get_window_extent(renderer=fig.canvas.get_renderer())
            (x0, y0) = inv.transform((bb.x0, bb.y0))
            (x1, y1) = inv.transform((bb.x1, bb.y1))
            assert x0 >= x + 1.5 and x1 <= x + w - 1.5, \
                "fig2: text wider than its box: %r" % t.get_text()[:40]
            assert y0 >= y + 1.5 and y1 <= y + h - 1.5, \
                "fig2: text taller than its box: %r" % t.get_text()[:40]
    S.save(fig, OUT, "Fig2")


# ------------------------------------------------------------------ figure 3
def fig3():
    fig = plt.figure(figsize=(W190, 96 * MM))
    gs = fig.add_gridspec(2, 6, left=0.055, right=0.99, bottom=0.11,
                          top=0.90, hspace=0.62, wspace=0.55)
    axa = fig.add_subplot(gs[0, 0:2])
    axb = fig.add_subplot(gs[0, 2:4])
    axc = fig.add_subplot(gs[0, 4:6])
    axd = fig.add_subplot(gs[1, 0:3])
    axe = fig.add_subplot(gs[1, 3:6])

    def corridor(ax, L, half=8.0, label=True):
        ax.add_patch(Rectangle((0, -half), L, 2 * half, facecolor=GREY_XL[:7],
                               edgecolor="none", zorder=0))
        ax.plot([0, L], [half, half], color=INK, lw=0.6, zorder=2)
        ax.plot([0, L], [-half, -half], color=INK, lw=0.6, zorder=2)
        ax.plot([0, L], [0, 0], color=GREY_M, lw=0.5, ls=(0, (3, 3)), zorder=2)
        ax.set_ylim(-9.6, 9.6)
        ax.set_yticks([-8, 0, 8])
        ax.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))
        ax.set_xlim(-6, L + 5)
        ax.set_xticks(np.arange(0, L + 1, 40))
        if label:
            ax.set_xlabel("arc length $s$ [m]")

    def flock(ax, s=8.0, n=36, seed=5):
        rng = np.random.default_rng(seed)
        xs = s + rng.normal(0, 2.5, n)
        ys = np.clip(rng.normal(0, 2.2, n), -4.4, 4.4)
        ax.scatter(xs, ys, s=2.2, c=INK, marker="o", lw=0, zorder=3)

    def obstacle(ax, x, y, u, tag="", dx=0.0, dy=0.0, ha="left"):
        ax.add_patch(plt.Circle((x, y), 1.2, facecolor=ACCENT, alpha=0.85,
                                edgecolor="none", zorder=4))
        ux, uy = u / np.linalg.norm(u)
        ax.annotate("", xy=(x + ux * 8, y + uy * 8), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", lw=0.8, color=ACCENT,
                                    shrinkA=5, shrinkB=1.5, mutation_scale=6),
                    zorder=5)
        if tag:
            ax.text(x + dx, y + dy, tag, fontsize=FS_TEXT, color=ACCENT,
                    ha=ha, va="center", zorder=6)

    uL = np.array([-1.0, -0.45])
    uR = np.array([-1.0, 0.45])

    ax = axa
    corridor(ax, 140)
    flock(ax, 8)
    ax.plot([140 * 0.92], [0], marker="*", ms=7, color=METHOD["B1"], zorder=4)
    ax.text(140 * 0.92 - 4, 2.2, "goal", fontsize=FS_TEXT, ha="right")
    ax.set_ylabel("lateral [m]")
    ax.set_title("V2S1", fontsize=FS_TEXT, pad=3)
    panel(ax, "a", dx=-0.16)
    thin(ax)

    ax = axb
    L, bend, ang = 140.0, 60.0, np.deg2rad(75.0)
    P1 = np.array([bend, 0.0])
    P2 = np.array([bend + (L - bend) * np.cos(ang), (L - bend) * np.sin(ang)])
    P0 = np.array([0.0, 0.0])
    d0 = np.array([1.0, 0.0])
    d1 = (P2 - P1) / np.linalg.norm(P2 - P1)
    R = 14.0
    t = R * np.tan(ang / 2)
    A = P1 - t * d0
    B = P1 + t * d1
    bis = d1 - d0
    bis = bis / np.linalg.norm(bis)
    C = P1 + bis * (R / np.sin((np.pi - ang) / 2))
    a0 = np.arctan2(*(A - C)[::-1])
    a1 = np.arctan2(*(B - C)[::-1])
    if a1 < a0:
        a1 += 2 * np.pi
    th = np.linspace(a0, a1, 60)
    arc = np.c_[C[0] + R * np.cos(th), C[1] + R * np.sin(th)]
    path = np.vstack([P0, A, arc, B, P2])
    tang = np.gradient(path, axis=0)
    tang = tang / np.hypot(tang[:, 0], tang[:, 1])[:, None]
    nvec = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    upper = path + 8.0 * nvec
    lower = path - 8.0 * nvec
    ax.add_patch(Polygon(np.vstack([upper, lower[::-1]]), closed=True,
                         facecolor=GREY_XL[:7], edgecolor=INK, lw=0.6,
                         zorder=0))
    ax.plot(path[:, 0], path[:, 1], color=GREY_M, lw=0.5, ls=(0, (3, 3)),
            zorder=2)
    ax.set_xlim(-8, L + 8)
    ax.set_ylim(-14, 86)
    ax.set_yticks([0, 20, 40, 60, 80])
    ax.set_xticks([0, 50, 100, 140])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("V2S1N (75° bend)", fontsize=FS_TEXT, pad=3)
    flock(ax, 8)
    panel(ax, "b", dx=-0.16)
    thin(ax)

    ax = axc
    corridor(ax, 140)
    flock(ax, 8)
    obstacle(ax, 70, 5.0, uL)
    ax.text(74.5, 7.8, "3 / 4 / 5 / 6 m/s", fontsize=FS_TEXT, ha="left",
            va="top", color=ACCENT)
    ax.text(70, -5.4, "encounter at 70 m", fontsize=FS_TEXT, ha="center",
            va="top")
    ax.set_title("V2S2 family", fontsize=FS_TEXT, pad=3)
    panel(ax, "c", dx=-0.16)
    thin(ax)

    ax = axd
    corridor(ax, 220)
    flock(ax, 8)
    obstacle(ax, 70, 5.0, uL, "1: 3 m/s", dx=0.6, dy=2.0)
    obstacle(ax, 83, -5.0, uR, "2: 6 m/s", dx=0.6, dy=-2.0)
    obstacle(ax, 145, 5.0, uL, "3: 3 m/s", dx=0.6, dy=2.0)
    ax.set_title("V2S3 (sequential, 220 m)", fontsize=FS_TEXT, pad=3)
    panel(ax, "d", dx=-0.09)
    thin(ax)

    ax = axe
    ax.set_xlim(-4, 150)
    ax.set_ylim(-10.6, 10.6)
    ax.set_xticks([0, 40, 80, 120])
    ax.set_yticks([])
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    for i, (ns, nr, y0) in enumerate([(18, 6, 6.6), (36, 12, 0.0),
                                      (60, 20, -6.6)]):
        ax.add_patch(Rectangle((0, y0 - 2.3), 140, 4.6,
                               facecolor=GREY_XL[:7], edgecolor=INK, lw=0.5,
                               zorder=0))
        ax.plot([0, 140], [y0, y0], color=GREY_M, lw=0.4, ls=(0, (3, 3)))
        rng = np.random.default_rng(20 + i)
        ax.scatter(8 + rng.normal(0, 2.0, ns), y0 + rng.normal(0, 0.85, ns),
                   s=1.8, c=INK, lw=0, zorder=3)
        ax.text(24, y0 + 1.2, "%d:%d" % (ns, nr), fontsize=FS_TEXT,
                va="center", ha="left")
        ax.add_patch(plt.Circle((70, y0 + 1.3), 0.85, facecolor=ACCENT,
                                edgecolor="none", zorder=4))
        ax.annotate("", xy=(65.0, y0 + 0.2), xytext=(70, y0 + 1.3),
                    arrowprops=dict(arrowstyle="-|>", lw=0.7, color=ACCENT,
                                    shrinkA=2, shrinkB=1, mutation_scale=5))
    ax.set_xlabel("arc length $s$ [m]")
    ax.set_title("V2S4 scale family (same corridor)", fontsize=FS_TEXT, pad=3)
    panel(ax, "e", dx=-0.09)
    S.save(fig, OUT, "Fig3")


# ------------------------------------------------------------------ figure 4
def fig4():
    m = main9().loc[METHODS]
    fig, axes = plt.subplots(1, 2, figsize=(W190, 74 * MM))
    fig.subplots_adjust(left=0.135, right=0.975, bottom=0.16, top=0.93,
                        wspace=0.16)
    y = np.arange(len(METHODS))[::-1]

    ax = axes[0]
    for yi, k in zip(y, METHODS):
        ec = GREY_D if k == "B-PCBF" else "none"
        ax.barh(yi, m.loc[k, "C"], height=0.62, color=METHOD[k],
                edgecolor=ec, lw=0.5, zorder=3)
    for yi, k in zip(y, METHODS):
        v = m.loc[k, "C"]
        ax.text(v + 0.004, yi, "%.3f" % v, va="center", fontsize=FS_TEXT)
    ax.set_yticks(y)
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    ax.set_xlim(0, 0.225)
    ax.set_xticks([0, 0.05, 0.10, 0.15, 0.20])
    ax.xaxis.set_major_formatter(FormatStrFormatter("%.2f"))
    ax.set_xlabel("animal–obstacle contact fraction $C$")
    xgrid(ax)
    thin(ax)
    panel(ax, "a", dx=-0.10)

    ax = axes[1]
    xmax = 2.4
    for yi, k in zip(y, METHODS):
        v = float(m.loc[k, "clr_so"])
        cc = METHOD[k]
        ax.plot([0, min(v, xmax)], [yi, yi], color=cc, lw=2.0,
                solid_capstyle="butt", zorder=3)
        if v > xmax:
            ax.plot([xmax], [yi], marker=">", ms=4.6, color=cc, zorder=4,
                    clip_on=False)
            ax.text(xmax + 0.10, yi, "%+.2f" % v, va="center", ha="left",
                    fontsize=FS_TEXT)
        else:
            ax.plot([v], [yi], marker="o", ms=4.0, color=cc, zorder=4)
            ax.text(v + (0.06 if v >= 0 else -0.06), yi, "%+.2f" % v,
                    va="center", ha="left" if v >= 0 else "right",
                    fontsize=FS_TEXT)
    ax.axvline(0, color=INK, lw=0.7, zorder=2)
    ax.set_yticks([])
    ax.set_xlim(-1.45, 2.9)
    ax.set_xticks([-1, 0, 1, 2])
    ax.xaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    ax.set_xlabel("worst-case animal–obstacle clearance [m]")
    xgrid(ax)
    thin(ax)
    panel(ax, "b", dx=-0.10)
    S.save(fig, OUT, "Fig4")


# ------------------------------------------------------------------ figure 5
def fig5():
    p = per_scene()
    mat = p.pivot(index="method", columns="scenario", values="C").loc[
        METHODS, SCENARIOS]
    fig, ax = plt.subplots(figsize=(W190, 66 * MM))
    fig.subplots_adjust(left=0.115, right=0.90, bottom=0.25, top=0.96)
    im = ax.imshow(mat.values, aspect="auto", cmap="Blues", vmin=0,
                   vmax=float(np.nanmax(mat.values)))
    ax.set_xticks(range(len(SCENARIOS)))
    ax.set_xticklabels(SCENARIOS, rotation=35, ha="right",
                       rotation_mode="anchor")
    ax.set_yticks(range(len(METHODS)))
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            v = mat.values[i, j]
            ax.text(j, i, "%.3f" % v, ha="center", va="center",
                    fontsize=FS_TEXT,
                    color="white" if v > 0.55 * np.nanmax(mat.values) else INK)
    ax.axvline(1.5, color=INK, lw=0.8, zorder=5)
    ax.set_xticks(np.arange(-0.5, len(SCENARIOS), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(METHODS), 1), minor=True)
    ax.grid(which="minor", color="white", lw=0.9)
    ax.tick_params(which="minor", length=0)
    ax.tick_params(which="major", length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    cb = fig.colorbar(im, ax=ax, fraction=0.022, pad=0.012,
                      ticks=[0, 0.1, 0.2, 0.3])
    cb.set_label("contact fraction $C$", fontsize=FS_LABEL)
    cb.ax.tick_params(labelsize=FS_TICK, width=0.6)
    cb.ax.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
    cb.outline.set_linewidth(0.5)
    S.save(fig, OUT, "Fig5")


# ------------------------------------------------------------------ figure 6
def fig6():
    a = ablation()
    rungs = ["R0_four_state", "R1_handover", "R2_event", "R3_rollout"]
    labels = ["R0\nfour-state", "R1\nhandover", "R2\nevent", "R3\nrollout"]
    fig, axes = plt.subplots(1, 2, figsize=(W190, 66 * MM))
    fig.subplots_adjust(left=0.095, right=0.985, bottom=0.20, top=0.92,
                        wspace=0.30)
    for ax, col, lab, letter in [
            (axes[0], "C", "contact fraction $C$", "a"),
            (axes[1], "clr_so", "minimum surface clearance [m]", "b")]:
        for s, cc, name in [("V2S2R2", METHOD["B1"], "V2S2R2"),
                            ("V2S3", OURS, "V2S3")]:
            d = a[a.scenario == s].set_index("rung")
            ys = [d.loc[r, col] for r in rungs]
            lw = HEAVY if s == "V2S3" else NORMAL
            ax.plot(range(4), ys, color=cc, lw=lw, marker="o", ms=3.8,
                    mfc=cc, mec=cc, zorder=4)
            ax.scatter([3.24], [d.loc["R3_no_speed_policy", col]],
                       marker="o", s=22, facecolor="white", edgecolor=cc,
                       lw=0.9, zorder=5, clip_on=False)
            ax.text(-0.16, ys[0], name, color=cc, fontsize=FS_TEXT,
                    va="center", ha="right")
        ax.set_xticks(range(4))
        ax.set_xticklabels(labels, fontsize=FS_TEXT)
        ax.set_ylabel(lab)
        ax.set_xlim(-0.95, 3.45)
        thin(ax)
        panel(ax, letter, dx=-0.24)
    axes[0].set_ylim(-0.02, 0.40)
    axes[0].set_yticks([0, 0.1, 0.2, 0.3, 0.4])
    axes[0].yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
    ygrid(axes[0])
    axes[1].set_ylim(-1.75, 0.98)
    axes[1].set_yticks([-1.5, -1.0, -0.5, 0, 0.5])
    axes[1].yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
    ygrid(axes[1])
    axes[1].axhline(0, color=INK, lw=0.6, ls=(0, (3, 2)), zorder=2)
    S.save(fig, OUT, "Fig6")


# ------------------------------------------------------------------ figure 7
def fig7():
    d = robust_summary()
    conds = ["exact", "obs_pos_0p2", "obs_pos_0p5", "animal_pos_0p2",
             "animal_pos_0p5", "delay_0p2", "delay_0p5", "loss_0p05",
             "loss_0p10", "blackout_0p5", "blackout_1p0"]
    clab = ["exact", "obstacle σ 0.2 m", "obstacle σ 0.5 m",
            "animal σ 0.2 m", "animal σ 0.5 m", "delay 0.2 s", "delay 0.5 s",
            "loss 5 %", "loss 10 %", "interrupt 0.5 s", "interrupt 1.0 s"]
    scenes = ["V2S3", "V2S2R2"]
    methods = ["Ours_rollout", "B1", "B2"]
    fig, axes = plt.subplots(2, 3, figsize=(W190, 104 * MM), sharey=True)
    fig.subplots_adjust(left=0.185, right=0.985, bottom=0.10, top=0.90,
                        wspace=0.16, hspace=0.42)
    y = np.arange(len(conds))[::-1]

    titles = ["contact fraction $C$", "joint completion",
              "runs with robot–animal contact"]
    xlims = [(-0.02, 0.62), (0.38, 1.02), (-0.02, 1.02)]
    ticks = [[0.0, 0.2, 0.4, 0.6], [0.4, 0.6, 0.8, 1.0],
             [0.0, 0.25, 0.5, 0.75, 1.0]]
    for row, scene in enumerate(scenes):
        sub = d[(d.scenario == scene) & (d.version == "safe")].set_index(
            ["factor", "method"])
        for col in range(3):
            ax = axes[row, col]
            for yi, cf in zip(y, conds):
                ax.axhline(yi, color=GREY_XL, lw=0.5, zorder=0)
                for k, off in zip(methods, (0.16, 0.0, -0.16)):
                    r = sub.loc[(cf, k)]
                    if col == 0:
                        v = r["C"]
                    elif col == 1:
                        v = r["joint"] / 30.0
                    else:
                        v = r["rs_contact_runs"] / 30.0
                    ax.plot([v], [yi + off], marker="o",
                            ms=3.6 if k == "Ours_rollout" else 3.2,
                            color=METHOD[k], mfc=METHOD[k], mec=METHOD[k],
                            lw=0, zorder=3)
            ax.set_xlim(*xlims[col])
            ax.set_xticks(ticks[col])
            ax.xaxis.set_major_formatter(FormatStrFormatter("%.2f"))
            thin(ax)
            if row == 0:
                ax.set_title(titles[col], fontsize=FS_TEXT, pad=4)
            if col == 0:
                ax.set_yticks(y)
                ax.set_yticklabels(clab)
                ax.set_ylabel(scene, fontsize=FS_LABEL, labelpad=6)
            if row == 1:
                ax.set_xlabel(["$C$", "joint fraction", "contact fraction"][col])
            if row == 0 and col == 2:
                hs = [plt.Line2D([], [], marker="o", lw=0, color=METHOD[k],
                                 label=LABEL[k], ms=3.2)
                      for k in methods]
                ax.legend(handles=hs, loc="lower right", bbox_to_anchor=(1.02, -0.02))
    for row in range(2):
        for col in range(3):
            panel(axes[row, col], "abcdef"[row * 3 + col], dx=-0.02)
    S.save(fig, OUT, "Fig7")


def confirm_verts():
    """Centreline of the 220 m three-turn corridor (45 vertices, 5 m spacing).

    The release ships geometry_confirm_verts.npy; if it is absent the polyline
    is rebuilt from the turn plan documented in geometry_check.json, which
    reproduces the stored vertices exactly (last vertex 189.582, 26.665;
    total arc length 219.847 m).
    """
    fn = os.path.join(T3, "geometry_confirm_verts.npy")
    if os.path.exists(fn):
        return np.load(fn)
    length, ds = 220.0, 5.0
    plan = [(45.0, 85.0, 50.0), (101.0, 147.0, -95.0), (160.0, 204.0, 80.0)]

    def heading(sv):
        th = 0.0
        for lo, hi, delta in plan:
            wgt = float(np.clip((sv - lo) / (hi - lo), 0.0, 1.0))
            th += np.deg2rad(delta) * wgt * wgt * (3.0 - 2.0 * wgt)
        return th

    fine = np.arange(0.0, length + 0.125, 0.25)
    pts = np.zeros((len(fine), 2))
    for i in range(1, len(fine)):
        h = heading(0.5 * (fine[i] + fine[i - 1]))
        pts[i] = pts[i - 1] + (fine[i] - fine[i - 1]) * np.array(
            [np.cos(h), np.sin(h)])
    grid = np.arange(0.0, length + 1e-9, ds)
    return np.column_stack([np.interp(grid, fine, pts[:, 0]),
                            np.interp(grid, fine, pts[:, 1])])


# ------------------------------------------------------------------ figure 8
def fig8():
    det = three_turn()
    verts = confirm_verts()
    z = np.load(os.path.join(T3, "traces",
                             "V2S5B__Ours_rollout__seed5000.npz"))
    fc = z["sheep_positions"].mean(axis=1)
    rc = z["robot_positions"].mean(axis=1)
    ob = z["obstacle_positions"][:, 0, :]

    fig = plt.figure(figsize=(W190, 84 * MM))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], left=0.075,
                          right=0.985, bottom=0.14, top=0.93, wspace=0.30)
    ax = fig.add_subplot(gs[0])
    tang = np.gradient(verts, axis=0)
    tang = tang / np.hypot(tang[:, 0], tang[:, 1])[:, None]
    nvec = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    upper = verts + 8.0 * nvec
    lower = verts - 8.0 * nvec
    ax.add_patch(Polygon(np.vstack([upper, lower[::-1]]), closed=True,
                         facecolor=GREY_XL[:7], edgecolor="none", zorder=0))
    ax.plot(upper[:, 0], upper[:, 1], color=INK, lw=0.5, zorder=2)
    ax.plot(lower[:, 0], lower[:, 1], color=INK, lw=0.5, zorder=2)
    ax.plot(verts[:, 0], verts[:, 1], color=GREY_M, lw=0.6, ls=(0, (3, 3)),
            zorder=2)
    ax.plot(fc[:, 0], fc[:, 1], color=OURS, lw=1.3, zorder=6)
    ax.plot(rc[:, 0], rc[:, 1], color=GREY_D, lw=0.7, ls=(0, (4, 2)), zorder=5)
    keep = ob[:, 1] < 75
    ax.plot(ob[keep, 0], ob[keep, 1], color=ACCENT, lw=0.9, zorder=4)
    ax.annotate("", xy=(ob[keep][-1, 0], ob[keep][-1, 1]),
                xytext=(ob[keep][-6, 0], ob[keep][-6, 1]),
                arrowprops=dict(arrowstyle="-|>", lw=0.8, color=ACCENT,
                                mutation_scale=6), zorder=4)
    ax.plot([fc[0, 0]], [fc[0, 1]], marker="o", ms=3.2, color=OURS, zorder=7)
    ax.plot([fc[-1, 0]], [fc[-1, 1]], marker="o", ms=3.2, color=OURS, zorder=7)
    ax.plot([104.2], [44.6], marker="o", ms=4.2, mfc="white", mec=ACCENT,
            mew=1.0, color=ACCENT, zorder=8)
    ax.annotate("encounter $t$ = 124.0 s", xy=(104.2, 44.6),
                xytext=(112, 62), fontsize=FS_TEXT, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", lw=0.6, color=ACCENT,
                                shrinkA=2, shrinkB=4))
    ax.text(fc[-1, 0] - 4, fc[-1, 1] + 5, "end", fontsize=FS_TEXT,
            ha="right", va="bottom")
    ax.text(fc[0, 0] + 3.0, fc[0, 1] + 4.5, "start", fontsize=FS_TEXT,
            ha="left", va="bottom")
    ax.set_xlim(0, 194)
    ax.set_ylim(-15, 70)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xticks([0, 50, 100, 150])
    ax.set_yticks([0, 20, 40, 60])
    ax.xaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))
    thin(ax)
    panel(ax, "a", dx=-0.10)
    hs = [plt.Line2D([], [], color=OURS, lw=1.3, label="flock centroid"),
          plt.Line2D([], [], color=GREY_D, lw=0.7, ls=(0, (4, 2)),
                     label="robot centroid"),
          plt.Line2D([], [], color=ACCENT, lw=0.9, label="obstacle path")]
    ax.legend(handles=hs, loc="upper left", bbox_to_anchor=(0.0, 1.02))

    ax = fig.add_subplot(gs[1])
    methods = ["Ours_rollout", "B1", "B2", "VH-Path"]
    rng = np.random.default_rng(11)
    for i, k in enumerate(methods):
        v = det[(det.scenario == "V2S5B") & (det.method == k)][
            "C_sampled"].to_numpy()
        jit = rng.uniform(-0.13, 0.13, len(v))
        ax.scatter(i + jit, v, s=9, facecolor=METHOD[k], edgecolor="none",
                   alpha=0.45, zorder=3)
        ax.plot([i - 0.24, i + 0.24], [v.mean()] * 2, color=METHOD[k],
                lw=2.2 if k == "Ours_rollout" else 1.8, zorder=5,
                solid_capstyle="butt")
        ax.plot([i, i], [v.mean() - v.std(), v.mean() + v.std()],
                color=METHOD[k], lw=0.7, zorder=4)
        for yy in (v.mean() - v.std(), v.mean() + v.std()):
            ax.plot([i - 0.07, i + 0.07], [yy, yy], color=METHOD[k], lw=0.7,
                    zorder=4)
        ax.text(i + 0.27, v.mean(), "%.4f" % v.mean(), ha="left", va="center",
                fontsize=FS_TEXT, color=METHOD[k])
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([LABEL[k] for k in methods])
    ax.set_xlim(-0.55, len(methods) - 0.18)
    ax.set_ylim(-0.02, 0.66)
    ax.set_yticks([0, 0.2, 0.4, 0.6])
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
    ax.set_ylabel("animal–obstacle contact fraction $C$")
    ygrid(ax)
    thin(ax)
    ax.text(-0.5, 0.63, "V2S5A control: $C$ = 0 in 4 × 30 runs",
            fontsize=FS_TEXT, color=GREY_D, ha="left", va="top")
    panel(ax, "b", dx=-0.16)
    S.save(fig, OUT, "Fig8")


def fig9():
    p1 = pd.read_csv(os.path.join(P100, "p1_ratio.csv"))
    p2 = pd.read_csv(os.path.join(P100, "p2_methods.csv"))
    methods = ["Ours_rollout", "B1", "B2", "VH-Path"]

    fig, axes = plt.subplots(2, 2, figsize=(W190, 100 * MM))
    fig.subplots_adjust(left=0.085, right=0.985, bottom=0.10, top=0.92,
                        wspace=0.30, hspace=0.42)

    # (a) strict completion against robot count, both conditions
    ax = axes[0, 0]
    counts = [4, 6, 8, 10]
    for cond, col, filled, lab, off, dy, va in [
            ("no_obstacle", GREY_D, False, "no obstacle", -0.12, 0.05, "bottom"),
            ("obstacle", OURS, True, "one obstacle", 0.12, -0.05, "top")]:
        d = p1[p1.condition == cond].set_index("robots").loc[counts]
        ax.plot(np.array(counts) + off, d.completed / 30.0, color=col, lw=NORMAL,
                marker="o", ms=3.8, mfc=col if filled else "white", mec=col,
                mew=0.9, zorder=4)
        lo = d.wilson_lo.to_numpy()
        hi = d.wilson_hi.to_numpy()
        ax.plot([np.array(counts) + off, np.array(counts) + off], [lo, hi],
                color=col, lw=0.7, zorder=3)
        ax.plot(np.array(counts) + off, lo, ls="none", marker="_", ms=4,
                color=col, mew=0.7, zorder=3)
        ax.plot(np.array(counts) + off, hi, ls="none", marker="_", ms=4,
                color=col, mew=0.7, zorder=3)
        ax.text(counts[-1] + 0.42, d.completed.iloc[-1] / 30.0 + dy, lab,
                fontsize=FS_TEXT, color=col, va=va, ha="left")
    ax.set_xlim(3.4, 12.4)
    ax.set_ylim(0, 1.0)
    ax.set_xticks(counts)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))
    ax.set_xlabel("robots")
    ax.set_ylabel("strict completion (of 30)")
    ygrid(ax)
    thin(ax)
    panel(ax, "a", dx=-0.17)
    ax.set_title("completion by robot count", fontsize=FS_TEXT, pad=4)

    # (b) sheep-obstacle episodes per run against robot count
    ax = axes[0, 1]
    d = p1[p1.condition == "obstacle"].set_index("robots").loc[counts]
    ax.plot(counts, d.so_mean, color=OURS, lw=HEAVY, marker="o", ms=4.0,
            mfc=OURS, mec=OURS, zorder=4)
    ax.plot(counts, d.so_boot_lo, ls="none", marker="_", ms=4, color=OURS,
            mew=0.7, zorder=3)
    ax.plot(counts, d.so_boot_hi, ls="none", marker="_", ms=4, color=OURS,
            mew=0.7, zorder=3)
    ax.plot([counts, counts], [d.so_boot_lo, d.so_boot_hi], color=OURS, lw=0.7,
            zorder=3)
    ax.set_xlim(3.4, 11.4)
    ax.set_ylim(0, 17)
    ax.set_xticks(counts)
    ax.set_yticks([0, 4, 8, 12, 16])
    ax.set_xlabel("robots")
    ax.set_ylabel("sheep–obstacle episodes per run")
    ygrid(ax)
    thin(ax)
    panel(ax, "b", dx=-0.17)
    ax.set_title("obstacle contact by robot count", fontsize=FS_TEXT, pad=4)
    ax.text(4.05, 15.2, "one obstacle; 30 seeds", fontsize=FS_TEXT, color=GREY_D,
            va="top", ha="left")

    # (c) contact fraction by method at six robots
    ax = axes[1, 0]
    d = p2[(p2.condition == "obstacle")].set_index("method")
    for i, k in enumerate(methods):
        r = d.loc[k]
        ax.plot([i], [r.contact_rate_mean], marker="o", ms=4.2, color=METHOD[k],
                mfc=METHOD[k], mec=METHOD[k], zorder=4)
        ax.plot([i, i], [r.contact_rate_boot_lo, r.contact_rate_boot_hi],
                color=METHOD[k], lw=0.7, zorder=3)
        for yy in (r.contact_rate_boot_lo, r.contact_rate_boot_hi):
            ax.plot([i - 0.06, i + 0.06], [yy, yy], color=METHOD[k], lw=0.7,
                    zorder=3)
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels(["%s\n%d/30" % (LABEL[k], int(d.loc[k, "completed"]))
                        for k in methods])
    ax.set_xlim(-0.55, len(methods) - 0.45)
    ax.set_ylim(0, 0.140)
    ax.set_yticks([0, 0.04, 0.08, 0.12])
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))
    ax.set_ylabel("contact fraction $C$")
    ygrid(ax)
    thin(ax)
    panel(ax, "c", dx=-0.17)
    ax.set_title("methods at six robots", fontsize=FS_TEXT, pad=4)

    # (d) episodes per run by method at six robots
    ax = axes[1, 1]
    for i, k in enumerate(methods):
        r = d.loc[k]
        ax.plot([i], [r.so_mean], marker="o", ms=4.2, color=METHOD[k],
                mfc=METHOD[k], mec=METHOD[k], zorder=4)
        ax.plot([i, i], [r.so_boot_lo, r.so_boot_hi], color=METHOD[k], lw=0.7,
                zorder=3)
        for yy in (r.so_boot_lo, r.so_boot_hi):
            ax.plot([i - 0.06, i + 0.06], [yy, yy], color=METHOD[k], lw=0.7,
                    zorder=3)
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([LABEL[k] for k in methods])
    ax.set_xlim(-0.55, len(methods) - 0.45)
    ax.set_ylim(0, 15)
    ax.set_yticks([0, 4, 8, 12])
    ax.set_ylabel("sheep–obstacle episodes per run")
    ygrid(ax)
    thin(ax)
    panel(ax, "d", dx=-0.17)
    ax.set_title("methods at six robots", fontsize=FS_TEXT, pad=4)
    S.save(fig, OUT, "Fig9")


def main(argv=None):
    global OUT
    args = parse_args(argv)
    if args.out:
        OUT = args.out
    print("PKG          ", PKG)
    print("OUT          ", OUT)
    print("T3           ", T3)
    print("P100         ", P100, "(not shipped)")
    if not os.path.isdir(OUT):
        os.makedirs(OUT, exist_ok=True)
    if args.dry_run:
        print("inputs V2    ", V2())
        print("inputs S3    ", S3())
        print("inputs ROB   ", ROB())
        print("inputs ONL   ", ONL())
        print("dry run: no figure written")
        return 0
    S.setup()
    # Figures 1-8 are submitted Figs. 1-8.  Fig. 9 of the *submitted* article
    # is the robot-loss panel, built by fig_loss.py; fig9() below is the
    # Word-era 100 s-scale panel and needs the unshipped P100 input, so it is
    # only built on an explicit --only 9.
    wanted = ([int(v) for v in args.only.split(",")] if args.only
              else list(range(1, 9)))
    if 9 in wanted and not os.path.isdir(P100):
        raise SystemExit(
            "figure 9 of the Word build needs the 100 s-scale panel, which is "
            "not shipped in this repository (%s).\n"
            "The submitted Fig. 9 is the robot-loss panel: run "
            "07_Figure_Source/fig_loss.py instead." % P100)
    for n in wanted:
        globals()["fig%d" % n]()
    print("wrote", len(wanted), "figure(s) to", OUT)
    return 0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Rebuild body figures 1-8 (submitted Figs. 1-8) in %s."
                    % OUT)
    ap.add_argument("--only", default=None,
                    help="comma list of figure numbers to build, e.g. 1,8")
    ap.add_argument("--out", default=None,
                    help="output directory (default: %s)" % OUT)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the resolved paths and exit")
    return ap.parse_args(argv)


if __name__ == "__main__":
    sys.exit(main())
