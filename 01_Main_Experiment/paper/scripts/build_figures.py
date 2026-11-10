"""Regenerate every V7 figure from the frozen V2 CSVs.

Output: figures/<name>.pdf (vector) and figures/<name>.png (600 dpi).
Style: Elsevier -- 190 mm width, Times, >= 7 pt body text, no in-figure titles,
panel labels "(a) V2S1" etc.
"""
from __future__ import annotations

import json
import os
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common_paths import DATA, FIGURES, LABEL, METHODS, SCENARIOS, S3, TABLES

MM = 1 / 25.4
W190 = 190 * MM

mpl.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 7,
    "axes.labelsize": 7,
    "axes.titlesize": 7,
    "xtick.labelsize": 6.8,
    "ytick.labelsize": 6.8,
    "legend.fontsize": 6.4,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.2,
    "ytick.major.size": 2.2,
    "lines.linewidth": 1.0,
    "patch.linewidth": 0.6,
    "savefig.dpi": 600,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "figure.dpi": 150,
})

C_OURS = "#C1272D"
PALETTE = {
    "Ours_rollout": C_OURS,
    "B1": "#4D4D4D",
    "B2": "#8C8C8C",
    "VH-Path": "#0072B2",
    "VH-Danger": "#56B4E9",
    "VH-Danger-path": "#009E73",
    "B-Outmost": "#E69F00",
    "B-PCBF": "#CC79A7",
}


def save(fig, name):
    os.makedirs(FIGURES, exist_ok=True)
    fig.savefig(os.path.join(FIGURES, name + ".pdf"), bbox_inches="tight",
                pad_inches=0.01)
    fig.savefig(os.path.join(FIGURES, name + ".png"), bbox_inches="tight",
                pad_inches=0.01, dpi=600)
    plt.close(fig)
    print("figure", name)


def panel(ax, letter, dx=-0.12, dy=1.02):
    ax.text(dx, dy, f"({letter})", transform=ax.transAxes, fontsize=8,
            fontweight="bold", va="bottom", ha="left")


def load_tables():
    t = {}
    for k in ["main9", "main10", "per_scenario", "ablation", "contrasts",
              "census", "radar"]:
        t[k] = pd.read_csv(os.path.join(TABLES, f"table_{k}.csv"))
    with open(os.path.join(DATA, "numbers.json"), encoding="utf-8") as f:
        t["num"] = json.load(f)
    return t


# ---------------------------------------------------------------- figure 1
def fig_geometry(t):
    fig, axes = plt.subplots(2, 3, figsize=(W190, 92 * MM))
    axes = axes.ravel()

    def corridor(ax, L, half=8.0):
        ax.add_patch(Rectangle((0, -half), L, 2 * half, facecolor="#F1F1F1",
                               edgecolor="none", zorder=0))
        ax.plot([0, L], [half, half], color="k", lw=0.7, zorder=2)
        ax.plot([0, L], [-half, -half], color="k", lw=0.7, zorder=2)
        ax.plot([0, L], [0, 0], color="k", lw=0.5, ls=(0, (3, 3)), zorder=2)
        ax.set_ylim(-9.4, 9.4)
        ax.set_xlim(-6, L + 4)
        ax.set_yticks([-8, 0, 8])
        ax.tick_params(labelsize=7)
        ax.set_xlabel("arc length $s$ [m]", labelpad=1.2)

    def flock(ax, s=8.0, n=36, sy=2.3, sx=2.6):
        rng = np.random.default_rng(5)
        xs = s + rng.normal(0, sx, n)
        ys = np.clip(rng.normal(0, sy, n), -4.6, 4.6)
        ax.scatter(xs, ys, s=2.0, c="#222222", marker="o", lw=0, zorder=3)

    def obstacle(ax, x, y, u, tag="", dx=0.0, dy=0.0, ha="left", va="center"):
        ax.add_patch(plt.Circle((x, y), 1.2, facecolor=C_OURS, alpha=0.9,
                                edgecolor="none", zorder=4))
        ux, uy = u
        ax.annotate("", xy=(x + ux * 9, y + uy * 9), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", lw=0.9, color=C_OURS,
                                    shrinkA=5, shrinkB=1.5, mutation_scale=6))
        if tag:
            ax.text(x + dx, y + dy, tag, fontsize=7, color="#8B1A1F",
                    ha=ha, va=va, zorder=6)

    uL = np.array([-0.8, -0.45]); uL = uL / np.linalg.norm(uL)
    uR = np.array([-0.8, 0.45]); uR = uR / np.linalg.norm(uR)

    # (a) V2S1
    ax = axes[0]
    corridor(ax, 140)
    flock(ax, 8)
    ax.plot([140 * 0.92], [0], marker="*", ms=7, color="#0072B2", zorder=4)
    ax.text(140 * 0.92 - 3, 2.4, "goal", fontsize=7, ha="right")
    ax.set_ylabel("lateral [m]", labelpad=1.2)

    # (b) V2S1N
    ax = axes[1]
    L, bend, ang = 140.0, 60.0, np.deg2rad(75)
    s1 = np.linspace(0, bend, 120)
    s2 = np.linspace(0, L - bend, 180)
    x = np.concatenate([s1, bend + s2 * np.cos(ang)])
    y = np.concatenate([np.zeros_like(s1), s2 * np.sin(ang)])
    tx, ty = np.gradient(x), np.gradient(y)
    nn = np.hypot(tx, ty)
    nx, ny = -ty / nn, tx / nn
    for sgn in (+1, -1):
        ax.plot(x + sgn * 8 * nx, y + sgn * 8 * ny, color="k", lw=0.7)
    poly = np.concatenate([
        np.c_[x + 8 * nx, y + 8 * ny],
        np.c_[x - 8 * nx, y - 8 * ny][::-1]])
    ax.plot(x, y, color="k", lw=0.5, ls=(0, (3, 3)))
    ax.set_xlim(-8, L + 6)
    ax.set_ylim(-12, 46)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]", labelpad=1.2)
    ax.set_ylabel("y [m]", labelpad=1.2)
    ax.tick_params(labelsize=7)
    flock(ax, 8)

    # (c) V2S2 family
    ax = axes[2]
    corridor(ax, 140)
    flock(ax, 8)
    obstacle(ax, 70, 5.0, uL, dx=2.6, dy=2.0, ha="left")
    ax.text(72.6, 7.4, "speeds 3 / 4 / 5 /\n6 m/s", fontsize=7,
            ha="left", va="top", color="#8B1A1F")

    # (d) V2S3
    ax = axes[3]
    corridor(ax, 220)
    ax.set_xlim(-6, 226)
    flock(ax, 8)
    obstacle(ax, 70, 5.0, uL, tag="1: 3 m/s", dx=2.6, dy=2.0, ha="left")
    obstacle(ax, 83, -5.0, uR, tag="2: 6 m/s", dx=2.6, dy=-2.0, ha="left")
    obstacle(ax, 145, 5.0, uL, tag="3: 3 m/s", dx=2.6, dy=2.0, ha="left")

    # (e) S4 scale family: three strips
    ax = axes[4]
    ax.set_xlim(-4, 148)
    ax.set_ylim(-10.2, 10.2)
    ax.set_yticks([])
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    for i, (ns, nr, y0) in enumerate([(18, 6, 6.4), (36, 12, 0.0),
                                      (60, 20, -6.4)]):
        ax.add_patch(Rectangle((0, y0 - 2.4), 140, 4.8, facecolor="#F1F1F1",
                               edgecolor="k", lw=0.5, zorder=0))
        ax.plot([0, 140], [y0, y0], color="k", lw=0.4, ls=(0, (3, 3)))
        rng = np.random.default_rng(20 + i)
        ax.scatter(8 + rng.normal(0, 2.0, ns), y0 + rng.normal(0, 0.9, ns),
                   s=1.6, c="#222222", lw=0, zorder=3)
        ax.text(26, y0 + 1.3, f"{ns}:{nr}", fontsize=7, va="center", ha="left",
                color=C_OURS)
        ax.add_patch(plt.Circle((70, y0 + 1.4), 0.9, facecolor=C_OURS,
                                edgecolor="none", zorder=4))
        ax.annotate("", xy=(70 - 2.2, y0 + 0.4), xytext=(70, y0 + 1.4),
                    arrowprops=dict(arrowstyle="-|>", lw=0.7, color=C_OURS,
                                    shrinkA=2, shrinkB=1, mutation_scale=5))
    ax.set_xlabel("arc length $s$ [m]", labelpad=1.2)

    axes[5].axis("off")
    fig.subplots_adjust(wspace=0.40, hspace=0.62)

    for ax, letter, tag in zip(axes[:5], "abcde",
                               ["V2S1", "V2S1N", "V2S2 family", "V2S3",
                                "V2S4 family"]):
        ax.set_title("")
        ax.text(-0.12, 1.02, f"({letter}) {tag}", transform=ax.transAxes,
                fontsize=7.4, fontweight="bold", va="bottom", ha="left")
    save(fig, "fig03_geometry")


# ---------------------------------------------------------------- figure 2
def fig_coupling(t):
    fig, ax = plt.subplots(figsize=(W190, 110 * MM))
    ax.set_xlim(0, 190)
    ax.set_ylim(0, 110)
    ax.axis("off")

    def box(x, y, w, h, header, body, fc="#FFFFFF", ec="#333333"):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                                    boxstyle="round,pad=0.4,rounding_size=1.4",
                                    facecolor=fc, edgecolor=ec, lw=0.8))
        ax.text(x + w / 2, y + h - 4.0, header, fontsize=7.4, ha="center",
                va="center", fontweight="bold")
        ax.plot([x + 6, x + w - 6], [y + h - 7.2, y + h - 7.2],
                color=ec, lw=0.4, alpha=0.6)
        ax.text(x + w / 2, y + h - 9.0, body, fontsize=7, ha="center",
                va="top", linespacing=1.42)

    box(2, 54, 55, 40, "flock-level contact risk",
        "observed animals and\nobstacles $S(t)$\n\n"
        "$\\tau_{io}$ = first-contact time\nof the pair $(i,o)$\n\n"
        "$T(t)=\\min_{i,o}\\tau_{io}$", fc="#F5F5F5")
    box(67, 54, 55, 40, "four-state response",
        "normal guidance\npre-positioning\ndiversion\n"
        "preemptible recovery\n\nhandover of the\navoidance slot", fc="#EAF2F8",
        ec="#21618C")
    box(132, 54, 56, 40, "action-conditioned rollout",
        "candidates $\\delta$ = 0,\n$\\pm$1.35, $\\pm$2.4, $\\pm$3.6 m\n\n"
        "coupled robot-flock-obstacle\nprediction, $H$ = 12 s\n\n"
        "$J(\\delta)$: predicted contact,\npath, spread, switching",
        fc="#FDEDEC", ec=C_OURS)
    box(30, 4, 130, 26, "shared execution layer",
        "finite bodies $r$ = 0.45 m  |  $a_{\\mathrm{lin}}$ = 4.0, "
        "$a_{\\mathrm{lat}}$ = 2.0 m/s$^2$, $v_{\\max}$ = 4.5 m/s\n"
        "layered speed policy $v\\leq\\sqrt{a_{\\mathrm{lat}}\\,r_{\\mathrm{req}}}$ "
        "in pre-positioning and diversion",
        fc="#F4F6F6")

    def arrow(p, q, txt="", ls="-", color="#333333", rad=0.0, toff=(0, 1.2),
              fs=6.4, ha="center", va="bottom"):
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", lw=0.8,
                                     color=color, mutation_scale=7,
                                     linestyle=ls,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=1, shrinkB=1))
        if txt:
            ax.text((p[0] + q[0]) / 2 + toff[0], (p[1] + q[1]) / 2 + toff[1],
                    txt, fontsize=fs, ha=ha, va=va, color="#222222")

    arrow((57, 70), (67, 70), "when")
    arrow((122, 70), (132, 70), "mode,\n$\\delta_{\\mathrm{prev}}$")
    arrow((150, 54), (120, 30), "selected $\\delta^{*}$", rad=0.12,
          toff=(-2, 0.5), ha="right")
    arrow((60, 30), (24, 54), "executed motion", ls=(0, (3, 2)), rad=-0.30,
          toff=(-6, -1.5), va="top")
    arrow((29, 94), (160, 94), "the risk $T(t)$ of every candidate is predicted: "
          "selection is by consequence", rad=-0.10, color=C_OURS,
          toff=(0, 7.0), va="bottom")
    save(fig, "fig02_coupling")


# ---------------------------------------------------------------- figure 3
def fig_observability(t):
    obs = pd.read_csv(os.path.join(S3, "observability",
                                   "observable_table_R70.csv"))
    req = pd.read_csv(os.path.join(S3, "required_range.csv"))
    fig, axes = plt.subplots(1, 2, figsize=(W190, 64 * MM))
    ax = axes[0]
    names = {"head_on": ("head-on", "#0072B2", "-"),
             "cross_0": ("perpendicular", "#009E73", "-"),
             "cross_3": ("perpendicular, 3.6 m", "#009E73", "--"),
             "cross_6": ("perpendicular, 6.0 m", "#009E73", ":"),
             "oblique_3": ("oblique (S2 direction)", "#E69F00", "-")}
    for g, (name, cc, ls) in names.items():
        d = obs[obs.geometry == g].sort_values("speed")
        ax.plot(d.speed, d.T_obs_mean, ls=ls, marker="o", ms=2.4, lw=0.9,
                color=cc, label=name, alpha=0.95)
        ax.plot(d.speed, d.T_obs_p10, ls=":", lw=0.7, color=cc, alpha=0.8)
    ax.axhline(12.0, color=C_OURS, lw=0.9, ls="--")
    ax.text(2.85, 11.2, "$T_{\\mathrm{PRE}}$ = 12 s (frozen)", fontsize=7,
            color=C_OURS, ha="left", va="top")
    ax.axhline(16.0, color="#555555", lw=0.8, ls="-.")
    ax.text(6.2, 16.6, "$T_{\\mathrm{PRE}}$ = 16 s (proposed)", fontsize=7,
            color="#555555", ha="right", va="bottom")
    ax.set_xlabel("obstacle speed [m/s]")
    ax.set_ylabel("observable warning time [s]")
    ax.set_xlim(2.7, 6.25)
    ax.set_ylim(2, 25.5)
    ax.legend(frameon=False, loc="lower left", handlelength=1.6,
              borderaxespad=0.2)
    ax.text(2.85, 24.6, "$R$ = 70 m;  solid: mean, dotted: 10th percentile",
            fontsize=7)
    ax.text(-0.12, 1.02, "(a)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")

    ax = axes[1]
    d = req[req.geometry == "head_on"].sort_values("speed")
    for col, lab, ls, cc in [
            ("R_required_T_DIVERT", "8", ":", "#777777"),
            ("R_required_T_PRE_frozen", "12", "-", "#0072B2"),
            ("R_required_T_PRE_16", "16", "--", C_OURS)]:
        ax.plot(d.speed, d[col], ls=ls, marker="o", ms=2.6, lw=0.9, color=cc,
                label=f"$T$ = {lab} s")
    ax.axhline(70.0, color="k", lw=0.8)
    ax.text(3.02, 72.2, "$R$ = 70 m (configured)", fontsize=7)
    ax.text(4.10, 84.5, "88.9 m", fontsize=7, color=C_OURS)
    ax.text(4.10, 62.0, "69.2 m", fontsize=7, color="#0072B2")
    ax.text(5.92, 123.5, "121.2 m", fontsize=7, color=C_OURS, ha="right")
    ax.set_xlabel("obstacle speed [m/s]")
    ax.set_ylabel("range needed [m]")
    ax.set_xlim(2.7, 6.25)
    ax.set_ylim(30, 135)
    ax.legend(frameon=False, loc="lower right", title="warning time",
              title_fontsize=7, handlelength=1.6, borderaxespad=0.2)
    ax.text(2.85, 128, "head-on (the binding geometry)", fontsize=7)
    ax.text(-0.12, 1.02, "(b)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")
    fig.subplots_adjust(wspace=0.32)
    save(fig, "fig01_observability")


# ---------------------------------------------------------------- figure 4
def fig_main(t):
    m = t["main9"].set_index("method").loc[METHODS]
    fig, axes = plt.subplots(1, 2, figsize=(W190, 68 * MM))
    y = np.arange(len(METHODS))[::-1]
    ax = axes[0]
    ax.barh(y, m.C, color=[PALETTE[k] for k in METHODS], height=0.66,
            alpha=0.95)
    for yi, v in zip(y, m.C):
        ax.text(v + 0.004, yi, f"{v:.3f}", va="center", fontsize=7)
    ax.set_yticks(y)
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    ax.set_xlabel("contact fraction $C$ (animal$-$obstacle)")
    ax.set_xlim(0, 0.235)
    ax.text(-0.30, 1.02, "(a)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")

    ax = axes[1]
    ax.barh(y, m.clr_so, color=[PALETTE[k] for k in METHODS], height=0.66,
            alpha=0.95)
    for yi, v in zip(y, m.clr_so):
        ax.text(v + (0.5 if v >= 0 else -0.5), yi, f"{v:+.2f}", va="center",
                ha="left" if v >= 0 else "right", fontsize=7)
    ax.set_yticks(y)
    ax.set_yticklabels([])
    ax.set_xlabel("worst-case animal$-$obstacle surface clearance [m]")
    ax.axvline(0, color="k", lw=0.7)
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlim(-3.4, 55)
    ax.set_xticks([-2, -1, 0, 1, 2, 5, 10, 20])
    ax.set_xticklabels(["-2", "-1", "0", "1", "2", "5", "10", "20"])

    ax.text(-0.06, 1.02, "(b)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")
    fig.subplots_adjust(wspace=0.10)
    save(fig, "fig04_main_comparison")


# ---------------------------------------------------------------- figure 5
def fig_per_scene(t):
    p = t["per_scenario"]
    mat = p.pivot(index="method", columns="scenario", values="C").loc[
        METHODS, SCENARIOS]
    fig, ax = plt.subplots(figsize=(W190, 64 * MM))
    im = ax.imshow(mat.values, aspect="auto", cmap="OrRd", vmin=0,
                   vmax=float(np.nanmax(mat.values)))
    ax.set_xticks(range(len(SCENARIOS)))
    ax.set_xticklabels(SCENARIOS, rotation=40, ha="right",
                       rotation_mode="anchor")
    ax.set_yticks(range(len(METHODS)))
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            v = mat.values[i, j]
            ax.text(j, i, f"{v:.3f}", ha="center", va="center", fontsize=7,
                    color="white" if v > 0.55 * np.nanmax(mat.values)
                    else "black")
    ax.set_xticks(np.arange(-0.5, len(SCENARIOS), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(METHODS), 1), minor=True)
    ax.grid(which="minor", color="white", lw=0.8)
    ax.tick_params(which="minor", length=0)
    cb = fig.colorbar(im, ax=ax, fraction=0.026, pad=0.015)
    cb.set_label("contact fraction $C$", fontsize=7)
    cb.ax.tick_params(labelsize=7)
    ax.tick_params(axis="x", labelsize=7)
    save(fig, "fig05_per_scenario")


# ---------------------------------------------------------------- figure 6
def fig_ablation(t):
    a = t["ablation"]
    rungs = ["R0_four_state", "R1_handover", "R2_event", "R3_rollout"]
    labels = ["R0\nfour-state", "R1\n+handover", "R2\n+event", "R3\n+rollout"]
    fig, axes = plt.subplots(1, 2, figsize=(W190, 64 * MM))
    for ax, col, lab, letter in [
            (axes[0], "C", "contact fraction $C$", "a"),
            (axes[1], "clr_so", "minimum surface clearance [m]", "b")]:
        for s, marker, cc, name in [
                ("V2S2R2", "s", "#0072B2", "V2S2R2: one obstacle"),
                ("V2S3", "o", C_OURS, "V2S3: sequential")]:
            d = a[a.scenario == s].set_index("rung")
            ys = [d.loc[r, col] for r in rungs]
            ax.plot(range(4), ys, marker=marker, ms=3.4, lw=1.0, color=cc,
                    label=name)
            ax.scatter([3], [d.loc["R3_no_speed_policy", col]], marker=marker,
                       s=16, facecolor="white", edgecolor=cc, lw=0.8, zorder=5)
        ax.set_xticks(range(4))
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel(lab)
        ax.set_xlim(-0.35, 3.45)
        ax.text(-0.20, 1.02, f"({letter})", transform=ax.transAxes,
                fontsize=8, fontweight="bold")
    axes[0].legend(frameon=False, loc="center left", bbox_to_anchor=(0.02, 0.62),
                   handlelength=1.5)
    d = a[a.scenario == "V2S2R2"].set_index("rung")
    axes[0].annotate("R0 = R1 = R2, bit-identical\n(60/60 cells)",
                     (1.0, d.loc["R2_event", "C"]), xytext=(0.28, 0.075),
                     fontsize=7, color="#0072B2",
                     arrowprops=dict(arrowstyle="-", lw=0.6, color="#0072B2"))
    axes[0].annotate("$C$: 0.341 $\\rightarrow$\n0.017", (2.55, 0.30),
                     fontsize=7, color=C_OURS, ha="left", va="bottom")
    axes[1].annotate("$-1.44 \\rightarrow +0.11$ m", (2.6, -0.55),
                     xytext=(1.05, 0.45), fontsize=7, color=C_OURS,
                     arrowprops=dict(arrowstyle="->", lw=0.6, color=C_OURS))
    fig.subplots_adjust(wspace=0.30)
    save(fig, "fig06_ablation")


# ---------------------------------------------------------------- figure 7
def fig_intervals(t):
    c = t["contrasts"]
    order = ["B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
             "B-Outmost", "B-PCBF"]
    eps = [("C", "$\\Delta C$", (-0.25, 0.02)),
           ("clr_so", "$\\Delta$ clearance [m]", (-2.6, 2.6)),
           ("devW", "$\\Delta$ max dev/$W$", (-3.6, 0.4))]
    fig, axes = plt.subplots(1, 3, figsize=(W190, 62 * MM))
    for ax, (ep, lab, xlim), letter in zip(axes, eps, "abc"):
        d = c[c.endpoint == ep].set_index("baseline").loc[order]
        y = np.arange(len(order))[::-1]
        for yi, (l, m_, h, s) in zip(y, zip(d.ci_lo, d.delta, d.ci_hi,
                                             d.p_holm < 0.05)):
            if not np.isfinite(l):
                continue
            lc, hc = max(l, xlim[0]), min(h, xlim[1])
            ax.plot([lc, hc], [yi, yi], color="#333333", lw=1.1,
                    solid_capstyle="butt")
            for xx in (lc, hc):
                ax.plot([xx, xx], [yi - 0.14, yi + 0.14], color="#333333",
                        lw=1.1)
            if h > xlim[1] or l < xlim[0]:
                left = l < xlim[0]
                ax.annotate(f"{m_:+.1f}" if ep == "clr_so" else f"{m_:+.2f}",
                            (lc if left else hc, yi),
                            xytext=(6 if left else -6, 0),
                            textcoords="offset points", fontsize=7,
                            ha="left" if left else "right",
                            va="center", color="#333333")
            ax.plot([m_], [yi], marker="o", ms=3.6,
                    color=C_OURS if s else "white", markeredgecolor=C_OURS,
                    markeredgewidth=0.9, zorder=4)
        ax.axvline(0, color="k", lw=0.7)
        ax.set_yticks(y)
        ax.set_yticklabels([LABEL[k] for k in order], fontsize=7)
        ax.set_xlabel(lab, fontsize=7)
        ax.set_xlim(*xlim)
        ax.tick_params(labelsize=7)
        ax.text(-0.34, 1.02, f"({letter})", transform=ax.transAxes,
                fontsize=8, fontweight="bold")
    fig.subplots_adjust(wspace=0.85)
    save(fig, "fig07_intervals")


# ---------------------------------------------------------------- figure 8
def fig_radar(t):
    r = t["radar"].set_index("method").loc[METHODS]
    keys = ["C", "clr_so", "ttc_min", "rho", "joint", "devW"]
    klab = ["$C$\nlow", "clearance\nhigh", "min TTC\nhigh", "$\\rho$\nhigh",
            "joint\ncompletion", "dev/$W$\nlow"]
    ang = np.linspace(0, 2 * np.pi, len(keys), endpoint=False)
    ang = np.concatenate([ang, ang[:1]])
    fig = plt.figure(figsize=(W190, 80 * MM))
    ax = fig.add_axes([0.00, 0.00, 0.54, 1.0], polar=True)
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)
    for k in METHODS:
        v = r.loc[k, keys].to_numpy(dtype=float)
        v = np.concatenate([v, v[:1]])
        ax.plot(ang, v, lw=1.7 if k == "Ours_rollout" else 0.8,
                color=PALETTE[k], alpha=1.0 if k == "Ours_rollout" else 0.8,
                label=LABEL[k], zorder=5 if k == "Ours_rollout" else 3)
        if k == "Ours_rollout":
            ax.fill(ang, v, color=PALETTE[k], alpha=0.08, zorder=2)
    ax.set_xticks(ang[:-1])
    ax.set_xticklabels(klab, fontsize=7)
    ax.set_ylim(0, 1.0)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0.25", "0.50", "0.75", "1.00"], fontsize=7)
    ax.tick_params(pad=2)
    ax.grid(color="#BBBBBB", lw=0.5)
    ax.spines["polar"].set_linewidth(0.6)
    fig.legend(loc="center left", bbox_to_anchor=(0.545, 0.55), ncol=1,
               frameon=False, fontsize=7, handlelength=1.4,
               labelspacing=0.55)
    fig.text(0.555, 0.06, "min$-$max over the eight methods;\n"
             "1 = best, 0 = worst on that axis", fontsize=7, ha="left")
    save(fig, "fig09_radar")


# ---------------------------------------------------------------- figure 9
def fig_timing(t):
    m = t["main9"].set_index("method").loc[METHODS]
    cen = t["census"].set_index("method").loc[METHODS]
    fig, axes = plt.subplots(1, 2, figsize=(W190, 62 * MM))
    y = np.arange(len(METHODS))[::-1]
    ax = axes[0]
    ax.barh(y, m.ctrl_ms, color=[PALETTE[k] for k in METHODS], height=0.62)
    ax.set_xscale("log")
    for yi, v in zip(y, m.ctrl_ms):
        ax.text(v * 1.10, yi, f"{v:.2f}", va="center", fontsize=7)
    ax.axvline(1.68, color=C_OURS, lw=0.8, ls="--")
    ax.text(1.80, -0.55, "dashed: 1.68 ms, uncontended serial",
            fontsize=7, color=C_OURS, ha="left", va="center")
    ax.set_yticks(y)
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    ax.set_xlabel("controller cycle time [ms], 12-way concurrent")
    ax.set_xlim(0.45, 120)
    ax.set_ylim(-0.95, 7.6)
    ax.text(-0.30, 1.02, "(a)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")

    ax = axes[1]
    fin = (cen.runs - cen.censored).to_numpy()
    cens = cen.censored.to_numpy()
    ax.barh(y, fin, color="#C8C8C8", height=0.62)
    ax.barh(y, cens, left=fin, color=C_OURS, height=0.62)
    for yi, f_, c_ in zip(y, fin, cens):
        if c_ > 0:
            ax.text(f_ + c_ + 5, yi, f"{c_}", va="center", fontsize=7)
    ax.set_yticks(y)
    ax.set_yticklabels([])
    ax.set_xlabel("runs (of 300 per method)")
    ax.set_xlim(0, 335)
    ax.set_ylim(-0.95, 7.6)
    ax.text(3, -0.55, "grey: reached the goal;  red: right-censored",
            fontsize=7, ha="left", va="center")
    ax.text(-0.06, 1.02, "(b)", transform=ax.transAxes, fontsize=8,
            fontweight="bold")
    fig.subplots_adjust(wspace=0.10, bottom=0.17)
    save(fig, "fig08_timing")


def main():
    t = load_tables()
    fig_geometry(t)
    fig_coupling(t)
    fig_observability(t)
    fig_main(t)
    fig_per_scene(t)
    fig_ablation(t)
    fig_intervals(t)
    fig_radar(t)
    fig_timing(t)


if __name__ == "__main__":
    main()








