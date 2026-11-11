"""Supplementary figures S1-S4, rebuilt with the same design system as the body.

Ported from the submission package's ``06_图表源码/fig_si.py``; the drawing code
is unchanged and ``pkgpaths.py`` maps the input tree onto this repository.
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
from matplotlib.ticker import FormatStrFormatter, LogLocator, NullFormatter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v9style as S
from v9style import (ACCENT, FS_LABEL, FS_TEXT, GREY_D, GREY_L, GREY_M,
                     GREY_XL, HEAVY, INK, LABEL, METHOD, METHODS, MM, NORMAL,
                     OURS, W190, panel, thin, xgrid, ygrid)

# Paths are mapped onto this repository's layout by pkgpaths.py: the archived
# result tables that the submission package read out of its nested archive
# (TTC_Shepherding_Complete_Reproducibility_Package.zip) are shipped here
# directly, byte-identical.
import pkgpaths  # noqa: E402

PKG = str(pkgpaths.ROOT)
OUT = str(pkgpaths.OUT_SI)


def V2(*parts):
    return str(pkgpaths.TABLES.joinpath(*parts))


def ROB(*parts):
    return str(pkgpaths.PERCEPTION.joinpath(*parts))


def ONL(*parts):
    return str(pkgpaths.ONLINE.joinpath(*parts))


# ------------------------------------------------------------------ figure S1
def figs1():
    c = pd.read_csv(V2("table_contrasts.csv"))
    order = ["B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
             "B-Outmost", "B-PCBF"]
    eps = [("C", "$\\Delta C$", (-0.26, 0.04)),
           ("clr_so", "$\\Delta$ clearance [m]", (-2.6, 2.2)),
           ("devW", "$\\Delta$ max dev/$W$", (-3.6, 0.4))]
    fig, axes = plt.subplots(1, 3, figsize=(W190, 64 * MM), sharey=True)
    fig.subplots_adjust(left=0.115, right=0.99, bottom=0.26, top=0.90,
                        wspace=0.10)
    y = np.arange(len(order))[::-1]
    xticks = {0: [-0.24, -0.16, -0.08, 0.0], 1: [-2, -1, 0, 1, 2],
              2: [-3, -2, -1, 0]}
    for pi, (ax, (ep, lab, xlim)) in enumerate(zip(axes, eps)):
        d = c[c.endpoint == ep].set_index("baseline").loc[order]
        for yi, (b, row) in zip(y, d.iterrows()):
            lo, hi, med = row.ci_lo, row.ci_hi, row.delta
            if not np.isfinite(lo):
                continue
            sig = row.p_holm < 0.05
            lclip = lo < xlim[0]
            hclip = hi > xlim[1]
            lc = xlim[0] if lclip else lo
            hc = xlim[1] if hclip else hi
            ax.plot([lc, hc], [yi, yi], color=INK, lw=0.9, zorder=3,
                    solid_capstyle="butt")
            if lclip:
                ax.plot([lc], [yi], marker="<", ms=3.4, color=INK, zorder=3,
                        clip_on=False)
            else:
                ax.plot([lc, lc], [yi - 0.12, yi + 0.12], color=INK, lw=0.9,
                        zorder=3)
            if hclip:
                ax.plot([hc], [yi], marker=">", ms=3.4, color=INK, zorder=3,
                        clip_on=False)
            else:
                ax.plot([hc, hc], [yi - 0.12, yi + 0.12], color=INK, lw=0.9,
                        zorder=3)
            if lclip or hclip:
                ax.annotate(("%+.1f" if ep == "clr_so" else "%+.2f") % med,
                            (lc if lclip else hc, yi), xytext=(0, 8),
                            textcoords="offset points", fontsize=FS_TEXT,
                            ha="left" if lclip else "right", va="bottom")
            ax.plot([med], [yi], marker="o", ms=3.8, color=INK,
                    mfc=INK if sig else "white", mec=INK, mew=0.9, zorder=4)
        ax.axvline(0, color=GREY_D, lw=0.7, zorder=2)
        ax.set_xlabel(lab)
        ax.set_xlim(*xlim)
        ax.set_xticks(xticks[pi])
        ax.set_yticks(y)
        if ax is axes[0]:
            ax.set_yticklabels([LABEL[k] for k in order])
        ax.xaxis.set_major_formatter(FormatStrFormatter(
            "%.1f" if ep == "devW" else ("%.0f" if ep == "clr_so" else "%.2f")))
        ax.tick_params(axis="y", length=0)
        thin(ax)
    for ax, letter in zip(axes, "abc"):
        panel(ax, letter, dx=-0.10)
    fig.text(0.5, 0.035, "filled: significant after Holm correction; "
             "open: not significant; bars are 95 % bootstrap intervals",
             ha="center", fontsize=FS_TEXT, color=GREY_D)
    S.save(fig, OUT, "FigS1")


# ------------------------------------------------------------------ figure S2
def figs2():
    m = pd.read_csv(V2("table_main9.csv")).set_index("method")
    cen = pd.read_csv(V2("table_census.csv")).set_index("method")
    fig, axes = plt.subplots(1, 2, figsize=(W190, 64 * MM))
    fig.subplots_adjust(left=0.125, right=0.985, bottom=0.17, top=0.93,
                        wspace=0.14)
    y = np.arange(len(METHODS))[::-1]

    ax = axes[0]
    for yi, k in zip(y, METHODS):
        v = float(m.loc[k, "ctrl_ms"])
        ax.plot([v], [yi], marker="o", ms=4.0, color=METHOD[k], zorder=4)
        ax.text(v * 1.12, yi, "%.2f" % v, va="center", fontsize=FS_TEXT)
    ax.axvline(1.68, color=INK, lw=0.7, ls=(0, (4, 2)), zorder=2)
    ax.text(1.75, len(METHODS) - 0.62, "1.68 ms", fontsize=FS_TEXT,
            ha="left", va="center")
    ax.set_xscale("log")
    ax.set_xlim(0.42, 120)
    ax.xaxis.set_major_locator(LogLocator(base=10, subs=(1, 3)))
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.set_xticks([0.5, 1, 2, 5, 10, 20, 50, 100])
    ax.set_xticklabels(["0.5", "1", "2", "5", "10", "20", "50", "100"])
    ax.set_yticks(y)
    ax.set_yticklabels([LABEL[k] for k in METHODS])
    ax.set_ylim(-0.8, len(METHODS) - 0.2)
    ax.set_xlabel("controller cycle time [ms], 12-way concurrent")
    xgrid(ax)
    ax.tick_params(axis="y", length=0)
    thin(ax)
    panel(ax, "a", dx=-0.28)

    ax = axes[1]
    fin = (cen.loc[METHODS, "runs"] - cen.loc[METHODS, "censored"]).to_numpy()
    cens = cen.loc[METHODS, "censored"].to_numpy()
    ax.barh(y, fin, height=0.55, color=GREY_L, edgecolor="none", zorder=3)
    ax.barh(y, cens, left=fin, height=0.55, color=ACCENT, edgecolor="none",
            zorder=3)
    for yi, f_, c_ in zip(y, fin, cens):
        if c_ > 0:
            ax.text(f_ + c_ + 6, yi, "%d" % c_, va="center", fontsize=FS_TEXT,
                    color=ACCENT)
    ax.set_yticks(y)
    ax.set_yticklabels([])
    ax.set_xlim(0, 340)
    ax.set_ylim(-0.8, len(METHODS) - 0.2)
    ax.set_xticks([0, 100, 200, 300])
    ax.set_xlabel("runs (of 300 per method)")
    ax.tick_params(axis="y", length=0)
    xgrid(ax)
    thin(ax)
    ax.text(6, -0.62, "grey: reached the goal;  orange: right-censored",
            fontsize=FS_TEXT, color=GREY_D, ha="left", va="center")
    panel(ax, "b", dx=-0.05)
    S.save(fig, OUT, "FigS2")


# ------------------------------------------------------------------ figure S3
def figs3():
    d = pd.read_csv(ROB("summary.csv"))
    methods = ["Ours_rollout", "B1", "B2"]
    ex = d[(d.version.isin(["archive", "strict", "safe"]))
           & (d.factor == "exact") & (d.method.isin(methods))]

    fig, axes = plt.subplots(1, 2, figsize=(W190, 62 * MM))
    fig.subplots_adjust(left=0.10, right=0.985, bottom=0.26, top=0.92,
                        wspace=0.30)
    versions = [("archive", "open"), ("strict", "grey"), ("safe", "filled")]
    offs = {"archive": -0.22, "strict": 0.0, "safe": 0.22}
    x = np.arange(len(methods))

    for ax, col, ylab, ylim, yt, letter in [
            (axes[0], "rs_clearance_mean",
             "mean robot–animal clearance [m]", (-0.78, 0.46),
             [-0.6, -0.4, -0.2, 0, 0.2, 0.4], "a"),
            (axes[1], "C", "animal–obstacle contact fraction $C$",
             (-0.015, 0.31), [0, 0.1, 0.2, 0.3], "b")]:
        g = ex.groupby(["method", "version"])[col].mean()
        for ver, kind in versions:
            for i, k in enumerate(methods):
                v = float(g.loc[(k, ver)])
                if kind == "open":
                    ax.plot([i + offs[ver]], [v], marker="o", ms=4.0,
                            color=METHOD[k], mfc="white", mec=METHOD[k],
                            mew=0.9, zorder=4)
                elif kind == "grey":
                    ax.plot([i + offs[ver]], [v], marker="s", ms=3.6,
                            color=GREY_D, mfc=GREY_L, mec=GREY_D, mew=0.7,
                            zorder=4)
                else:
                    ax.plot([i + offs[ver]], [v], marker="o", ms=4.4,
                            color=METHOD[k], mfc=METHOD[k], mec=METHOD[k],
                            zorder=4)
        ax.set_xticks(x)
        ax.set_xticklabels([LABEL[k] for k in methods])
        ax.set_xlim(-0.6, len(methods) - 0.4)
        ax.set_ylim(*ylim)
        ax.set_yticks(yt)
        ax.yaxis.set_major_formatter(FormatStrFormatter(
            "%.1f" if letter == "a" else "%.1f"))
        ax.set_ylabel(ylab)
        ygrid(ax)
        thin(ax)
        panel(ax, letter, dx=-0.20)
    axes[0].axhline(0, color=INK, lw=0.7, ls=(0, (3, 2)), zorder=2)
    axes[1].axhline(0, color=INK, lw=0.7, zorder=2)
    hs = [plt.Line2D([], [], marker="o", lw=0, color=INK, mfc="white",
                     mec=INK, label="archive"),
          plt.Line2D([], [], marker="s", lw=0, color=GREY_D, mfc=GREY_L,
                     mec=GREY_D, label="strict input"),
          plt.Line2D([], [], marker="o", lw=0, color=INK, mfc=INK, mec=INK,
                     label="with supervisor")]
    fig.legend(handles=hs, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 0.005), columnspacing=2.0)
    S.save(fig, OUT, "FigS3")


# ------------------------------------------------------------------ figure S4
def figs4():
    t = pd.read_csv(ONL("timing_summary.csv"))
    factors = ["exact", "animal_pos_0p5", "delay_0p5", "blackout_1p0"]
    flab = ["exact", "animal σ 0.5 m", "delay 0.5 s", "interrupt 1.0 s"]
    methods = ["Ours_rollout", "B1", "B2"]
    fig, ax = plt.subplots(figsize=(W190, 58 * MM))
    fig.subplots_adjust(left=0.175, right=0.975, bottom=0.30, top=0.93)
    y = np.arange(len(factors))[::-1]
    for yi, f in zip(y, factors):
        ax.axhline(yi, color=GREY_XL, lw=0.5, zorder=0)
        for k, off in zip(methods, (0.16, 0.0, -0.16)):
            v = float(t[(t.factor == f) & (t.method == k)].cycle_p99_ms.iloc[0])
            ax.plot([v], [yi + off], marker="o",
                    ms=4.0 if k == "Ours_rollout" else 3.4, color=METHOD[k],
                    mfc=METHOD[k], mec=METHOD[k], lw=0, zorder=4)
    ax.axvline(100.0, color=INK, lw=0.7, ls=(0, (4, 2)), zorder=2)
    ax.text(96, len(factors) - 0.45, "100 ms budget", fontsize=FS_TEXT,
            ha="right", va="center")
    ax.set_xscale("log")
    ax.set_xlim(8, 160)
    ax.set_xticks([10, 20, 50, 100])
    ax.set_xticklabels(["10", "20", "50", "100"])
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.set_yticks(y)
    ax.set_yticklabels(flab)
    ax.set_ylim(-0.6, len(factors) - 0.3)
    ax.set_xlabel("p99 complete-cycle time [ms]")
    hs = [plt.Line2D([], [], marker="o", lw=0, color=METHOD[k],
                     label=LABEL[k], ms=3.8) for k in methods]
    fig.legend(handles=hs, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.55, 0.01))
    thin(ax)
    S.save(fig, OUT, "FigS4")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Rebuild supplementary figures S1-S4 in %s." % OUT)
    ap.add_argument("--only", default=None,
                    help="comma list of supplementary figures, e.g. 1,4")
    ap.add_argument("--out", default=None,
                    help="output directory (default: %s)" % OUT)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the resolved paths and exit")
    return ap.parse_args(argv)


def main(argv=None):
    global OUT
    args = parse_args(argv)
    if args.out:
        OUT = args.out
    print("PKG          ", PKG)
    print("OUT          ", OUT)
    if not os.path.isdir(OUT):
        os.makedirs(OUT, exist_ok=True)
    if args.dry_run:
        print("inputs V2    ", V2())
        print("inputs ROB   ", ROB())
        print("inputs ONL   ", ONL())
        print("dry run: no figure written")
        return 0
    S.setup()
    wanted = ([int(v) for v in args.only.split(",")] if args.only
              else list(range(1, 5)))
    for n in wanted:
        globals()["figs%d" % n]()
    print("wrote", len(wanted), "figure(s) to", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
