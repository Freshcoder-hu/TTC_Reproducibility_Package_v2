"""Body figure 9: the 36:12 robot-loss panel.

Ported from the submission package's ``06_图表源码/fig_loss.py``; the drawing
code is unchanged and ``pkgpaths.py`` maps ``04_数据/机器人损毁_1080/`` onto this
repository's ``06_Robot_Loss_1080/`` (byte-identical inputs).

Four panels, 30 paired seeds per cell:
(a) animals with obstacle contact [%] by failure condition and scene;
(b) lateral deviation max dev / W by scene;
(c) attribution of the robot--animal contact episodes (body of the disabled
    robot vs the remaining eleven), with the paired F0 level marked;
(d) goal-and-cohesion completion [%] by method.

Same design system as figures 1--8 (v9style): one palette, one type scale,
panel letters at a fixed offset, 183.6 mm design width.
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
from matplotlib.patches import Patch
from matplotlib.ticker import FormatStrFormatter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v9style as S
from v9style import (FS_TEXT, GREY_D, GREY_L, HEAVY, LABEL, METHOD, MM,
                     NORMAL, OURS, W, panel, thin, ygrid)

import pkgpaths  # noqa: E402

PKG = str(pkgpaths.ROOT)
LOSS = str(pkgpaths.LOSS)
OUT = os.environ.get("OUT", str(pkgpaths.OUT_BODY))


def summary():
    return pd.read_csv(os.path.join(LOSS, "summary.csv"))


def attribution():
    return pd.read_csv(os.path.join(LOSS, "attribution.csv"))


def fig9():
    s = summary()
    a = attribution()
    conds = ["F0", "F1", "F2", "F3"]
    x = np.arange(len(conds))
    o = s[s.method == "Ours_rollout"].set_index(["scene", "condition"])

    fig, axes = plt.subplots(2, 2, figsize=(W, 104 * MM))
    fig.subplots_adjust(left=0.088, right=0.985, bottom=0.10, top=0.915,
                        wspace=0.30, hspace=0.46)

    # (a) animals with obstacle contact, by scene -------------------------
    ax = axes[0, 0]
    for scene, col, lab in [("V2S2", METHOD["B1"], "one obstacle"),
                            ("V2S3", OURS, "three threats")]:
        v = [o.loc[(scene, c), "collision_rate"] * 100 for c in conds]
        ax.plot(x, v, color=col, lw=HEAVY if scene == "V2S3" else NORMAL,
                marker="o", ms=3.8, mfc=col, mec=col, zorder=4)
        ax.text(x[-1] + 0.14, v[-1], lab, color=col, fontsize=FS_TEXT,
                va="center", ha="left")
    ax.set_xticks(x)
    ax.set_xticklabels(conds)
    ax.set_xlim(-0.25, 4.45)
    ax.set_ylim(0, 8)
    ax.set_yticks([0, 2, 4, 6, 8])
    ax.set_xlabel("failure condition")
    ax.set_ylabel("animals with obstacle contact [%]")
    ygrid(ax)
    thin(ax)
    panel(ax, "a", dx=-0.12, dy=1.09)
    ax.set_title("animal exposure", fontsize=FS_TEXT, pad=4)
    ax.text(-0.18, 7.6, "V2S1: 0 in all conditions", fontsize=FS_TEXT,
            color=GREY_D, ha="left", va="top")

    # (b) lateral deviation by scene --------------------------------------
    ax = axes[0, 1]
    for scene, col, lab in [("V2S1", GREY_D, "V2S1"),
                            ("V2S2", METHOD["B1"], "V2S2"),
                            ("V2S3", OURS, "V2S3")]:
        v = [o.loc[(scene, c), "max_dev_w"] for c in conds]
        ax.plot(x, v, color=col, lw=HEAVY if scene == "V2S3" else NORMAL,
                marker="o", ms=3.8, mfc=col, mec=col, zorder=4)
        ax.text(x[-1] + 0.14, v[-1], lab, color=col, fontsize=FS_TEXT,
                va="center", ha="left")
    ax.set_xticks(x)
    ax.set_xticklabels(conds)
    ax.set_xlim(-0.25, 4.30)
    ax.set_ylim(0.20, 0.68)
    ax.set_yticks([0.2, 0.3, 0.4, 0.5, 0.6])
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
    ax.set_xlabel("failure condition")
    ax.set_ylabel("max deviation / $W$")
    ygrid(ax)
    thin(ax)
    panel(ax, "b", dx=-0.12, dy=1.09)
    ax.set_title("lateral deviation (higher is worse)", fontsize=FS_TEXT, pad=4)

    # (c) contact attribution: body vs survivors --------------------------
    ax = axes[1, 0]
    ap = a[a.method == "Ours_rollout"].groupby("condition")[
        ["epi_body_C", "epi_rest_C", "epi_robot_F0"]].mean()
    xs = np.arange(3)
    body = np.array([ap.loc[c, "epi_body_C"] for c in conds[1:]])
    rest = np.array([ap.loc[c, "epi_rest_C"] for c in conds[1:]])
    ax.bar(xs, rest, width=0.55, color=GREY_L, edgecolor="none", zorder=3,
           label="remaining 11 robots")
    ax.bar(xs, body, width=0.55, bottom=rest, color=GREY_D, edgecolor="none",
           zorder=3, label="body of the disabled robot")
    for i, c in enumerate(conds[1:]):
        tot = ap.loc[c, "epi_robot_F0"]
        ax.plot([i - 0.34, i + 0.34], [tot, tot], color=OURS, lw=1.3, zorder=5)
        ax.text(i, body[i] + rest[i] + 0.28, "%.2f" % (body[i] + rest[i]),
                ha="center", va="bottom", fontsize=FS_TEXT)
    ax.plot([], [], color=OURS, lw=1.3, label="F0 total, robot alive")
    ax.set_xticks(xs)
    ax.set_xticklabels(conds[1:])
    ax.set_xlim(-0.6, 2.7)
    ax.set_ylim(0, 12.6)
    ax.set_yticks([0, 4, 8])
    ax.set_xlabel("failure condition")
    ax.set_ylabel("robot-involving episodes per run")
    ygrid(ax)
    thin(ax)
    panel(ax, "c", dx=-0.12, dy=1.09)
    ax.set_title("contact attribution", fontsize=FS_TEXT, pad=4)
    # The two-tone split (body of the disabled robot over the remaining
    # eleven) and the F0 rule are named in the caption rather than in a
    # legend: a legend large enough to be legible would cover the bar labels.

    # (d) goal-and-cohesion completion by method --------------------------
    ax = axes[1, 1]
    for k, dy in [("Ours_rollout", 0.8), ("B1", -0.8), ("B2", 0.0)]:
        d = s[s.method == k].groupby("condition")["joint"].mean()
        v = [d[c] * 100 for c in conds]
        ax.plot(x, v, color=METHOD[k],
                lw=HEAVY if k == "Ours_rollout" else NORMAL,
                marker="o", ms=3.8, mfc=METHOD[k], mec=METHOD[k], zorder=4)
        ax.text(x[-1] + 0.14, v[-1] + dy, LABEL[k], color=METHOD[k],
                fontsize=FS_TEXT, va="center", ha="left")
    ax.set_xticks(x)
    ax.set_xticklabels(conds)
    ax.set_xlim(-0.25, 4.20)
    ax.set_ylim(66, 88)
    ax.set_yticks([70, 75, 80, 85])
    ax.set_xlabel("failure condition")
    ax.set_ylabel("goal-and-cohesion completion [%]")
    ygrid(ax)
    thin(ax)
    panel(ax, "d", dx=-0.12, dy=1.09)
    ax.set_title("task completion", fontsize=FS_TEXT, pad=4)
    S.save(fig, OUT, "Fig9")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Rebuild body Fig. 9 (36:12 robot-loss panel) in %s." % OUT)
    ap.add_argument("--out", default=None,
                    help="output directory (default: %s)" % OUT)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the resolved paths and exit")
    return ap.parse_args(argv)


def main(argv=None):
    global OUT
    args = parse_args(argv)
    if args.out:
        OUT = os.path.abspath(args.out)
    print("PKG  ", PKG)
    print("LOSS ", LOSS)
    print("OUT  ", OUT)
    if args.dry_run:
        for name in ("summary.csv", "attribution.csv"):
            p = os.path.join(LOSS, name)
            print("input", p, "exists" if os.path.isfile(p) else "MISSING")
        print("dry run: no figure written")
        return 0
    S.setup()
    fig9()
    return 0


if __name__ == "__main__":
    sys.exit(main())
