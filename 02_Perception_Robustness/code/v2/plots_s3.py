"""Stage-3 figures (read the CSVs written by the acceptance suite).

    python -X utf8 -m v2.plots_s3      ->  results_v2_stage3/plots/*.png
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "results_v2_stage3"
OBS = OUT / "observability"
PL = OUT / "plots"


def rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def fig_observable():
    agg = rows(OBS / "observable_table_R70.csv")
    cal = json.loads((OUT / "ttc_calibration.json").read_text(encoding="utf-8"))
    geoms = sorted({r["geometry"] for r in agg})
    speeds = sorted({float(r["speed"]) for r in agg})
    fig, ax = plt.subplots(figsize=(7.2, 4.4), dpi=150)
    for g in geoms:
        xs, ys, lo = [], [], []
        for v in speeds:
            r = next(x for x in agg if x["geometry"] == g
                     and float(x["speed"]) == v)
            xs.append(v)
            ys.append(float(r["T_obs_mean"]))
            lo.append(float(r["T_obs_p10"]))
        ax.plot(xs, ys, "-o", label=g, lw=1.6, ms=5)
        ax.plot(xs, lo, ":", lw=1.0, alpha=0.75)
    fr = cal["frozen"]
    ax.axhline(16.0, color="crimson", ls="--", lw=1.2,
               label="proposed T_PRE = 16 s")
    ax.axhline(fr["ttc_pre"], color="seagreen", ls="-", lw=1.4,
               label=f"frozen T_PRE = {fr['ttc_pre']:g} s")
    ax.axhline(fr["ttc_divert"], color="darkorange", ls="-.", lw=1.2,
               label=f"frozen T_DIVERT = {fr['ttc_divert']:g} s")
    ax.set_xlabel("obstacle approach speed [m/s]")
    ax.set_ylabel("observable warning time [s]")
    ax.set_title("Observable warning at 70 m (solid = mean, dotted = p10)")
    ax.set_xticks(speeds)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, ncol=2)
    fig.tight_layout()
    fig.savefig(PL / "C_observable_time.png")
    plt.close(fig)


def fig_range():
    sw = rows(OBS / "observable_sweep.csv")
    cal = json.loads((OUT / "ttc_calibration.json").read_text(encoding="utf-8"))
    # exact-quality slice only (delay = loss = 0)
    s = [r for r in sw if float(r["delay"]) == 0.0 and float(r["loss"]) == 0.0]
    R = [float(r["R"]) for r in s]
    p10 = [float(r["T_obs_p10"]) for r in s]
    p10a = [float(r["T_obs_p10_all"]) for r in s]
    st = [float(r["stress_p10"]) for r in s]
    fig, ax = plt.subplots(figsize=(6.6, 4.2), dpi=150)
    ax.plot(R, p10, "-o", label="design envelope 3-4 m/s (p10)", lw=1.7)
    ax.plot(R, st, "-s", label="6 m/s stress (p10)", lw=1.7)
    ax.plot(R, p10a, ":", label="all speeds (p10)", lw=1.2)
    fr = cal["frozen"]
    ax.axhline(fr["ttc_pre"], color="seagreen", ls="-", lw=1.2,
               label=f"frozen T_PRE = {fr['ttc_pre']:g} s")
    ax.axhline(16.0, color="crimson", ls="--", lw=1.2,
               label="proposed T_PRE = 16 s")
    ax.axvline(70.0, color="grey", ls="--", lw=1.0)
    ax.annotate("initial 70 m", (70.0, ax.get_ylim()[0]), fontsize=7.5,
                rotation=90, va="bottom", ha="right", color="grey")
    ax.annotate(f"16 s needs {cal['required_range']['stress_16']:.0f} m "
                f"at 6 m/s", (0.03, 0.06), xycoords="axes fraction",
                fontsize=8, color="crimson")
    ax.set_xlabel("robot->obstacle range R [m]")
    ax.set_ylabel("10th percentile observable time [s]")
    ax.set_title("Range sweep (20 calibration seeds x 5 geometries)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5)
    fig.tight_layout()
    fig.savefig(PL / "D_range_sweep.png")
    plt.close(fig)


def fig_pilot():
    p = OUT / "E_pilot_summary.csv"
    if not p.exists():
        return
    t = rows(p)
    arms = []
    for r in t:
        if r["arm"] not in arms:
            arms.append(r["arm"])
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), dpi=150)
    for ax, sid in zip(axes, ("S2", "S3")):
        xs = np.arange(len(arms))
        for i, alg in enumerate(("Ours_rollout", "B2")):
            vals = [float(next(r for r in t if r["arm"] == a
                               and r["scenario"] == sid
                               and r["alg"] == alg)["C"]) for a in arms]
            ax.bar(xs + (i - 0.5) * 0.38, vals, width=0.36,
                   label=("Ours (full)" if alg == "Ours_rollout" else "B2"))
        ax.set_xticks(xs)
        ax.set_xticklabels([a.replace("_", "\n") for a in arms], fontsize=7)
        ax.set_ylabel("contacted animal fraction C")
        ax.set_title(sid)
        ax.grid(alpha=0.3, axis="y")
        ax.legend(fontsize=8)
    fig.suptitle("Honest-sensing pilot (2 seeds): contact fraction", fontsize=10)
    fig.tight_layout()
    fig.savefig(PL / "E_pilot.png")
    plt.close(fig)


def main():
    PL.mkdir(parents=True, exist_ok=True)
    fig_observable()
    fig_range()
    fig_pilot()
    print(f"wrote {PL}")


if __name__ == "__main__":
    main()
