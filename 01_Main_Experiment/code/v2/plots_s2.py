"""Stage-2 behaviour plots (reads the acceptance/calibration CSVs).

    python -X utf8 -m v2.plots_s2

Writes PNGs to ``results_v2_stage2/plots/``.  Nothing here feeds back into any
number reported elsewhere: the plots are drawn from the CSV/JSON artefacts that
``v2.acceptance_s2`` and ``v2.calibrate_v2`` write.
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
OUT = ROOT / "results_v2_stage2"
BEH = OUT / "behaviour"
PLOTS = OUT / "plots"


def read(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def fnum(rows, key):
    return np.array([float(r[key]) for r in rows])


def plot_b():
    rows = read(BEH / "B_corridor.csv")
    per = read(BEH / "B_corridor_per_seed.csv")
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    labels = [r["setting"] for r in rows]
    R = np.array([float(r["sense_range"]) for r in rows])
    ax[0].bar(range(len(rows)), fnum(rows, "animal_contact_fraction"),
              yerr=fnum(rows, "animal_contact_fraction_sd"), color="C3")
    ax[0].set_xticks(range(len(rows)))
    ax[0].set_xticklabels(labels)
    ax[0].set_ylabel("animal contact fraction")
    ax[0].set_title("B: corridor contact rate (20 seeds)")

    ax[1].bar(range(len(rows)), fnum(rows, "mean_min_clearance"),
              yerr=fnum(rows, "mean_min_clearance_sd"), color="C0")
    ax[1].set_xticks(range(len(rows)))
    ax[1].set_xticklabels(labels)
    ax[1].axhline(0.0, color="k", lw=0.8, ls=":")
    ax[1].set_ylabel("mean min surface clearance [m]")
    ax[1].set_title("B: clearance (M1 monotone in range)")

    for lab in labels:
        sel = [r for r in per if r["setting"] == lab]
        if not sel:
            continue
        y = np.array([float(r["animal_contact_fraction"]) for r in sel])
        x = np.full(len(y), labels.index(lab)) + np.linspace(-0.15, 0.15, len(y))
        ax[2].scatter(x, y, s=8, alpha=0.6)
    ax[2].set_xticks(range(len(rows)))
    ax[2].set_xticklabels(labels)
    ax[2].set_ylabel("contact fraction per seed")
    ax[2].set_title("B: per-seed spread")
    fig.tight_layout()
    fig.savefig(PLOTS / "B_corridor.png", dpi=130)
    plt.close(fig)


def plot_distributions():
    """Speed / heading-rate distributions per model (from fresh traces)."""
    import corrections
    corrections.install(environment=True, contact=True)
    from v2.config_v2 import stage2_v2cfg
    from v2.corridor_v2 import run_corridor

    ship = json.loads((OUT / "calibration_summary.json").read_text(
        encoding="utf-8"))
    base = dict(m1_gain=ship["M1"]["gain"], m1_decay=ship["M1"]["decay"],
                m1_sense_range=6.0)
    base.update({k: v for k, v in ship["M2"].items()})
    cfgs = [("M0", stage2_v2cfg("M0", **base)),
            ("M1", stage2_v2cfg("M1", **base)),
            ("M2", stage2_v2cfg("M2", **base))]
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for name, cfg in cfgs:
        spd, om, gap = [], [], []
        for s in range(1000, 1004):
            m, res = run_corridor(s, cfg)
            tr = res["trace_states"]
            ssh = np.array([r[5] for r in tr])
            shead = np.array([r[4] for r in tr])
            opos = np.array([r[10] for r in tr])
            spos = np.array([r[3] for r in tr])
            spd.append(ssh.ravel())
            om.append((np.abs(np.angle(np.exp(1j * np.diff(shead, axis=0))))
                       / cfg_dt(cfg)).ravel())
            gap.append((np.linalg.norm(spos[:, :, None] - opos[:, None],
                                       axis=3) - 1.55).min(axis=2).ravel())
        spd = np.concatenate(spd)
        om = np.concatenate(om)
        gap = np.concatenate(gap)
        ax[0].hist(spd, bins=60, histtype="step", density=True, label=name)
        ax[1].hist(om, bins=60, histtype="step", density=True, label=name)
        ax[2].hist(np.clip(gap, -2, 12), bins=60, histtype="step", density=True,
                   label=name)
    ax[0].set_xlabel("animal speed [m/s]")
    ax[0].set_title("speed distribution")
    ax[1].set_xlabel("|dtheta|/dt [rad/s]")
    ax[1].set_title("heading-rate distribution")
    ax[2].set_xlabel("surface gap to obstacle [m] (clipped at -2/12)")
    ax[2].set_title("gap distribution")
    for a in ax:
        a.legend()
        a.set_ylabel("density")
    fig.tight_layout()
    fig.savefig(PLOTS / "B_distributions.png", dpi=130)
    plt.close(fig)


def cfg_dt(cfg):
    return 0.05


def plot_c():
    rows = read(BEH / "C_delay.csv")
    x = fnum(rows, "react_delay")
    y = fnum(rows, "latency")
    e = fnum(rows, "latency_sd")
    yp = fnum(rows, "paired_first_response")
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.6))
    ax[0].errorbar(x, y, yerr=e, marker="o", capsize=3, label="mean onset (20% threshold)")
    ax[0].plot(x, yp, marker="s", label="animal-paired first response")
    ax[0].plot(x, x + yp[0], "k--", lw=0.8, label="slope-1 reference")
    ax[0].set_xlabel("M2 reaction delay [s]")
    ax[0].set_ylabel("avoidance onset [s]")
    ax[0].set_title("C: the delay reaches the response (paired, slope 1)")
    ax[0].legend(fontsize=8)

    g = read(BEH / "C_gains.csv")
    vals = fnum(g, "gain")
    ax[1].hist(vals, bins=30, color="C2")
    ax[1].set_xlabel("per-animal response gain")
    ax[1].set_ylabel("count")
    ax[1].set_title(f"C: gain distribution (mean {vals.mean():.3f}, "
                    f"sd {vals.std(ddof=1):.3f})")
    fig.tight_layout()
    fig.savefig(PLOTS / "C_delay_and_gains.png", dpi=130)
    plt.close(fig)


def plot_policy_log():
    """r_req / v_allow during pre/divert, from a shipped M2 run."""
    import corrections
    corrections.install(environment=True, contact=True)
    from v2.config_v2 import stage2_v2cfg
    from v2.controller_v2 import unpack
    from v2.runner_v2 import run_cell

    _, res = run_cell("S3", "Ours_rollout", 1000,
                      v2cfg=stage2_v2cfg("M2"), return_res=True)
    core, pol, _ = unpack(res["controller_obj"])
    lg = [e for e in pol.policy_log if e["applied"]]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    rr = np.array([e["r_req"] for e in lg])
    va = np.array([e["v_allow"] for e in lg])
    ax[0].hist(rr[np.isfinite(rr)], bins=40, color="C1")
    ax[0].axvline(4.167, color="k", ls=":", lw=1,
                  label="V1 DIVERT geometry 4.17 m")
    ax[0].set_xlabel("required turn radius r_req [m]")
    ax[0].set_ylabel("ticks")
    ax[0].set_title("derived r_req while pre/diverting")
    ax[0].legend()
    ax[1].plot([e["t"] for e in lg], va, ".", ms=3)
    ax[1].axhline(4.5, color="k", ls="--", lw=0.8, label="v_max = 4.5")
    ax[1].axhline(2.887, color="r", ls=":", lw=1,
                  label="sqrt(a_lat*4.167) = 2.89")
    ax[1].set_xlabel("time [s]")
    ax[1].set_ylabel("commanded speed bound [m/s]")
    ax[1].set_title("bound actually applied (S3, M2)")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(PLOTS / "DIVERT_speed_policy.png", dpi=130)
    plt.close(fig)


def plot_paths():
    """x-y behaviour in the corridor: M0 walks into it, M1/M2 do not."""
    import corrections
    corrections.install(environment=True, contact=True)
    from v2.config_v2 import stage2_v2cfg
    from v2.corridor_v2 import run_corridor

    ship = json.loads((OUT / "calibration_summary.json").read_text(
        encoding="utf-8"))
    base = dict(m1_gain=ship["M1"]["gain"], m1_decay=ship["M1"]["decay"],
                m1_sense_range=6.0)
    base.update(ship["M2"])
    fig, ax = plt.subplots(1, 3, figsize=(13, 4.0), sharey=True)
    for k, (name, cfg) in enumerate([
            ("M0", stage2_v2cfg("M0", **base)),
            ("M1", stage2_v2cfg("M1", **base)),
            ("M2", stage2_v2cfg("M2", **base))]):
        m, res = run_corridor(1000, cfg)
        tr = res["trace_states"]
        spos = np.array([r[3] for r in tr])
        opos = np.array([r[10] for r in tr])
        for i in range(spos.shape[1]):
            ax[k].plot(spos[:, i, 0], spos[:, i, 1], lw=0.6, alpha=0.7)
        ax[k].plot(opos[:, 0, 0], opos[:, 0, 1], "k-", lw=2.0, label="obstacle")
        ax[k].axhline(6.0, color="gray", ls=":", lw=0.8)
        ax[k].axhline(-6.0, color="gray", ls=":", lw=0.8)
        ax[k].set_title(f"{name}: contact {m['animal_contact_fraction']:.2f}, "
                        f"mean clr {m['mean_min_clearance']:.2f} m")
        ax[k].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(PLOTS / "B_paths.png", dpi=130)
    plt.close(fig)


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)
    for fn in (plot_b, plot_c, plot_paths, plot_distributions,
               plot_policy_log):
        try:
            fn()
            print("ok", fn.__name__)
        except Exception as e:      # keep going; a missing table is not fatal
            print("skip", fn.__name__, type(e).__name__, e)


if __name__ == "__main__":
    main()
