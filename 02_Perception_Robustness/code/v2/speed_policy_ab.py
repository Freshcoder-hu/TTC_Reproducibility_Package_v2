"""A/B of the layered speed policy on the real scenario (S3), same seeds.

    python -X utf8 -m v2.speed_policy_ab --seeds 1000 1001 --model M2

Compares ``speed_policy="off"`` with ``"manoeuvre"`` and reports what the rule
does to the *commanded* and the *executed* speed in PRE/DIVERT, plus the
downstream robot/animal separation numbers.  This is the "observed effect on
DIVERT speed" table of the Stage-2 report; it is not an acceptance test.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage2_v2cfg  # noqa: E402
from v2.controller_v2 import unpack  # noqa: E402
from v2.metrics_v2 import summarize_v2  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage2" / "behaviour"

KEYS = ["mean_cmd_divert", "mean_speed_divert", "max_speed_divert",
        "sp_v_allow_mean", "sp_v_allow_min", "sp_r_req_mean", "sp_n_clamped_robots",
        "n_contact_events_ro", "min_surface_clearance_ro",
        "n_contact_events_rs", "min_surface_clearance_rs",
        "n_contact_events_rr", "min_surface_clearance_rr",
        "collision_rate", "min_dist", "rho_avg", "max_dev", "success",
        "dur_divert", "switch_count"]


def divert_speeds(res, cfg):
    """Mean commanded / executed speed over the PRE+DIVERT ticks."""
    ctrls = res.get("controller_obj")
    core, pol, _ = unpack(ctrls)
    logs = res["logs"]
    tags = [e["tag"] for e in logs]
    n_div = sum(1 for t in tags if t in ("pre", "divert"))
    # executed robot speed: from the trace is not recorded here, so use the
    # controller-side log for the bound and the metrics for the outcomes
    v_allow = np.nan
    if pol is not None:
        va = [e["v_allow"] for e in pol.policy.log if e["applied"]]
        v_allow = float(np.mean(va)) if va else np.nan
    return dict(n_div_ticks=n_div, v_allow_mean_log=v_allow)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", nargs="+", type=int, default=[1000, 1001])
    ap.add_argument("--model", default="M2")
    ap.add_argument("--scenario", default="S3")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    rows = []
    for policy in ("off", "manoeuvre"):
        for seed in args.seeds:
            cfg = stage2_v2cfg(args.model, speed_policy=policy)
            row, res = run_cell(args.scenario, "Ours_rollout", seed, v2cfg=cfg,
                                return_res=True, record_states=True)
            m = summarize_v2(res, cfg)
            tr = res["trace_states"]
            spd = np.array([r[8] for r in tr])        # robot speed per tick
            tags = [e["tag"] for e in res["logs"]]
            n_div = sum(1 for t in tags if t in ("pre", "divert"))
            # robot speed during divert ticks: logs are per control tick, the
            # trace is per integration tick -> take every ctrl_period-th sample
            mask = np.zeros(len(spd), dtype=bool)
            cp = 2
            for k, tg in enumerate(tags):
                if tg in ("pre", "divert"):
                    idx = min(k * cp, len(spd) - 1)
                    mask[max(0, idx - cp + 1):idx + 1] = True
            mean_div = float(spd[mask].mean()) if mask.any() else np.nan
            v2 = res["v2"]
            rows.append(dict(
                policy=policy, seed=seed, model=args.model,
                mean_speed_divert=mean_div,
                max_speed_divert=float(spd[mask].max()) if mask.any() else np.nan,
                mean_speed_all=float(spd.mean()),
                n_div_ticks=n_div,
                sp_v_allow_mean=v2.get("sp_v_allow_mean", np.nan),
                sp_v_allow_min=v2.get("sp_v_allow_min", np.nan),
                sp_r_req_mean=v2.get("sp_r_req_mean", np.nan),
                sp_n_clamped_robots=v2.get("sp_n_clamped_robots", 0),
                **{k: m.get(k) for k in KEYS if k not in (
                    "mean_speed_divert", "max_speed_divert", "sp_v_allow_mean",
                    "sp_v_allow_min", "sp_r_req_mean", "sp_n_clamped_robots")}))
            print(f"{policy:10s} seed {seed}: divert speed "
                  f"{mean_div:.3f} (max {rows[-1]['max_speed_divert']:.3f}), "
                  f"v_allow {rows[-1]['sp_v_allow_mean']}, "
                  f"ro contacts {rows[-1]['n_contact_events_ro']}, "
                  f"worst ro clr {rows[-1]['min_surface_clearance_ro']}, "
                  f"animal collision rate {rows[-1]['collision_rate']:.3f}",
                  flush=True)
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(OUT / "speed_policy_AB.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    # paired summary
    off = [r for r in rows if r["policy"] == "off"]
    on = [r for r in rows if r["policy"] == "manoeuvre"]
    summ = {}
    for k in ("mean_speed_divert", "n_contact_events_ro",
              "min_surface_clearance_ro", "min_surface_clearance_rs",
              "collision_rate", "rho_avg", "max_dev"):
        summ[k] = dict(off=float(np.nanmean([float(r[k]) for r in off])),
                       on=float(np.nanmean([float(r[k]) for r in on])))
    (OUT / "speed_policy_AB.json").write_text(
        json.dumps(summ, indent=2), encoding="utf-8")
    print(json.dumps(summ, indent=2))


if __name__ == "__main__":
    main()
