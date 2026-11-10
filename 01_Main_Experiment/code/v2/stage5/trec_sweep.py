"""Item 2: is ``T_REC = 15 s`` still right on the 220 m geometry?

    python -X utf8 -m v2.stage5.trec_sweep --jobs 8

Rule R3 of the Stage-3 calibration (``V2_STAGE3_REPORT.md`` §6.3) is

    T_REC = T_PRE + max(h, ceil_0.5(median RECOVER duration))

derived from 7 RECOVER episodes on the V1 geometry, where the median duration was
1.10 s and the rule returned ``12 + max(3, 1.5) = 15 s``.  On the Stage-4 220 m
scene the *preempted* window is 1.0 s but the windows that actually closed are
9.5 s and 2.0 s, so the frozen 15 s has to be re-derived on windows that
**closed normally** (a window cut short by a new threat is not a recovery).

This script sweeps ``ttc_rec in {12, 15, 18}`` on the calibration seeds
1000-1004 (never the evaluation seeds), on V2S3 (220 m) and on V2S2 (140 m
control), and reports per (scene, T_REC):

* RECOVER episodes: total, closed normally, preempted, still open at the
  horizon;
* the duration distribution of the episodes that closed;
* the **re-entry delay** ``window_end - t_clear`` (``t_clear`` = first tick in
  the window at which no visible threat has TTC <= T_PRE): the quantity that a
  too-large T_REC inflates, i.e. the "does 15 s delay legitimate re-entry"
  question;
* the outcome columns (C, surface clearance, rho, arrival, switches), so a
  truncation cost would show up as a safety regression and a delay cost as a
  path-keeping / goal-time regression.

The frozen value is not changed here; the script writes the evidence *and* the
rule's own answer (``T_REC_rule``) to ``results_v2_stage5/trec_sweep/``.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import run_cell as _run_cell  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5 import probe as _probe  # noqa: E402
from v2.stage5.common import W_HALF  # noqa: E402
from v2.stage5.preempt import analyse_recover  # noqa: E402

LEVELS = [12.5, 15.0, 18.0]
# ``T_REC = 12.0`` is *inadmissible*: ``Proposed.__init__`` asserts
# ``t_rec > t_pre`` and the Stage-3 acceptance gate "T_REC > T_PRE" is part of
# the frozen tuple, so the requested "12 s" arm cannot be built at all.  12.5 s
# is the admissible low end closest to it and is what the sweep runs.
WIN_COLS = ["scene", "seed", "t_rec", "win", "start", "end", "dur", "end_tag",
            "complete", "preempted_ctrl", "truncated_ctrl", "intruder_threat",
            "t_clear", "delay"]


def _risk_series(res, v2cfg):
    """Per control tick: is any *visible* obstacle at TTC <= T_PRE?"""
    t_pre = float(v2cfg.ttc_pre)
    out = []
    for sr in (res.get("sense_log") or []):
        d = sr.get("ttc_ctrl")
        if d is None:
            out.append(False)
            continue
        out.append(any(float(v) <= t_pre for v in d.values()))
    return np.array(out, dtype=bool)


def window_detail(res, v2cfg):
    """RECOVER windows + ``t_clear`` / re-entry delay / exit reason."""
    pre = analyse_recover(res, v2cfg)
    wins = pre.get("_windows") or []
    risk = _risk_series(res, v2cfg)
    ts = np.array([float(l["t"]) for l in (res.get("logs") or [])])
    out = []
    for iw, w in enumerate(wins):
        i0 = int(np.searchsorted(ts, w["start"] - 1e-9))
        i1 = int(np.searchsorted(ts, w["end"] + 1e-9)) - 1
        i1 = min(max(i1, i0), len(risk) - 1)
        clear = None
        for i in range(i0, i1 + 1):
            if i < len(risk) and not risk[i]:
                clear = float(ts[i])
                break
        ic = w.get("intruder_ctrl")
        out.append(dict(win=iw, start=w["start"], end=w["end"], dur=w["dur"],
                        end_tag=w["end_tag"],
                        complete=int(bool(w["complete"])),
                        preempted_ctrl=int(bool(w["preempted_ctrl"])),
                        truncated_ctrl=int(bool(w["truncated_ctrl"])),
                        intruder_threat=(int(ic[1]) + 1 if ic else -1),
                        t_clear=(clear if clear is not None else np.nan),
                        delay=(w["end"] - clear) if clear is not None
                        else np.nan))
    return pre, out


def run_case(scene, seed, t_rec, method="Ours_rollout"):
    v2cfg = stage3_v2cfg("M1").with_(ttc_rec=float(t_rec))
    with _probe.installed():
        row, res = _run_cell(scene, method, seed, v2cfg=v2cfg, return_res=True)
    pre, wins = window_detail(res, v2cfg)
    run = dict(scene=scene, seed=seed, t_rec=float(t_rec),
               config_id=row.get("config_id"),
               C=row.get("collision_rate"),
               clr_so=row.get("min_surface_clearance_so"),
               ttc_min=row.get("ttc_min"),
               rho=row.get("rho_avg"),
               devW=float(row.get("max_dev") or np.nan) / W_HALF,
               arrival=int(bool(row.get("success"))),
               exec_time=row.get("exec_time"),
               switch=row.get("switch_count"),
               time_in_recover=row.get("dur_recover"),
               ended=row.get("terminated_by"),
               n_windows=pre.get("n_recover_windows"),
               n_complete=pre.get("n_recover_complete"),
               n_preempted=pre.get("n_recover_preempted"),
               run_error="")
    for w in wins:
        w.update(scene=scene, seed=seed, t_rec=float(t_rec))
    return run, wins


def _task(t):
    return run_case(*t)


def _mean(vals):
    v = [float(x) for x in vals
         if x is not None and x != "" and np.isfinite(float(x))]
    return float(np.mean(v)) if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=["V2S3", "V2S2"])
    ap.add_argument("--seeds", nargs="+", type=int, default=common.CALIB_SEEDS)
    ap.add_argument("--levels", nargs="+", type=float, default=LEVELS)
    ap.add_argument("--jobs", type=int, default=8)
    args = ap.parse_args()
    common.ensure_dirs()
    out = common.TREC_DIR
    tasks = [(s, sd, tr) for s in args.scenes for tr in args.levels
             for sd in args.seeds]
    print(f"T_REC sweep: {len(tasks)} runs "
          f"({args.scenes} x {args.levels} x seeds {args.seeds}), "
          f"jobs={args.jobs}", flush=True)
    t0 = time.time()
    if args.jobs > 1:
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            res = list(ex.map(_task, tasks, chunksize=1))
    else:
        res = [_task(t) for t in tasks]
    wall = time.time() - t0
    runs = [r for r, _ in res]
    wins = [w for _, ws in res for w in ws]
    runs.sort(key=lambda r: (r["scene"], r["t_rec"], r["seed"]))
    wins.sort(key=lambda w: (w["scene"], w["t_rec"], w["seed"], w["win"]))
    with open(out / "trec_sweep_runs.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(runs)
    with open(out / "trec_windows.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=WIN_COLS, extrasaction="ignore")
        w.writeheader()
        w.writerows(wins)

    # ---- aggregate per (scene, T_REC) ---------------------------------
    agg = []
    for s in args.scenes:
        for tr in args.levels:
            sub = [r for r in runs if r["scene"] == s and r["t_rec"] == tr]
            wsub = [w for w in wins if w["scene"] == s and w["t_rec"] == tr]
            closed = [w for w in wsub if w["complete"]]
            durs = [w["dur"] for w in closed]
            delays = [w["delay"] for w in closed if np.isfinite(w["delay"])]
            preempt = [w for w in wsub if w["preempted_ctrl"]]
            agg.append(dict(
                scene=s, t_rec=tr, n_runs=len(sub), n_windows=len(wsub),
                n_closed=len(closed), n_preempted=len(preempt),
                n_open_at_end=sum(1 for w in wsub if w["end_tag"] == "end"),
                dur_med=(float(np.median(durs)) if durs else float("nan")),
                dur_max=(float(np.max(durs)) if durs else float("nan")),
                delay_med=(float(np.median(delays)) if delays
                           else float("nan")),
                delay_max=(float(np.max(delays)) if delays
                           else float("nan")),
                C=_mean([r["C"] for r in sub]),
                clr_so=_mean([r["clr_so"] for r in sub]),
                ttc_min=_mean([r["ttc_min"] for r in sub]),
                rho=_mean([r["rho"] for r in sub]),
                devW=_mean([r["devW"] for r in sub]),
                arrival=_mean([r["arrival"] for r in sub]),
                exec_time=_mean([r["exec_time"] for r in sub]),
                switch=_mean([r["switch"] for r in sub]),
                time_in_recover=_mean([r["time_in_recover"] for r in sub]),
                n_censored_runs=sum(1 for r in sub if not r["arrival"]),
                config_id=sub[0]["config_id"]))
    with open(out / "trec_sweep_summary.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(agg[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(agg)

    # ---- what does rule R3 itself say, on windows that actually closed? --
    closed_s3 = [w for w in wins if w["scene"] == "V2S3" and w["t_rec"] == 15.0
                 and w["complete"]]
    durs = [w["dur"] for w in closed_s3]
    med = float(np.median(durs)) if durs else float("nan")
    # round to 10 ms before the 0.5 s ceiling: the durations are sums of 0.1 s
    # control ticks, so a raw median of 3.0000000000002416 must not ceil to 3.5
    med_r = float(np.round(med, 2)) if np.isfinite(med) else med
    rule = dict(t_pre=12.0, h=3.0, n_closed_episodes=len(durs),
                median_closed_duration=med,
                median_closed_duration_rounded=med_r,
                t_rec_rule=(12.0 + max(3.0, math.ceil(med_r * 2.0) / 2.0))
                if np.isfinite(med_r) else float("nan"),
                t_rec_frozen=15.0)
    common.write_json(out / "trec_rule.json", dict(
        rule=rule, aggregate=agg, levels=args.levels, scenes=args.scenes,
        seeds=args.seeds, wall_s=wall))

    hdr = (f"{'scene':6s}{'T_REC':>6s}{'win':>5s}{'closed':>7s}{'preempt':>8s}"
           f"{'open':>5s}{'medDur':>8s}{'maxDur':>8s}{'delayMed':>9s}"
           f"{'C':>8s}{'clr':>7s}{'rho':>7s}{'devW':>7s}{'arriv':>6s}"
           f"{'exec':>7s}{'switch':>7s}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in agg:
        print(f"{r['scene']:6s}{r['t_rec']:>6.0f}{r['n_windows']:>5d}"
              f"{r['n_closed']:>7d}{r['n_preempted']:>8d}"
              f"{r['n_open_at_end']:>5d}{r['dur_med']:>8.2f}"
              f"{r['dur_max']:>8.2f}{r['delay_med']:>9.2f}{r['C']:>8.4f}"
              f"{r['clr_so']:>7.2f}{r['rho']:>7.3f}{r['devW']:>7.3f}"
              f"{r['arrival']:>6.2f}{r['exec_time']:>7.1f}"
              f"{r['switch']:>7.1f}")
    print(f"\nrule R3 on the closed windows of V2S3 @ T_REC=15: "
          f"{len(durs)} episodes, median {med:.2f} s -> T_REC_rule "
          f"{rule['t_rec_rule']:.1f} s (frozen: 15.0 s)")
    print(f"wrote {out}  ({len(runs)} runs, {len(wins)} windows, {wall:.1f} s)")


if __name__ == "__main__":
    main()
