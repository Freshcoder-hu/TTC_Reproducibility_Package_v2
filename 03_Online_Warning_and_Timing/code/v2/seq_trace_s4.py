"""Dump the state / risk timeline of a sequential-obstacle run (test D).

    python -X utf8 -m v2.seq_trace_s4 --scenario V2S3 --seed 1000 \
        --alg Ours_rollout

Writes ``results_v2_stage4/geometry/seq_trace_<scenario>_<alg>_<seed>.csv`` with
one row per control tick:

    t, tag, rho, lat_c, n_obs_visible, ttc_o1..o3, sense_detect_1..3

and prints the state transitions plus, for each RECOVER window, whether a *new*
threat's risk onset (TTC_k <= T_PRE) fell inside it.  That is the evidence
behind acceptance test D ("a new threat arrives while RECOVER is still
active") and behind the release-ordering verification (the order of the
per-obstacle first-detection and risk-onset times must match the designed
release order in every seed, not just on average).
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import run_cell  # noqa: E402
from v2.scenarios_v2 import resolve_scene  # noqa: E402

OUT = ROOT / "results_v2_stage4" / "geometry"


def control_rows(res, v2cfg):
    """Per-control-tick risk/state table from the V1 log."""
    rows = []
    for lg in res["logs"]:
        ttc = lg.get("ttc_each") or []
        row = dict(t=float(lg["t"]), tag=lg.get("tag", "normal"),
                   rho=float(lg.get("rho", np.nan)),
                   lat_c=float(lg.get("lat", np.nan)),
                   n_obs_visible=np.nan, ttc_min=float(lg.get("ttc", np.nan)))
        for k in range(3):
            row[f"ttc_o{k + 1}"] = float(ttc[k]) if k < len(ttc) else np.nan
        rows.append(row)
    # visibility per obstacle from the Stage-3 sense log (track = obstacle idx)
    senses = res.get("sense_log") or []
    ts = np.array([r["t"] for r in rows], dtype=float)
    for k in range(3):
        vis = np.full(len(rows), np.nan)
        for sr in senses:
            i = int(np.argmin(np.abs(ts - float(sr["t"]))))
            vis[i] = 1.0 if k in sr.get("obs_index", []) else 0.0
        if np.isfinite(vis).any():
            first = ts[np.where(vis > 0)[0][0]] if (vis > 0).any() else np.nan
            for r in rows:
                r[f"visible_o{k + 1}"] = vis[int(np.argmin(np.abs(
                    ts - r["t"])))]
                r[f"first_detect_o{k + 1}"] = float(first)
        else:
            for r in rows:
                r[f"visible_o{k + 1}"] = np.nan
                r[f"first_detect_o{k + 1}"] = np.nan
    return rows


def analyse(rows, v2cfg):
    """Recover windows + which threat's risk onset lands inside them."""
    ts = np.array([r["t"] for r in rows], dtype=float)
    tags = [r["tag"] for r in rows]
    t_pre, t_div = float(v2cfg.ttc_pre), float(v2cfg.ttc_divert)
    # contiguous RECOVER windows
    win, start = [], None
    for i, tg in enumerate(tags):
        if tg == "recover" and start is None:
            start = ts[i]
        elif tg != "recover" and start is not None:
            win.append((float(start), float(ts[i - 1])))
            start = None
    if start is not None:
        win.append((float(start), float(ts[-1])))
    # risk onset (TTC_k <= T_PRE) **as the controller can see it**, i.e. gated
    # by visibility: the log's ``ttc_each`` is computed on the *truth* obstacle
    # positions (simulator truth is reserved for assessment), so an obstacle
    # that has not been detected yet must not contribute an earlier onset.
    onset, detect = {}, {}
    for k in range(3):
        col = np.array([r[f"ttc_o{k + 1}"] for r in rows], dtype=float)
        vis = np.array([r[f"visible_o{k + 1}"] for r in rows], dtype=float)
        seen = np.isfinite(vis) & (vis > 0)
        hit = np.where(seen & np.isfinite(col) & (col <= t_pre))[0]
        onset[k + 1] = float(ts[hit[0]]) if len(hit) else np.nan
        hit = np.where(seen & np.isfinite(col) & (col <= t_div))[0]
        onset[f"divert_{k + 1}"] = float(ts[hit[0]]) if len(hit) else np.nan
        fd = [r[f"first_detect_o{k + 1}"] for r in rows]
        fd = [x for x in fd if np.isfinite(x)]
        detect[k + 1] = float(fd[0]) if fd else np.nan
    # overlap test, two readings of "a new threat arrives during RECOVER":
    #   onset  -- the new threat's *risk onset* (TTC_k <= T_PRE) falls inside a
    #             RECOVER window of an earlier threat (the strict reading:
    #             the controller's own warning threshold fires while in RECOVER)
    #   detect -- the new threat is *detected* (enters the 70 m sensing horizon)
    #             while the controller is in RECOVER (the observable-threat
    #             reading).  Both are reported; they differ by the TTC
    #             detect->onset lag (a few seconds).
    overlap, overlap_detect = [], []
    for k in range(2, 4):
        for (a, b) in win:
            t_on = onset[k]
            if np.isfinite(t_on) and a <= t_on <= b:
                overlap.append(dict(threat=k, recover_window=(a, b),
                                    t_onset=float(t_on)))
            t_det = detect[k]
            if np.isfinite(t_det) and a <= t_det <= b:
                overlap_detect.append(dict(threat=k, recover_window=(a, b),
                                           t_detect=float(t_det)))
    return dict(recover_windows=win, risk_onset=onset, first_detect=detect,
                overlap=overlap, overlap_detect=overlap_detect,
                order_ok=bool(np.isfinite([onset[1], onset[2], onset[3]]).all()
                              and onset[1] < onset[2] < onset[3]),
                detect_order_ok=bool(np.isfinite(
                    [detect[1], detect[2], detect[3]]).all()
                    and detect[1] < detect[2] < detect[3]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="V2S3")
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--alg", default="Ours_rollout")
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args()
    v2cfg = stage3_v2cfg("M1")
    row, res = run_cell(args.scenario, args.alg, args.seed, v2cfg=v2cfg,
                        return_res=True)
    rows = control_rows(res, v2cfg)
    info = analyse(rows, v2cfg)
    OUT.mkdir(parents=True, exist_ok=True)
    if not args.no_write:
        k = list(rows[0])
        p = OUT / f"seq_trace_{args.scenario}_{args.alg}_{args.seed}.csv"
        with open(p, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=k, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"wrote {p}")
    print(f"\n{args.scenario} / {args.alg} / seed {args.seed}: "
          f"{len(rows)} control ticks, ended {res['terminated_by']}, "
          f"C={row['collision_rate']:.3f}, min_dist={row['min_dist']:.2f} m, "
          f"rho={row['rho_avg']:.3f}")
    print(f"  first detection per obstacle: "
          + ", ".join(f"o{k}={info['first_detect'][k]:.2f}"
                      for k in (1, 2, 3)))
    print(f"  risk onset (TTC<=T_PRE) per obstacle: "
          + ", ".join(f"o{k}={info['risk_onset'][k]:.2f}"
                      for k in (1, 2, 3))
          + f"   ordered={info['order_ok']}")
    print(f"  RECOVER windows: "
          + (", ".join(f"[{a:.2f}, {b:.2f}] ({b - a:.2f} s)"
                       for a, b in info["recover_windows"]) or "none"))
    print(f"  new threat's RISK ONSET inside a RECOVER window: "
          + (", ".join(f"o{o['threat']} at {o['t_onset']:.2f} s in "
                       f"[{o['recover_window'][0]:.2f}, "
                       f"{o['recover_window'][1]:.2f}]"
                       for o in info["overlap"]) or "NO"))
    print(f"  new threat DETECTED inside a RECOVER window: "
          + (", ".join(f"o{o['threat']} at {o['t_detect']:.2f} s in "
                       f"[{o['recover_window'][0]:.2f}, "
                       f"{o['recover_window'][1]:.2f}]"
                       for o in info["overlap_detect"]) or "NO"))
    print("\n  state timeline (tag changes and risk-onset ticks):")
    prev = None
    for i, r in enumerate(rows):
        changed = (r["tag"] != prev)
        mark = ""
        for k in (1, 2, 3):
            if abs(r[f"ttc_o{k}"] - float(v2cfg.ttc_pre)) < 0.35 or \
                    (np.isfinite(r[f"first_detect_o{k}"])
                     and abs(r["t"] - r[f"first_detect_o{k}"]) < 1e-6):
                mark += f" <o{k}>"
        if changed or mark:
            print(f"    t={r['t']:7.2f}  tag={r['tag']:<8s}"
                  f"rho={r['rho']:.3f} lat={r['lat_c']:+.2f} "
                  f"ttc=({r['ttc_o1']:6.2f},{r['ttc_o2']:6.2f},"
                  f"{r['ttc_o3']:6.2f}){mark}")
        prev = r["tag"]


if __name__ == "__main__":
    main()
