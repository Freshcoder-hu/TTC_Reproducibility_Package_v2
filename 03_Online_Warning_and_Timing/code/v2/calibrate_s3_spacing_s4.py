"""Calibrate the S3 sequential spacing (the "during RECOVER" gap).

The Stage-4 brief requires the 220 m S3 to contain **both** a
recovery-complete spacing and a threat-arrival-during-recovery spacing — the
case the V1 110 m geometry could not create.  This script measures, for a grid
of second-release encounter points, in how many seeds a *new* threat's risk
onset falls inside a RECOVER window of an earlier threat, and how often the
first two threats instead merge into one continuous DIVERT.

    python -X utf8 -m v2.calibrate_s3_spacing_s4 --seeds 1000 1001 1002
    python -X utf8 -m v2.calibrate_s3_spacing_s4 --e2 85 87 89 91 --seeds 1000 1001 1002 1003 1004 1005

The chosen spacing is then written into ``v2/scenarios_v2.py`` (``_S3_PLAN``) as
a literal, and the *held-out* calibration seeds are re-checked by
``v2.acceptance_s4`` test D.  Tuning is confined to calibration seeds
1000-1019; evaluation seeds 2000-2029 are never touched.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from path import straight_path  # noqa: E402

from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import run_cell  # noqa: E402
from v2.scenarios_v2 import (GRAZE, L_S3, R_OBS, R_SENSE, T_ENC_MIN_FACTOR,
                             U_BACK_LEFT, U_BACK_RIGHT, W_V2, _base_cfg,
                             release, t_arr)  # noqa: E402
from v2.seq_trace_s4 import analyse, control_rows  # noqa: E402

OUT = ROOT / "results_v2_stage4" / "geometry"


def s3_variant(e2, e3=145.0, lat=GRAZE, speed=(3.0, 3.0, 3.0), t_max=None,
               u=None):
    """S3 with the second encounter point / speed / angle moved."""
    u = u or (U_BACK_LEFT, U_BACK_RIGHT, U_BACK_LEFT)
    speed = tuple(float(x) for x in speed)
    path = straight_path(0.0, L_S3, halfwidth=W_V2)
    plan = [(70.0, +lat, u[0], speed[0]), (e2, -lat, u[1], speed[1]),
            (e3, +lat, u[2], speed[2])]
    obs, recs = [], []
    for i, (e_s, lateral, ui, v) in enumerate(plan, start=1):
        t_enc = T_ENC_MIN_FACTOR * R_SENSE / v
        t_spawn = t_arr(e_s, L_S3) - t_enc
        o, rec = release(path, e_s, lateral, ui, v, t_enc,
                         t_spawn=t_spawn, name=f"o{i}")
        obs.append(o)
        recs.append(rec)
    return dict(name=f"V2S3_e2_{e2:.0f}_v{'-'.join(f'{x:g}' for x in speed)}",
                path=path, obstacles=obs,
                cfg=_base_cfg(), T_max=t_max or 320.0, s0=8.0,
                params=dict(divert_shift=3.0), design=recs)


def run_variant(scn, seed, alg="Ours_rollout", v2cfg=None, sid="V2S3"):
    """Run a hand-built variant through the Stage-4 runner machinery."""
    from v2.runner_s4 import ALGS
    from v2.sim_v2 import run_scene_v2
    from v2.metrics_v2 import summarize_v2
    from v2.runner_v2 import _needs_stage2_stack
    from v2.scenarios_v2 import cfg_for
    v2cfg = v2cfg or stage3_v2cfg("M1")
    cfg = cfg_for(scn, v2cfg)
    if _needs_stage2_stack(v2cfg):
        from v2.controller_v2 import make_controller
        ctrl = make_controller(alg, ALGS, v2cfg, dict(scn.get("params", {})))
    else:
        ctrl = ALGS[alg](**dict(scn.get("params", {})))
    res = run_scene_v2(cfg, scn["path"], scn["obstacles"], ctrl, seed=seed,
                       T_max=scn["T_max"], s0=scn["s0"], v2cfg=v2cfg,
                       contact=True, feasibility=True)
    return summarize_v2(res, v2cfg), res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--e2", nargs="+", type=float, default=[85.0, 87.0, 89.0,
                                                            91.0])
    ap.add_argument("--seeds", nargs="+", type=int,
                    default=[1000, 1001, 1002])
    ap.add_argument("--alg", default="Ours_rollout")
    ap.add_argument("--e3", type=float, default=145.0)
    ap.add_argument("--v2-speed", type=float, default=3.0,
                    help="speed of the SECOND obstacle (threat-during-recovery)")
    ap.add_argument("--divert-shift", type=float, default=3.0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    v2cfg = stage3_v2cfg("M1")
    speeds = (3.0, float(args.v2_speed), 3.0)
    rows = []
    summary = []
    for e2 in args.e2:
        scn = s3_variant(e2, e3=args.e3, speed=speeds)
        n_over = 0
        n_over_det = 0
        for seed in args.seeds:
            m, res = run_variant(scn, seed, args.alg, v2cfg)
            cr = control_rows(res, v2cfg)
            info = analyse(cr, v2cfg)
            over = info["overlap"]
            over_det = info["overlap_detect"]
            n_over += 1 if over else 0
            n_over_det += 1 if over_det else 0
            rows.append(dict(
                e2=e2, e3=args.e3, divert_shift=args.divert_shift, seed=seed,
                alg=args.alg,
                recover_windows=";".join(f"{a:.2f}-{b:.2f}"
                                         for a, b in info["recover_windows"]),
                n_recover=len(info["recover_windows"]),
                detect_1=info["first_detect"][1], detect_2=info["first_detect"][2],
                detect_3=info["first_detect"][3],
                onset_1=info["risk_onset"][1], onset_2=info["risk_onset"][2],
                onset_3=info["risk_onset"][3],
                order_ok=info["order_ok"], detect_order_ok=info["detect_order_ok"],
                overlap=bool(over),
                overlap_detect=bool(over_det),
                overlap_desc=";".join(f"o{o['threat']}@{o['t_onset']:.2f}"
                                      for o in over),
                overlap_detect_desc=";".join(
                    f"o{o['threat']}@{o['t_detect']:.2f}" for o in over_det),
                collision_rate=m.get("collision_rate"),
                min_dist=m.get("min_dist"), rho_avg=m.get("rho_avg"),
                success=m.get("success"), ended=res["terminated_by"]))
            print(f"e2={e2:5.1f} ds={args.divert_shift:.1f} seed={seed}  "
                  f"recover={rows[-1]['recover_windows']:34s}"
                  f" onset2={rows[-1]['onset_2']:.1f} "
                  f"det2={rows[-1]['detect_2']:.1f}  "
                  f"over(onset)={bool(over)} over(det)={bool(over_det)}"
                  f"  C={rows[-1]['collision_rate']:.3f}"
                  f"  min_dist={rows[-1]['min_dist']:.2f}", flush=True)
        summary.append(dict(e2=e2, e3=args.e3,
                            divert_shift=args.divert_shift,
                            seeds=list(args.seeds),
                            n_overlap=n_over, n_overlap_detect=n_over_det,
                            n=len(args.seeds)))
        print(f"  -> e2={e2:.1f}: onset-overlap {n_over}/{len(args.seeds)}, "
              f"detect-overlap {n_over_det}/{len(args.seeds)}", flush=True)
    if rows:
        keys = list(rows[0])
        with open(OUT / "s3_spacing_sweep.csv", "w", newline="",
                  encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
    (OUT / "s3_spacing_sweep.json").write_text(
        json.dumps(dict(summary=summary, rows=rows), indent=2),
        encoding="utf-8")
    print(f"\nwrote {OUT / 's3_spacing_sweep.csv'}")


if __name__ == "__main__":
    main()
