"""Stage-4 batch runner: the Stage-3 runner with a scenario resolver.

``v2/runner_v2.run_cell`` resolves its scene through the **V1** registry
(``scenarios.SCENARIOS``), which is exactly right for Stages 1-3 and exactly
wrong for Stage 4.  This module keeps the same plumbing (``make_cfg``,
``_needs_stage2_stack``, ``make_controller``, ``run_scene_v2``, ``summarize_v2``)
and only changes *scene resolution*, so:

* a Stage-4 id (``V2S3`` ...) resolves through ``v2.scenarios_v2``;
* a V1 id (``S3`` ...) still resolves through the frozen V1 ``scenarios``
  module — which is how acceptance test A pushes the *old V1 scene definitions*
  through the *Stage-4 runner* and demands bit equality with the Stage-3
  trajectory.

Everything else is untouched: the plant, the sensing interface, the controller
stack and the metrics are the accepted Stage-2/3 code.

    python -X utf8 -m v2.runner_s4 --scenarios V2S2 V2S3 --seeds 3 \
        --seed-offset 1000 --algs Ours_rollout B2 --jobs 4 \
        --out results_v2_stage4/pilot

Team size is a scene property in the fixed-width S4 family, so the runner
mirrors the scene's ``n_sheep``/``n_robots`` into the configuration record
before the run; the resulting ``config_id`` therefore identifies the scale.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

import run_revised_experiments  # noqa: E402,F401  (registers Ours_rollout)
from run_experiments import ALGS, KEYS as V1_KEYS  # noqa: E402

from v2.config_v2 import (V2Cfg, compat_v1cfg, default_v2cfg,  # noqa: E402
                          stage3_v2cfg)
from v2.metrics_v2 import V2_KEYS, summarize_v2  # noqa: E402
from v2.runner_v2 import _needs_stage2_stack, make_cfg  # noqa: E402
from v2.scenarios_v2 import cfg_for, resolve_scene  # noqa: E402
from v2.sim_v2 import run_scene_v2  # noqa: E402

ALL_KEYS = list(V1_KEYS) + [k for k in V2_KEYS if k not in V1_KEYS]
# Stage-4 extras needed for the censoring / sequential-recovery report (they
# exist in ``metrics.summarize``'s base row but are not part of the V1/V2
# column lists, which are frozen for the archived tables).
S4_EXTRA = ["terminated_by", "timeout", "success_t", "n_ctrl_ticks",
            "sim_time_s", "dur_recover", "recovery_events",
            "recovery_success_rate", "event_risk_on_1", "event_risk_on_2",
            "event_risk_on_3", "event_clear_1", "dt12", "dt23",
            "event_order_ok", "recover_start_1", "preempt_ok", "ttc0"]
ALL_KEYS = ALL_KEYS + S4_EXTRA
HEADER = ["config_id", "geometry", "scenario", "scenario_name", "alg", "seed"]
OUT_ROOT = ROOT / "results_v2_stage4"


# ---------------------------------------------------------------------------
def registry_alg(alg):
    """Register the event controller (the rollout-free V2 variant) on demand.

    ``proposed_event.EventProposed`` is the manuscript's "event controller
    without rollout" (``Ours_event``); it exists in the V1 tree but is not in
    ``run_experiments.ALGS``, which only registers the methods the V1 paper
    compares.  Adding it to the *runtime* dictionary (never to the file) is the
    same mechanism ``run_revised_experiments`` uses for ``Ours_rollout``.
    """
    if alg == "Ours_event" and "Ours_event" not in ALGS:
        from proposed_event import EventProposed

        ALGS["Ours_event"] = lambda **kw: EventProposed(
            **{k: v for k, v in kw.items()
               if k in ("divert_shift", "pre_shift_ratio", "t_pre",
                        "t_divert", "t_rec", "hyst")})
    return alg


def scale_cfg(scn, v2cfg):
    """Mirror the scene's team size into the configuration record."""
    ns = int(scn["cfg"].n_sheep)
    nr = int(scn["cfg"].n_robots)
    if (v2cfg.n_sheep, v2cfg.n_robots) != (ns, nr):
        v2cfg = replace(v2cfg, n_sheep=ns, n_robots=nr)
    return v2cfg


def run_cell(sid, alg, seed, v2cfg=None, compat=False, contact=True,
             projection="joint", return_res=False, record_states=False,
             controller=None, t_max=None):
    """One (scene, method, seed) cell of the Stage-4 geometry."""
    registry_alg(alg)
    v2cfg = v2cfg or (compat_v1cfg() if compat else stage3_v2cfg("M1"))
    scn = resolve_scene(sid)
    if not compat:
        v2cfg = scale_cfg(scn, v2cfg)
    cfg = cfg_for(scn, v2cfg, compat)
    params = dict(scn.get("params", {}))
    if controller is not None:
        ctrl = controller
    elif _needs_stage2_stack(v2cfg):
        from v2.controller_v2 import make_controller
        ctrl = make_controller(alg, ALGS, v2cfg, params,
                               shadow=v2cfg.predictor_shadow)
    else:
        ctrl = ALGS[alg](**params)
    res = run_scene_v2(cfg, scn["path"], scn["obstacles"], ctrl, seed=seed,
                       T_max=float(t_max or scn["T_max"]), s0=scn["s0"],
                       v2cfg=v2cfg, contact=contact, feasibility=not compat,
                       projection=projection, record_states=record_states)
    res["geometry"] = "v2" if str(sid).startswith("V2") else "v1"
    m = summarize_v2(res, v2cfg)
    m["n_ctrl_ticks"] = len(res["logs"])
    m["sim_time_s"] = float(res["logs"][-1]["t"]) if res["logs"] else 0.0
    row = dict(config_id=v2cfg.config_id, geometry=res["geometry"],
               scenario=sid, scenario_name=scn["name"], alg=alg, seed=seed)
    for k in ALL_KEYS:
        row[k] = m.get(k, np.nan)
    if return_res:
        return row, res
    return row


def _task_entry(task):
    return run_cell(*task["args"], **task["kw"])


# ---------------------------------------------------------------------------
def write_rows(path, rows, header=None):
    header = header or (["config_id", "geometry", "scenario", "scenario_name",
                         "alg", "seed"] + ALL_KEYS)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r.get(k, np.nan) for k in header])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=["V2S2"])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seed-offset", type=int, default=1000)
    ap.add_argument("--algs", nargs="+", default=["Ours_rollout"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--out", default=str(OUT_ROOT / "runs"))
    ap.add_argument("--compat", action="store_true",
                    help="V1 archive configuration (Stage-4 test A)")
    ap.add_argument("--projection", default="joint",
                    choices=["joint", "sequential"])
    ap.add_argument("--t-max", type=float, default=None,
                    help="override the scene horizon [s]")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE",
                    help="override V2Cfg fields")
    args = ap.parse_args()

    def _parse(v):
        if v.lower() in ("true", "false"):
            return v.lower() == "true"
        for fn in (int, float):
            try:
                return fn(v)
            except ValueError:
                pass
        return tuple(v.split(",")) if "," in v else v

    over = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        over[k] = _parse(v)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    base = compat_v1cfg() if args.compat else stage3_v2cfg("M1")
    v2cfg = replace(base, **over) if over else base
    for a in args.algs:
        registry_alg(a)
    tasks = [dict(args=(sid, alg, args.seed_offset + i),
                  kw=dict(v2cfg=v2cfg, compat=args.compat,
                          projection=args.projection, t_max=args.t_max))
             for sid in args.scenarios for alg in args.algs
             for i in range(args.seeds)]

    t0 = time.time()
    print(f"Stage-4 runner: scenarios={args.scenarios} algs={args.algs} "
          f"seeds={args.seeds}@{args.seed_offset} jobs={args.jobs} "
          f"compat={args.compat}")
    print(f"config_id={v2cfg.config_id}")
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            rows = list(ex.map(_task_entry, tasks))
    else:
        rows = [_task_entry(t) for t in tasks]

    path = out / ("detail_compat.csv" if args.compat else "detail.csv")
    write_rows(path, rows)
    (out / "run_meta.json").write_text(json.dumps(dict(
        config_id=v2cfg.config_id, compat=bool(args.compat),
        projection=args.projection, scenarios=args.scenarios,
        algs=args.algs, seeds=args.seeds, seed_offset=args.seed_offset,
        jobs=args.jobs, t_max=args.t_max, wall_s=time.time() - t0,
        python=sys.version.split()[0], numpy=np.__version__), indent=2),
        encoding="utf-8")
    print(f"wrote {path}  ({len(rows)} rows, {time.time() - t0:.1f} s)")


if __name__ == "__main__":
    main()
