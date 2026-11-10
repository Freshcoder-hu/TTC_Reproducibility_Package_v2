"""Batch runner for V2 Stage 1 (single cell, batch, or parallel).

The V1 tree is untouched: this runner only *reads* the frozen scenario/controller
modules and calls ``v2.sim_v2.run_scene_v2``.

    python -m v2.runner_v2 --scenarios S3 --seeds 2 --seed-offset 200 \
        --algs Ours_rollout --out results_v2_stage1

``--compat`` runs the identical pipeline with the V1 archive configuration
(``compat_v1cfg``), which is how acceptance test A is executed.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
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
from scenarios import SCENARIOS  # noqa: E402

from v2.config_v2 import (V2Cfg, compat_v1cfg, default_v2cfg,  # noqa: E402
                          sim_overrides)
from v2.metrics_v2 import V2_KEYS, summarize_v2  # noqa: E402
from v2.sim_v2 import run_scene_v2  # noqa: E402

ALL_KEYS = list(V1_KEYS) + [k for k in V2_KEYS if k not in V1_KEYS]
HEADER = ["config_id", "scenario", "scenario_name", "alg", "seed"] + ALL_KEYS


def make_cfg(scn, v2cfg, compat):
    cfg = scn["cfg"]
    if compat:
        return cfg
    cfg = replace(cfg, **sim_overrides(v2cfg))
    v2cfg.assert_consistent(cfg)
    return cfg


def _needs_stage2_stack(v2cfg):
    """True when the run asks for a Stage-2 feature.

    Everything else keeps the *exact* Stage-1 code path (the V1 controller
    class), which is what makes the M0 inertness claim bit-exact rather than
    "behaviourally equal".
    """
    return (str(v2cfg.animal_model) != "M0"
            or str(v2cfg.speed_policy) != "off"
            or str(v2cfg.predictor_engine) != "dll"
            or str(v2cfg.predictor_model) != "M0"
            or bool(v2cfg.predictor_shadow))


def run_cell(sid, alg, seed, v2cfg=None, compat=False, contact=True,
             projection="joint", return_res=False, r_robot=None,
             record_states=False, controller=None):
    """One cell.  ``compat=True`` runs the V1 plant bit-exactly; ``r_robot``
    overrides the body radius for the *detector only* (used for the
    "V1 trajectory, solid re-check" arm -- with the feasibility layer and the
    guard off, the radius cannot influence the dynamics)."""
    v2cfg = v2cfg or (compat_v1cfg() if compat else default_v2cfg())
    if r_robot is not None:
        v2cfg = replace(v2cfg, r_robot=float(r_robot))
    scn = SCENARIOS[sid]()
    cfg = make_cfg(scn, v2cfg, compat)
    params = scn.get("params", {})
    if controller is not None:
        ctrl = controller
    elif _needs_stage2_stack(v2cfg):
        from v2.controller_v2 import make_controller
        ctrl = make_controller(alg, ALGS, v2cfg, params,
                               shadow=v2cfg.predictor_shadow)
    else:
        ctrl = ALGS[alg](**params)
    res = run_scene_v2(cfg, scn["path"], scn["obstacles"], ctrl, seed=seed,
                       T_max=scn["T_max"], s0=scn["s0"], v2cfg=v2cfg,
                       contact=contact, feasibility=not compat,
                       projection=projection, record_states=record_states)
    m = summarize_v2(res, v2cfg)
    row = dict(config_id=v2cfg.config_id, scenario=sid, scenario_name=scn["name"],
               alg=alg, seed=seed)
    for k in ALL_KEYS:
        row[k] = m.get(k, np.nan)
    if return_res:
        return row, res
    return row


def _task_entry(task):
    return run_cell(*task["args"], **task["kw"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=["S3"])
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--seed-offset", type=int, default=200)
    ap.add_argument("--algs", nargs="+", default=["Ours_rollout"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--out", default="results_v2_stage1")
    ap.add_argument("--compat", action="store_true",
                    help="V1 archive configuration (acceptance test A)")
    ap.add_argument("--projection", default="joint",
                    choices=["joint", "sequential"])
    ap.add_argument("--no-contact", action="store_true")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE",
                    help="override V2Cfg fields (sensitivity / diagnostics)")
    args = ap.parse_args()

    def _parse(v):
        if v.lower() in ("true", "false"):
            return v.lower() == "true"
        for f in (int, float):
            try:
                return f(v)
            except ValueError:
                pass
        return tuple(v.split(",")) if "," in v else v

    over = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        over[k] = _parse(v)

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    base = compat_v1cfg() if args.compat else default_v2cfg()
    v2cfg = replace(base, **over) if over else base
    tasks = [dict(args=(sid, alg, args.seed_offset + k), kw=dict(
        v2cfg=v2cfg, compat=args.compat, contact=not args.no_contact,
        projection=args.projection))
        for sid in args.scenarios for alg in args.algs
        for k in range(args.seeds)]

    t0 = time.time()
    print(f"V2 runner: scenarios={args.scenarios} algs={args.algs} "
          f"seeds={args.seeds}@{args.seed_offset} jobs={args.jobs} "
          f"compat={args.compat} projection={args.projection}")
    print(f"config_id={v2cfg.config_id}  config_sha256={v2cfg.config_sha256}")
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            rows = list(ex.map(_task_entry, tasks))
    else:
        rows = [_task_entry(t) for t in tasks]

    path = out / ("detail_compat.csv" if args.compat else "detail.csv")
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for r in rows:
            w.writerow([r["config_id"], r["scenario"], r["scenario_name"],
                        r["alg"], r["seed"]] + [r.get(k, np.nan)
                                                for k in ALL_KEYS])
    (out / "run_meta.json").write_text(json.dumps(dict(
        config_id=v2cfg.config_id, config_sha256=v2cfg.config_sha256,
        compat=bool(args.compat), projection=args.projection,
        scenarios=args.scenarios, algs=args.algs, seeds=args.seeds,
        seed_offset=args.seed_offset, jobs=args.jobs,
        wall_s=time.time() - t0, python=sys.version.split()[0],
        numpy=np.__version__), indent=2), encoding="utf-8")
    print(f"wrote {path}  ({len(rows)} rows, {time.time() - t0:.1f} s)")


if __name__ == "__main__":
    main()
