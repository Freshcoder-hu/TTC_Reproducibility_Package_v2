"""The four-rung ablation ladder on the two evidence-carrying scenes.

    python -X utf8 -m v2.stage5.ablation --jobs 12

Rungs (each adds exactly one mechanism to the one above it):

    R0_four_state  proposed.Proposed          four-state FSM only
    R1_handover    proposed_improved.HandoverProposed
                                              + handover of the avoidance slot
    R2_event       proposed_event.EventProposed
                                              + the event layer (= "w/o rollout",
                                                the pilot's Ours_event)
    R3_rollout     proposed_rollout.RolloutProposed through the Stage-2
                   OursRolloutV2 stack        + the rollout planner
                                              (= Ours_rollout, the shipped method)

The ladder is built through the **same** controller factory the formal matrix
uses, so R3 *is* the formal ``Ours_rollout`` cell: the 60 R3 rows must be
bit-identical to the corresponding 60 rows of the formal matrix (checked by
``v2.stage5.checks``).  Everything else is frozen: same plant, same sensing,
same M1 animal, same TTC tuple, same scenes, same seeds 2000-2029.

The layered speed policy (``v2cfg.speed_policy = "manoeuvre"``) is part of the
shipped configuration and wraps every rung; it is a no-op for R0-R2 because
only the rollout controller *declares* the manoeuvre geometry the rule needs
(``SpeedPolicy.limit`` leaves the command untouched when no guide is
declared).  ``sp_n_bounded_ticks`` is reported per rung so this is visible, and
the supplementary probe ``--probe-nopolicy`` runs R3 with the policy off so the
R2 -> R3 step can be attributed to the planner alone.

Outputs: ``results_v2_stage5/ablation/<rung>.csv`` (60 rows each),
``ablation/detail.csv`` (240 rows), ``ablation/summary.csv``, ``_manifest.json``.
"""
from __future__ import annotations

import argparse
import csv
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
from v2.runner_s4 import ALGS, registry_alg  # noqa: E402,F401
from v2.scenarios_v2 import resolve_scene  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5.runs import HEADER, run_one  # noqa: E402

import run_revised_experiments  # noqa: E402,F401  (registers Ours_rollout)


def rung_params(v2cfg, scn):
    """The controller kwargs the formal matrix passes for the Ours_* methods."""
    params = dict(scn.get("params", {}))
    if v2cfg.ttc_from_config:
        params.setdefault("t_pre", float(v2cfg.ttc_pre))
        params.setdefault("t_divert", float(v2cfg.ttc_divert))
        params.setdefault("t_rec", float(v2cfg.ttc_rec))
        params.setdefault("hyst", float(v2cfg.ttc_hyst))
    return params


def build_rung(rung, v2cfg, params, speed_policy=True):
    from v2.controller_v2 import SpeedPolicyController, make_controller
    if rung in ("R3_rollout", "R3_no_speed_policy"):
        if speed_policy and rung == "R3_rollout":
            return make_controller("Ours_rollout", ALGS, v2cfg, params)
        cfg = v2cfg.with_(speed_policy="off")
        return make_controller("Ours_rollout", ALGS, cfg, params)
    from proposed import Proposed
    from proposed_event import EventProposed
    from proposed_improved import HandoverProposed
    if rung == "R0_four_state":
        ctrl = Proposed(**params)
    elif rung == "R1_handover":
        ctrl = HandoverProposed(**params)
    elif rung == "R2_event":
        ctrl = EventProposed(**params)
    else:
        raise KeyError(rung)
    if speed_policy and v2cfg.speed_policy != "off":
        ctrl = SpeedPolicyController(ctrl, v2cfg)
    return ctrl


def run_ablation_cell(rung, scenario, seed, v2cfg, speed_policy=True):
    scn = resolve_scene(scenario)
    ctrl = build_rung(rung, v2cfg, rung_params(v2cfg, scn),
                      speed_policy=speed_policy)
    row = run_one(scenario, rung, seed, v2cfg=v2cfg, controller=ctrl)
    row["rung"] = rung
    row["method"] = rung
    return row


def _task(t):
    return run_ablation_cell(*t["args"], **t["kw"])


def _write(path, rows, header):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r.get(k, np.nan) for k in header])
    tmp.replace(path)


def run_batch(rung, scenes, seeds, jobs, prefix="", speed_policy=True):
    path = common.ABL_DIR / (prefix + rung + ".csv")
    v2cfg = stage3_v2cfg("M1")
    n_expected = len(scenes) * len(seeds)
    if path.exists():
        try:
            with open(path, encoding="utf-8-sig") as f:
                rows = list(csv.DictReader(f))
            if len({(r.get("scenario"), r.get("seed")) for r in rows}) >= n_expected:
                print(f"[{rung:22s}] cached -> {path.name}", flush=True)
                return dict(rung=rung, cached=True, n=len(rows), wall_s=0.0)
        except (OSError, csv.Error):
            pass
    tasks = [dict(args=(rung, sid, sd, v2cfg),
                  kw=dict(speed_policy=speed_policy))
             for sid in scenes for sd in seeds]
    t0 = time.perf_counter()
    if jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            rows = list(ex.map(_task, tasks, chunksize=1))
    else:
        rows = [_task(t) for t in tasks]
    wall = time.perf_counter() - t0
    order = {s: i for i, s in enumerate(scenes)}
    rows.sort(key=lambda r: (order.get(r.get("scenario"), 9),
                             int(r.get("seed") or 0)))
    hdr = ["rung"] + [k for k in HEADER if k != "rung"]
    _write(path, rows, hdr)
    print(f"[{rung:22s}] {len(rows)} rows in {wall:.1f} s -> {path.name}",
          flush=True)
    return dict(rung=rung, cached=False, n=len(rows), wall_s=wall, jobs=jobs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=common.ABLATION_SCENARIOS)
    ap.add_argument("--seeds", nargs="+", type=int, default=common.SEEDS)
    ap.add_argument("--rungs", nargs="+", default=common.ABLATION_RUNGS)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--probe-nopolicy", action="store_true",
                    help="also run R3 with the layered speed policy off")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    common.ensure_dirs()
    v2cfg = stage3_v2cfg("M1")
    print(f"Stage-5 ablation  config_id={v2cfg.config_id} "
          f"scenes={args.scenes} seeds={args.seeds[0]}..{args.seeds[-1]} "
          f"rungs={args.rungs} jobs={args.jobs}", flush=True)
    if args.force:
        for r in args.rungs:
            p = common.ABL_DIR / f"{r}.csv"
            if p.exists():
                p.unlink()
    t0 = time.time()
    batches = [run_batch(r, args.scenes, args.seeds, args.jobs)
               for r in args.rungs]
    if args.probe_nopolicy:
        batches.append(run_batch("R3_no_speed_policy", args.scenes,
                                 args.seeds, args.jobs, prefix="_probe_",
                                 speed_policy=False))
    # ---- merge ---------------------------------------------------------
    rows = []
    for b in batches:
        p = common.ABL_DIR / (("_probe_" if b["rung"].startswith("R3_no")
                               else "") + b["rung"] + ".csv")
        if p.exists():
            with open(p, encoding="utf-8-sig") as f:
                rows.extend(csv.DictReader(f))
    hdr = ["rung"] + [k for k in HEADER if k != "rung"]
    _write(common.ABL_DIR / "detail.csv", rows, hdr)
    total = time.time() - t0
    common.write_json(common.ABL_DIR / "_manifest.json", dict(
        config_id=v2cfg.config_id, scenes=args.scenes, seeds=args.seeds,
        rungs=args.rungs, jobs=args.jobs, batches=batches,
        total_wall_s=total, n_rows=len(rows),
        labels=common.ABLATION_LABELS,
        python=sys.version.split()[0], numpy=np.__version__))
    print(f"\nablation: {len(rows)} rows, wall {total:.1f} s", flush=True)


if __name__ == "__main__":
    main()
