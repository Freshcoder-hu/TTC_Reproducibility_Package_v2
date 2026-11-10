"""Stage B -- Rollout ablation ladder (S2R2, S3; seeds 200-229).

One JSON per (scenario, variant, seed) -> resumable; a run that already exists is
never recomputed.  Same physical corrections and the same `run_scene` /
`summarize` path as Stage A (`corrections.install(environment=True, contact=True)`),
so the rungs are comparable with the main matrix.

Rungs (see proposed_rollout_ablation.py for the ladder documentation):
  R0  R0_base          Proposed            four-state FSM only (the OLD `Ours`; ablation only)
  R1  w/o_Rollout      EventProposed       event coordination, no rollout   <-- key ablation
  R2  w/o_event_coord  RolloutBare         rollout on the bare four-state FSM
  R3  Ours_rollout     RolloutProposed     event coordination + rollout (full)
  R4  horizon_6        RolloutProposed(horizon=6)
  R5  horizon_18       RolloutProposed(horizon=18)

Example:
  python run_cea_ablation.py --out results_cea_ablation --jobs 4
"""
import argparse
import csv
import hashlib
import json
import os
import platform
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

VARIANTS = ["Ours_rollout", "w/o_Rollout", "w/o_event_coord",
            "horizon_6", "horizon_18", "R0_base"]

NOTES = {
    "Ours_rollout": "R3 full: event coordination + rollout (paper's Ours)",
    "w/o_Rollout": "R1 event coordination only (no rollout planner)",
    "w/o_event_coord": "R2 rollout planner driven by the bare four-state FSM",
    "horizon_6": "R4 full, planning horizon 6 s (default 12 s)",
    "horizon_18": "R5 full, planning horizon 18 s (default 12 s)",
    "R0_base": "R0 four-state FSM only (old Ours, ablation-only reference)",
}


def build_controller(variant, kw):
    from proposed import Proposed
    from proposed_event import EventProposed
    from proposed_rollout import RolloutProposed
    from proposed_rollout_ablation import RolloutBare
    if variant == "Ours_rollout":
        return RolloutProposed(**kw)
    if variant == "w/o_Rollout":
        return EventProposed(**kw)
    if variant == "w/o_event_coord":
        return RolloutBare(**kw)
    if variant == "horizon_6":
        return RolloutProposed(horizon=6.0, **kw)
    if variant == "horizon_18":
        return RolloutProposed(horizon=18.0, **kw)
    if variant == "R0_base":
        return Proposed(**kw)
    raise KeyError(variant)


def execute(task):
    sid, variant, seed, out, environment, contact = task
    import corrections
    corrections.install(environment=environment, contact=contact)
    from scenarios import SCENARIOS
    from simulation import run_scene
    from metrics import summarize
    scn = SCENARIOS[sid]()
    kw = dict(scn.get("params", {}))
    ctrl = build_controller(variant, kw)
    t0 = time.monotonic()
    res = run_scene(scn["cfg"], scn["path"], scn["obstacles"], ctrl,
                    seed=seed, T_max=scn["T_max"], s0=scn["s0"])
    row = dict(scenario=sid, variant=variant, seed=seed,
               environment_fixed=environment, contact_fixed=contact,
               horizon=getattr(ctrl, "horizon", np.nan),
               wall_seconds=time.monotonic() - t0, **summarize(res))
    folder = Path(out)
    path = folder / f'{sid}__{variant.replace("/", "-")}__{seed}.json'
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False, indent=2, default=str))
    os.replace(tmp, path)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenarios", nargs="+", default=["S2R2", "S3"])
    ap.add_argument("--variants", nargs="+", default=VARIANTS)
    ap.add_argument("--seed-start", type=int, default=200)
    ap.add_argument("--seed-count", type=int, default=30)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="results_cea_ablation")
    args = ap.parse_args()

    import sys
    folder = Path(args.out)
    folder.mkdir(parents=True, exist_ok=True)
    manifest = dict(vars(args), python=platform.python_version(),
                    numpy=np.__version__, notes=NOTES,
                    code_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sorted(Path(__file__).parent.glob("*.py"))})
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2))

    seeds = list(range(args.seed_start, args.seed_start + args.seed_count))
    rows, tasks = [], []
    for sid in args.scenarios:
        for variant in args.variants:
            for seed in seeds:
                p = folder / f'{sid}__{variant.replace("/", "-")}__{seed}.json'
                if p.exists():
                    rows.append(json.loads(p.read_text()))
                else:
                    tasks.append((sid, variant, seed, str(folder), True, True))
    print(f"cached={len(rows)} to_run={len(tasks)}", flush=True)
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        fs = [pool.submit(execute, t) for t in tasks]
        for f in as_completed(fs):
            r = f.result()
            rows.append(r)
            print(f'{len(rows)} {r["scenario"]} {r["variant"]} {r["seed"]} '
                  f'C={r["collision_rate"]:.4f} d={r["min_dist"]:.3f} '
                  f'rho={r["rho_avg"]:.3f} completion={r["completion"]}',
                  flush=True)
    wall = time.time() - t0

    keys = list(rows[0])
    with (folder / "detail.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r["scenario"], r["variant"], r["seed"])))

    import pandas as pd
    d = pd.DataFrame(rows)
    cols = ["collision_rate", "min_dist", "ttc_min", "rho_avg", "max_dev",
            "success", "completion", "switch_count", "ctrl_ms", "role_switches"]
    summ = d.groupby(["scenario", "variant"])[cols].agg(["mean", "std"])
    summ.to_csv(folder / "summary.csv", encoding="utf-8-sig")
    eq = d.groupby("variant")[cols].mean()
    eq.to_csv(folder / "summary_scenario_equal.csv", encoding="utf-8-sig")
    print(eq.to_string())
    print(f"\nwall {wall:.1f} s; wrote {folder}/detail.csv, summary.csv, "
          f"summary_scenario_equal.csv, manifest.json")


if __name__ == "__main__":
    main()
