"""Per-method wall-clock on the Stage-4 geometry (the Stage-5 projection input).

    python -X utf8 -m v2.runtime_s4          # first: 10-scene Ours_rollout table
    python -X utf8 -m v2.runtime_methods_s4  # then: the other seven methods

``v2.runtime_s4`` times ``Ours_rollout`` on every Stage-4 scene plus ``B2`` on the
two pilot scenes; that covers 2 of the 8 methods of the formal matrix.  This
module fills the rest: it times every remaining method once on **V2S2** (140 m)
and once on **V2S3** (220 m), so the formal 8-method matrix can be projected from
measured numbers instead of a scaling law.

The two scenes are the pilot geometries; the *other* eight scenes are covered by
``runtime.json`` (``Ours_rollout`` per scene), and the projection in
``V2_STAGE4_REPORT.md`` combines the two: per-scene cost from ``runtime.json``,
method-to-method ratio from this table.

Single worker, BLAS pinned to one thread (see the shell command in the report),
so the numbers are comparable with the stored V1 per-cycle table
(``results_cea_timing/timing_table.csv``), which is quoted alongside as the
independent per-tick cost measurement.

Writes ``results_v2_stage4/runtime_methods.csv`` and ``.json``.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from v2.config_v2 import STAGE3_TTC, stage3_v2cfg  # noqa: E402
from v2.runner_s4 import registry_alg, run_cell  # noqa: E402

OUT = ROOT / "results_v2_stage4"
FORMAL_METHODS = ["Ours_rollout", "B1", "B2", "B3", "VH-Path", "VH-Danger",
                  "B-Outmost", "B-PCBF"]
# covered by v2.runtime_s4 (reused, never re-timed here)
FROM_RUNTIME_S4 = {("V2S2", "Ours_rollout"), ("V2S2", "B2"),
                   ("V2S3", "Ours_rollout"), ("V2S3", "B2")}


def time_cell(sid, alg, seed, v2cfg):
    registry_alg(alg)
    t0 = time.perf_counter()
    row, res = run_cell(sid, alg, seed, v2cfg=v2cfg, return_res=True)
    dt = time.perf_counter() - t0
    n_ctrl = max(len(res["logs"]), 1)
    sim_s = float(res["logs"][-1]["t"]) if res["logs"] else 0.0
    return dict(scenario=sid, alg=alg, seed=seed, wall_s=dt,
                n_ctrl_ticks=n_ctrl, sim_s=sim_s,
                ms_per_ctrl_tick=1e3 * dt / n_ctrl,
                ended=res["terminated_by"],
                collision_rate=row.get("collision_rate"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=["V2S2", "V2S3"])
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--methods", nargs="+", default=FORMAL_METHODS)
    args = ap.parse_args()
    v2cfg = stage3_v2cfg("M1", **STAGE3_TTC)
    runtime_json = OUT / "runtime.json"
    base = {}
    if runtime_json.exists():
        base = json.loads(runtime_json.read_text(encoding="utf-8"))
    reused = {}
    for r in base.get("measured", []):
        reused[(r["scenario"], r["alg"])] = r

    rows = []
    t_all = time.perf_counter()
    for sid in args.scenes:
        for alg in args.methods:
            key = (sid, alg)
            if key in FROM_RUNTIME_S4 and key in reused:
                r = dict(reused[key])
                r["source"] = "runtime_s4 (reused)"
                rows.append(r)
                continue
            r = time_cell(sid, alg, args.seed, v2cfg)
            r["source"] = "measured here"
            rows.append(r)
            print(f"{sid:6s}{alg:14s}wall={r['wall_s']:7.2f} s  "
                  f"ctrl={r['n_ctrl_ticks']:5d}  "
                  f"{r['ms_per_ctrl_tick']:6.2f} ms/tick  {r['ended']}",
                  flush=True)
    wall = time.perf_counter() - t_all
    keys = ["scenario", "alg", "seed", "wall_s", "n_ctrl_ticks", "sim_s",
            "ms_per_ctrl_tick", "ended", "collision_rate", "source"]
    with open(OUT / "runtime_methods.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---- ratio table ---------------------------------------------------
    ratios = {}
    for sid in args.scenes:
        sub = {r["alg"]: r for r in rows if r["scenario"] == sid}
        if "Ours_rollout" not in sub:
            continue
        ref = sub["Ours_rollout"]["ms_per_ctrl_tick"]
        ratios[sid] = {a: sub[a]["ms_per_ctrl_tick"] / ref for a in sub}
    print(f"\nper-control-tick cost ratio to Ours_rollout "
          f"(measured, double-precision):")
    hdr = f"{'method':16s}" + "".join(f"{s:>12s}" for s in ratios)
    print(hdr)
    for a in args.methods:
        print(f"{a:16s}" + "".join(
            f"{ratios[s].get(a, float('nan')):12.3f}" for s in ratios))
    (OUT / "runtime_methods.json").write_text(json.dumps(dict(
        config_id=v2cfg.config_id, rows=rows, ratios=ratios,
        wall_s=wall, seed=args.seed), indent=2, default=float),
        encoding="utf-8")
    print(f"\nwrote {OUT / 'runtime_methods.csv'} ({len(rows)} cells, "
          f"{wall:.1f} s)")


if __name__ == "__main__":
    main()
