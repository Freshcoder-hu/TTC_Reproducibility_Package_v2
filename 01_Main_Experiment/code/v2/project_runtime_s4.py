"""Stage-5 runtime projection for the formal V2 matrix (2400 runs).

    python -X utf8 -m v2.runtime_s4
    python -X utf8 -m v2.runtime_methods_s4
    python -X utf8 -m v2.project_runtime_s4

Inputs (all measured, nothing assumed except where stated):

* ``results_v2_stage4/runtime.json``       -- ``Ours_rollout`` wall-clock on every
  Stage-4 scene (so the *scene* factor -- path length, team size, tick count --
  is measured, not extrapolated);
* ``results_v2_stage4/runtime_methods.json`` -- per-control-tick cost of every
  method on V2S2 (140 m) and V2S3 (220 m), so the *method* factor is measured;
* ``results_cea_timing/timing_table.csv``  -- the stored strict-serial V1
  per-cycle cost table, quoted as the independent cross-check of the ratios.

The projection is arithmetic:

    per_run(scene, method) = wall(scene, Ours_rollout) * r_method
    total = 30 seeds * sum over the 10 scenes, 8 methods

``r_method`` is the measured per-tick cost ratio to ``Ours_rollout`` (1.0 by
construction).  **Assumption** (stated in the report): the method-to-method
per-tick cost ratio is scene-independent -- the scenes differ in tick count and
team size, which the ``Ours_rollout`` per-scene wall-clock already contains.
The ratios are checked on both a 140 m and a 220 m scene; if they agree within a
few per cent the assumption is supported by the data.

Writes ``results_v2_stage4/runtime_projection.csv`` / ``.json``.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OUT = ROOT / "results_v2_stage4"
METHODS = ["Ours_rollout", "B1", "B2", "B3", "VH-Path", "VH-Danger",
           "B-Outmost", "B-PCBF"]
SCENES_220 = {"V2S3"}
JOBS = (4, 12)
SEEDS = 30


def main():
    rt = json.loads((OUT / "runtime.json").read_text(encoding="utf-8"))
    rm = json.loads((OUT / "runtime_methods.json").read_text(encoding="utf-8"))
    ticks = {}
    for r in rm["rows"]:
        ticks.setdefault((r["scenario"], r["alg"]), r)
    wall = {(r["scenario"], r["alg"]): r["wall_s"] for r in rt["measured"]}
    nctrl = {(r["scenario"], r["alg"]): r["n_ctrl_ticks"]
             for r in rt["measured"]}

    scenes = [r["scenario"] for r in rt["measured"]
              if r["alg"] == "Ours_rollout"]
    # ---- per-method ratio to Ours_rollout, measured on both pilot scenes ----
    ratio = {"140": {}, "220": {}}
    for scene, key in (("V2S2", "140"), ("V2S3", "220")):
        ref = ticks[(scene, "Ours_rollout")]["ms_per_ctrl_tick"]
        for m in METHODS:
            ratio[key][m] = ticks[(scene, m)]["ms_per_ctrl_tick"] / ref
    # stored V1 strict-serial table, cross-check (7 of the 8 methods)
    v1 = {}
    p = ROOT / "results_cea_timing" / "timing_table.csv"
    if p.exists():
        import csv
        with open(p, newline="", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                v1[r["方法"]] = float(r["中位数 (ms)"])
    v1_ratio = ({k: v / v1["Ours_rollout"] for k, v in v1.items()}
                if v1 else {})

    # ---- per-cell wall-clock -------------------------------------------
    rows = []
    for s in scenes:
        key = "220" if s in SCENES_220 else "140"
        for m in METHODS:
            w = wall[(s, "Ours_rollout")] * ratio[key][m]
            rows.append(dict(scene=s, method=m, group=key,
                             wall_s=w, n_ctrl_ticks=nctrl[(s,
                                                           "Ours_rollout")],
                             ratio_to_ours=ratio[key][m],
                             runs=SEEDS,
                             total_s=w * SEEDS))
    per_method = {}
    for m in METHODS:
        t = sum(r["total_s"] for r in rows if r["method"] == m)
        per_method[m] = dict(
            wall_140_s=sum(r["wall_s"] for r in rows
                           if r["method"] == m and r["group"] == "140") / 9,
            wall_220_s=sum(r["wall_s"] for r in rows
                           if r["method"] == m and r["group"] == "220"),
            ratio_140=ratio["140"][m], ratio_220=ratio["220"][m],
            total_h=t / 3600.0, n_runs=10 * SEEDS)
    grand = sum(v["total_h"] for v in per_method.values())
    # measured parallel speed-up (v2.parallel_efficiency_s4); the jobs=12 probe
    # submitted only 8 tasks, so it is a *lower bound* on a 12-worker pool.
    speed = {}
    pe = OUT / "parallel_efficiency.json"
    if pe.exists():
        for r in json.loads(pe.read_text(encoding="utf-8")):
            speed[int(r["jobs"])] = float(r["speedup"])
    proj = dict(
        assumption=("per_run = wall(scene, Ours_rollout) x r_method; r_method = "
                    "measured per-control-tick cost ratio. Scene and team-size "
                    "effects are in the measured Ours_rollout wall; the ratio "
                    "is assumed scene-independent."),
        n_runs=len(METHODS) * len(scenes) * SEEDS,
        n_scenes=len(scenes), n_methods=len(METHODS), seeds=SEEDS,
        scenes=scenes, per_scene_wall_ours_s=
        {s: wall[(s, "Ours_rollout")] for s in scenes},
        per_scene_ctrl_ticks={s: nctrl[(s, "Ours_rollout")] for s in scenes},
        per_method=per_method,
        total_single_thread_h=grand,
        jobs={str(j): dict(h=grand / j, days=grand / j / 24.0,
                           ideal_speedup=j,
                           measured_speedup=speed.get(j),
                           measured_h=(grand / speed[j]) if j in speed else None)
              for j in JOBS},
        measured_speedup=speed,
        v1_cross_check=v1_ratio,
        cells=rows,
    )
    (OUT / "runtime_projection.json").write_text(
        json.dumps(proj, indent=2, default=float), encoding="utf-8")
    import csv as _csv
    with open(OUT / "runtime_projection.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = _csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    print(f"{'method':14s}{'wall 140 m [s]':>16s}{'wall 220 m [s]':>16s}"
          f"{'ratio 140':>11s}{'ratio 220':>11s}{'ratio V1':>10s}"
          f"{'300-run [h]':>13s}{'share':>8s}")
    for m in sorted(METHODS, key=lambda x: -per_method[x]["total_h"]):
        v = per_method[m]
        print(f"{m:14s}{v['wall_140_s']:16.2f}{v['wall_220_s']:16.2f}"
              f"{v['ratio_140']:11.3f}{v['ratio_220']:11.3f}"
              f"{v1_ratio.get(m, float('nan')):10.3f}"
              f"{v['total_h']:13.2f}{v['total_h'] / grand * 100:7.1f} %")
    print(f"\n2400 runs, single thread: {grand:.2f} h")
    for j in JOBS:
        s = speed.get(j)
        print(f"  jobs={j:2d}: ideal {grand / j:.2f} h"
              + (f"   measured speed-up {s:.2f}x -> {grand / s:.2f} h"
                 if s else ""))
    print(f"\nwrote {OUT / 'runtime_projection.csv'} / .json")


if __name__ == "__main__":
    main()
