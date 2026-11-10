"""Probe the no-intervention flock kinematics on the Stage-4 paths.

    python -X utf8 -m v2.probe_geometry_s4

Runs the passive controller (B1) with **no obstacles** on the Stage-4 path and
records, per seed, the flock centroid's arc length vs time plus the flock's
lateral extent.  These measurements are what the release rule needs:

    t_spawn = t_arr(e_s) - T_enc         (T_enc >= 2 R / v)

and they are also what fixes the "graze" lateral offset of the single-obstacle
family.  The *fitted* numbers are then written into ``v2/scenarios_v2.py`` as
literals so that scene construction never depends on a cached file.
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

from config import default_cfg  # noqa: E402
from path import straight_path  # noqa: E402

from v2.config_v2 import sim_overrides, stage3_v2cfg  # noqa: E402
from v2.runner_v2 import run_cell  # noqa: E402
from v2.sim_v2 import run_scene_v2  # noqa: E402

OUT = ROOT / "results_v2_stage4" / "geometry"


def reference_run(seed, length, halfwidth, T_max, v2cfg=None):
    v2cfg = v2cfg or stage3_v2cfg("M1")
    cfg = replace(default_cfg(n_sheep=36, n_robots=12), **sim_overrides(v2cfg))
    from run_experiments import ALGS
    path = straight_path(0.0, float(length), halfwidth=float(halfwidth))
    res = run_scene_v2(cfg, path, [], ALGS["B1"](), seed=seed, T_max=T_max,
                       s0=8.0, v2cfg=v2cfg, contact=False, record_states=True)
    tr = res["trace_states"]
    t = np.array([r[0] for r in tr], dtype=float)
    spos = np.array([r[3] for r in tr], dtype=float)   # (T,N,2)
    alive = np.array([r[2] for r in tr], dtype=bool)
    s = spos[:, :, 0]                                  # straight path: s == x
    c = spos.mean(axis=1)
    lat = spos[:, :, 1] - c[:, 1][:, None]
    return dict(t=t, s_centroid=c[:, 0], lat_abs=np.abs(lat).ravel(),
                s_min=s.min(axis=1), s_max=s.max(axis=1),
                ended=res["terminated_by"], n_ticks=len(tr))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    v2cfg = stage3_v2cfg("M1")
    seeds = list(range(1000, 1005))
    out = {}
    for length, T_max in ((140.0, 200.0), (220.0, 320.0)):
        refs = [reference_run(s, length, 8.0, T_max) for s in seeds]
        t = refs[0]["t"]
        T = max(len(r["s_centroid"]) for r in refs)
        sc = np.full((len(refs), T), np.nan)
        for i, r in enumerate(refs):
            sc[i, :len(r["s_centroid"])] = r["s_centroid"]      # NaN past goal
        lat_abs = np.concatenate([r["lat_abs"] for r in refs])
        # arrival time at a set of arc lengths (mean/interp over seeds)
        grid = np.arange(10.0, min(length * 0.95, sc.max()) + 1e-9, 5.0)
        t_arr = []
        for s0 in grid:
            vals = []
            for row in sc:
                idx = np.where(row >= s0)[0]
                vals.append(t[idx[0]] if len(idx) else np.nan)
            t_arr.append(float(np.nanmean(vals)))
        t_arr = np.array(t_arr)
        v_mean = float(np.mean(np.diff(grid) / np.diff(t_arr)))
        d = dict(length=length, T_max=T_max, seeds=seeds,
                 ticks=refs[0]["n_ticks"], ended=refs[0]["ended"],
                 v_flock_mean=v_mean,
                 lat_abs_p50=float(np.percentile(lat_abs, 50)),
                 lat_abs_p90=float(np.percentile(lat_abs, 90)),
                 lat_abs_p95=float(np.percentile(lat_abs, 95)),
                 lat_abs_p99=float(np.percentile(lat_abs, 99)),
                 s_centroid_max=float(np.nanmax(sc)),
                 t_last=float(t[-1]),
                 grid=grid.tolist(), t_arr=t_arr.tolist())
        out[str(int(length))] = d
        print(f"L={length:.0f} m  v_flock={v_mean:.3f} m/s  "
              f"|lat| p50/p90/p95/p99 = {d['lat_abs_p50']:.2f}/"
              f"{d['lat_abs_p90']:.2f}/{d['lat_abs_p95']:.2f}/"
              f"{d['lat_abs_p99']:.2f} m  centroid reaches "
              f"{d['s_centroid_max']:.1f} m at t={d['t_last']:.1f} s "
              f"({d['ended']})")
        for s0, ta in zip(grid, t_arr):
            if abs(s0 % 20.0) < 1e-9:
                print(f"    s={s0:6.1f} m  t_arr={ta:7.2f} s")
    (OUT / "reference_kinematics.json").write_text(
        json.dumps(out, indent=2), encoding="utf-8")
    print(f"\nwrote {OUT / 'reference_kinematics.json'}")


if __name__ == "__main__":
    main()
