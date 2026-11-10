"""Ramp-up / structural-blindness analysis for the R = 30 m condition.

Two numbers per scenario:

  * analytic ramp-up  t_ramp = R / |v_rel|
      v_rel = v_obstacle - v_flock, with v_flock = cfg.v_base * tangent at the
      obstacle's entry point (the flock advances at its nominal speed).
      This is the time between "the obstacle enters the detection shell" and
      "the obstacle reaches the flock", i.e. the whole warning budget the gate
      leaves at radius R.  Pure geometry, method-independent.

  * measured first-detection time  t_known(o_k)
      run through the production path with a recorder attached to the gate
      (`sensing_gate.install(..., recorder=...)`), reporting the first cycle at
      which obstacle k is known, and the fleet's minimum distance to it then.
      Method-dependent (the fleet's own motion matters), so it is reported for
      `Ours_rollout` (and B1 as a cross-check).

Compared against the two control constants: the 12 s rollout horizon and the
6 s DIVERT threshold (`b2_t_equiv` / `T_DIVERT`).

Usage:
    python gate_rampup_30m.py --radius 30 --algs Ours_rollout B1
"""
import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np
import pandas as pd

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

SCEN = ["S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]
HORIZON = 12.0      # RolloutProposed default planning horizon [s]
DIVERT = 6.0        # b2_t_equiv / T_DIVERT nominal warning time [s]


def analytic():
    """t_ramp = R / |v_rel| per scenario, from the scenario definitions."""
    from scenarios import SCENARIOS
    rows = []
    for sid in SCEN:
        scn = SCENARIOS[sid]()
        cfg, path = scn["cfg"], scn["path"]
        for k, o in enumerate(scn["obstacles"]):
            v = np.asarray(o.vel, dtype=float)
            s, _, _ = path.closest(np.asarray(o.start, dtype=float))
            tang = path.tangent_at(s)
            v_fl = cfg.v_base * tang
            v_rel = float(np.linalg.norm(v - v_fl))
            rows.append(dict(scenario=sid, obstacle=k, v_obs=float(np.linalg.norm(v)),
                             v_obs_vec=str(np.round(v, 3).tolist()),
                             v_flock=float(np.linalg.norm(v_fl)),
                             v_rel=v_rel,
                             flock_to_obs0=float(np.linalg.norm(
                                 path.point_at(scn["s0"])
                                 - np.asarray(o.start, dtype=float))),
                             t_spawn=float(o.t_spawn),
                             radius=float(cfg.r_obs),
                             T_max=float(scn["T_max"])))
    return pd.DataFrame(rows)


class Recorder:
    """First-known time per obstacle index (OR over the two refreshes per step)."""

    def __init__(self, n_obs, radius):
        self.n = int(n_obs)
        self.R = float(radius)
        self.first_known = np.full(self.n, np.inf)
        self.dmin_at_known = np.full(self.n, np.inf)
        self.t_alive = np.full(self.n, np.inf)
        self._t = None
        self._buf = None
        self.n_cycles = 0
        self.n_cycles_unknown = 0

    def __call__(self, t, alive, known, dmin, robot_pos):
        if robot_pos is not None and np.all(robot_pos == 0.0):
            return                      # initial refresh with the all-zero fleet
        if self._t is None or t != self._t:
            if self._t is not None:
                self._flush()
            self._t = t
            self._buf = dict(alive=alive.copy(), known=known.copy(),
                             dmin=dmin.copy())
            self.n_cycles += 1
        else:
            self._buf["alive"] |= alive
            self._buf["known"] |= known
            self._buf["dmin"] = np.minimum(self._buf["dmin"], dmin)

    def _flush(self):
        b = self._buf
        t = self._t
        if b["alive"].any() and not b["known"].any():
            self.n_cycles_unknown += 1
        for k in range(self.n):
            if not b["alive"][k]:
                continue
            self.t_alive[k] = min(self.t_alive[k], t)
            if b["known"][k] and not np.isfinite(self.first_known[k]):
                self.first_known[k] = t
                self.dmin_at_known[k] = float(b["dmin"][k])

    def finalize(self):
        if self._t is not None:
            self._flush()
            self._t = None
        return self


def measured(radius, algs, seeds):
    import corrections
    corrections.install(environment=True, contact=True)
    import sensing_gate
    from scenarios import SCENARIOS
    from run_experiments import ALGS
    from simulation import run_scene
    from metrics import summarize

    rows = []
    for alg in algs:
        for sid in SCEN:
            scn = SCENARIOS[sid]()
            n_obs = len(scn["obstacles"])
            for seed in seeds:
                rec = Recorder(n_obs, radius)
                sensing_gate.install(radius, False, recorder=rec)
                ctrl = sensing_gate.wrap(ALGS[alg](**scn.get("params", {})))
                res = run_scene(scn["cfg"], scn["path"], scn["obstacles"], ctrl,
                                seed=seed, T_max=scn["T_max"], s0=scn["s0"])
                rec.finalize()
                m = summarize(res)
                for k in range(n_obs):
                    rows.append(dict(
                        alg=alg, scenario=sid, seed=seed, obstacle=k,
                        t_alive=(rec.t_alive[k]
                                 if np.isfinite(rec.t_alive[k]) else np.nan),
                        t_first_known=(rec.first_known[k]
                                       if np.isfinite(rec.first_known[k])
                                       else np.nan),
                        dmin_at_first_known=(rec.dmin_at_known[k]
                                             if np.isfinite(rec.dmin_at_known[k])
                                             else np.nan),
                        n_cycles=rec.n_cycles,
                        n_cycles_obstacle_live_unseen=rec.n_cycles_unknown,
                        frac_cycles_unseen=(rec.n_cycles_unknown / rec.n_cycles
                                            if rec.n_cycles else np.nan),
                        collision_rate=m["collision_rate"], min_dist=m["min_dist"],
                        success=m["success"], completion=m["completion"]))
                sensing_gate.uninstall()
                print(f"  {alg:13s} {sid:5s} seed={seed} "
                      f"t_known={np.round(rec.first_known, 2).tolist()} "
                      f"unseen={rec.n_cycles_unknown}/{rec.n_cycles}", flush=True)
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=30.0)
    ap.add_argument("--algs", nargs="+", default=["Ours_rollout", "B1"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[200])
    ap.add_argument("--out", default="results_cea_matrix_30m")
    ap.add_argument("--skip-measured", action="store_true")
    args = ap.parse_args()
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)

    a = analytic()
    a["t_ramp_analytic"] = args.radius / a.v_rel
    a["ramp_minus_horizon"] = a.t_ramp_analytic - HORIZON
    a["ramp_minus_divert"] = a.t_ramp_analytic - DIVERT
    a["lt_horizon"] = a.t_ramp_analytic < HORIZON
    a["lt_divert"] = a.t_ramp_analytic < DIVERT
    a.to_csv(out / "rampup_analytic.csv", index=False, encoding="utf-8-sig")
    print("=== 解析 ramp-up：R/|v_rel| ===")
    print(a[["scenario", "obstacle", "v_rel", "t_ramp_analytic",
             "ramp_minus_horizon", "ramp_minus_divert"]].round(3)
          .to_string(index=False))

    if not args.skip_measured:
        m = measured(args.radius, args.algs, args.seeds)
        m.to_csv(out / "rampup_measured.csv", index=False,
                 encoding="utf-8-sig")
        g = m.groupby(["scenario", "obstacle"]).agg(
            t_first_known=("t_first_known", "min"),
            dmin_at_first_known=("dmin_at_first_known", "max"),
            frac_cycles_unseen=("frac_cycles_unseen", "mean")).reset_index()
        print("\n=== 实测首次探测（最近一次；dmin 为队—障碍物距离）===")
        print(g.round(3).to_string(index=False))
        g.to_csv(out / "rampup_measured_summary.csv", index=False,
                 encoding="utf-8-sig")

    # written summary for the report
    s = a.groupby("scenario").agg(
        v_rel=("v_rel", "max"), t_ramp=("t_ramp_analytic", "min"),
        t_ramp_max=("t_ramp_analytic", "max")).reset_index()
    s.to_csv(out / "rampup_by_scenario.csv", index=False, encoding="utf-8-sig")
    (out / "rampup_meta.json").write_text(json.dumps(
        dict(radius=args.radius, horizon=HORIZON, divert=DIVERT,
             rule="structurally blind for the planner if R/|v_rel| < horizon; "
                  "for DIVERT if R/|v_rel| < 6 s"), indent=2), encoding="utf-8")
    print(f"\n已写 {out}/rampup_analytic.csv, rampup_by_scenario.csv"
          f"{'' if args.skip_measured else ', rampup_measured*.csv'}")


if __name__ == "__main__":
    main()
