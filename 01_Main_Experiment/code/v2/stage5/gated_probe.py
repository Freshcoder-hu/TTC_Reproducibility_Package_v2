"""Equivalence proof behind the Stage-5 preemption fix (item 1).

    python -X utf8 -m v2.stage5.gated_probe

``preempt.analyse_recover`` takes the controller-side TTC from
``res["sense_log"][i]["ttc_ctrl"]`` (recorded by :mod:`v2.stage5.probe` at the
publish instant) and the truth-side TTC from the frozen log series
``logs[i]["ttc_each"]``.  Three things have to be true for that to be a fair
pair of readings, and this module measures all three on real cells:

``A`` the published track **is** the truth obstacle at the same instant
      (position and velocity, bit-identical) -- so the controller is not being
      handed stale or degraded data in the shipped configuration;
``B`` the TTC recorded from the published view equals the TTC computed from the
      truth state at that same instant, for every published obstacle
      (``max|diff| = 0``) -- so ``ttc_ctrl`` really is "the TTC the controller
      could compute";
``C`` the *logged* truth series is **not** that quantity: it is evaluated one
      integration step later (``sim_v2`` writes the log row after
      ``o.update(t + dt)``), so its values differ by up to several seconds even
      inside the risk band (TTC <= 15 s), and by up to ~56 s far from it, where
      the analytic TTC is almost flat.  That is exactly why Stage 5 records the
      TTC at the publish instant instead of masking the log.  ``C`` also
      quantifies the onset-time shift this introduces and checks that the
      **truth-side preemption flag is nevertheless unchanged** (it is, on every
      probe cell) -- which is what makes ``preempt_truth`` usable as a
      sensitivity variant.

Outputs ``results_v2_stage5/checks/G_gated_ttc.csv`` and ``.json``.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import corrections  # noqa: E402

corrections.install(environment=True, contact=True)

import numpy as np  # noqa: E402

import v2.sim_v2 as sim_v2  # noqa: E402
from ttc import ttc_matrix  # noqa: E402
from v2.config_v2 import stage3_v2cfg  # noqa: E402
from v2.runner_s4 import run_cell  # noqa: E402
from v2.stage5 import common  # noqa: E402
from v2.stage5.preempt import analyse_recover  # noqa: E402
from v2.stage5.probe import ProbeSensorium  # noqa: E402

TOL = 1e-12
DEFAULT_CELLS = [("V2S3", "Ours_rollout", 1000),
                 ("V2S2", "Ours_rollout", 2000),
                 ("V2S2", "B1", 2000),
                 ("V2S1", "Ours_rollout", 2000)]
_LAST = {}


class RecSensorium(ProbeSensorium):
    """ProbeSensorium + the truth-at-publish series and array snapshots."""

    def __init__(self, v2cfg, cfg, seed=0):
        super().__init__(v2cfg, cfg, seed)
        self.rec = []
        self._op_all = np.zeros((0, 2))
        self._ov_all = np.zeros((0, 2))
        self._alive_all = np.zeros(0, dtype=bool)

    def _sample(self, t, robot_pos, obs):
        super()._sample(t, robot_pos, obs)
        if obs:
            self._op_all = np.array([o.pos for o in obs], dtype=float)
            self._ov_all = np.array([o.vel for o in obs], dtype=float)
            self._alive_all = np.array([o.is_alive() for o in obs], dtype=bool)
        else:
            self._op_all = np.zeros((0, 2))
            self._ov_all = np.zeros((0, 2))
            self._alive_all = np.zeros(0, dtype=bool)

    def publish(self, view_truth, t):
        v = super().publish(view_truth, t)          # records ttc_ctrl
        if not self._ctrl_tick:
            return v
        sp_t = np.asarray(view_truth.sheep_pos_global, dtype=float)
        sv_t = np.asarray(view_truth.sheep_vel_global, dtype=float)
        truth_t = np.full(len(self._op_all), np.inf)
        if len(self._op_all) and len(sp_t):
            T, _, _ = ttc_matrix(sp_t, sv_t, self._op_all, self._ov_all,
                                 self._r_sum, horizon=12.0)
            truth_t = np.min(T, axis=0)
            truth_t = np.where(self._alive_all, truth_t, np.inf)
        self.rec.append(dict(
            t=float(t),
            idx=[int(o.index) for o in v.obstacles_active],
            pub_pos=np.array(v.obs_pos, dtype=float).reshape(-1, 2),
            pub_vel=np.array(v.obs_vel, dtype=float).reshape(-1, 2),
            truth_pos=self._op_all.copy(), truth_vel=self._ov_all.copy(),
            truth_alive=self._alive_all.copy(),
            truth_ttc=truth_t.copy(),
            ttc_ctrl=dict(self.log[-1].get("ttc_ctrl") or {}),
            n_sheep_ctrl=int(len(v.sheep_pos)),
            n_sheep_truth=int(len(sp_t))))
        return v


def _build(v2cfg, cfg, seed):
    mode = str(getattr(v2cfg, "sense_mode", "exact"))
    if mode == "exact":
        return None
    if mode != "range":
        raise ValueError(mode)
    s = RecSensorium(v2cfg, cfg, seed)
    _LAST["s"] = s
    return s


def probe_cell(scenario, method, seed, v2cfg):
    saved = sim_v2.build_sensing
    sim_v2.build_sensing = _build
    try:
        row, res = run_cell(scenario, method, seed, v2cfg=v2cfg,
                            return_res=True)
        rec = _LAST["s"].rec
    finally:
        sim_v2.build_sensing = saved
    logs = res["logs"]
    K = len(logs[0].get("ttc_each") or [])
    n_sheep = int(v2cfg.n_sheep)
    # ---- A/B: published vs truth at the same instant ----------------------
    n_ticks = n_pairs = 0
    d_pos = d_vel = d_ttc = 0.0
    n_animal_full = 0
    n_dead_track = n_dead_track_risk = 0
    # ---- C: logged series (one step later) -------------------------------
    d_log = d_log_near = 0.0
    n_log_pairs = n_log_both = n_log_inf_boundary = 0
    for i, (r, lg) in enumerate(zip(rec, logs)):
        n_ticks += 1
        if r["n_sheep_ctrl"] == n_sheep == r["n_sheep_truth"]:
            n_animal_full += 1
        for j, k in enumerate(r["idx"]):
            if k >= K:
                continue
            alive = bool(r["truth_alive"][k]) if k < len(r["truth_alive"]) \
                else True
            if not alive:
                # a track may legally outlive its obstacle by <= sense_ttl:
                # it is a stale message, so it is excluded from A/B and
                # counted separately (it must never contribute a risk onset)
                n_dead_track += 1
                t_pub = r["ttc_ctrl"].get(k, np.nan)
                if np.isfinite(t_pub) and t_pub <= float(v2cfg.ttc_pre):
                    n_dead_track_risk += 1
                continue
            n_pairs += 1
            d_pos = max(d_pos, float(np.abs(r["pub_pos"][j]
                                            - r["truth_pos"][k]).max()))
            d_vel = max(d_vel, float(np.abs(r["pub_vel"][j]
                                            - r["truth_vel"][k]).max()))
            t_pub = r["ttc_ctrl"].get(k, np.nan)
            t_tru = r["truth_ttc"][k]
            if np.isfinite(t_pub) or np.isfinite(t_tru):
                d_ttc = max(d_ttc, abs(float(t_pub) - float(t_tru)))
        for k in range(K):
            lg_ttc = lg["ttc_each"][k] if k < len(lg["ttc_each"]) else np.inf
            t_tru = r["truth_ttc"][k]
            n_log_pairs += 1
            if np.isfinite(lg_ttc) and np.isfinite(t_tru):
                d = abs(float(lg_ttc) - float(t_tru))
                d_log = max(d_log, d)
                if min(float(lg_ttc), float(t_tru)) <= 15.0:
                    d_log_near = max(d_log_near, d)
                n_log_both += 1
            elif np.isfinite(lg_ttc) != np.isfinite(t_tru):
                # the obstacle crossed its life boundary between the control
                # instant and the log instant: one side is inf by construction
                n_log_inf_boundary += 1
    # ---- C: does the one-step offset change the truth-side onset? --------
    ts = np.array([r["t"] for r in rec], dtype=float)
    t_pre = float(v2cfg.ttc_pre)
    onset_inst = {}
    for k in range(K):
        col = np.array([r["truth_ttc"][k] for r in rec], dtype=float)
        hit = np.where(np.isfinite(col) & (col <= t_pre))[0]
        onset_inst[k] = float(ts[hit[0]]) if len(hit) else np.nan
    pre = analyse_recover(res, v2cfg)
    onset_log = {k: pre.get(f"onset_truth_{k + 1}", np.nan) for k in range(K)}
    d_onset = 0.0
    for k in range(K):
        a, b = onset_inst[k], onset_log[k]
        if np.isfinite(a) and np.isfinite(b):
            d_onset = max(d_onset, abs(a - b))
    # truth-side preemption recomputed from the *instant* series
    pre_inst = _preempt_from_series(res, v2cfg, ts, onset_inst)

    def _i(x):
        try:
            x = float(x)
            return int(x) if np.isfinite(x) else -1
        except (TypeError, ValueError):
            return -1

    return dict(scenario=scenario, method=method, seed=seed,
                config_id=row["config_id"], n_ticks=n_ticks, n_pairs=n_pairs,
                n_ticks_full_animal=n_animal_full,
                n_dead_track_pairs=n_dead_track,
                n_dead_track_risk=n_dead_track_risk,
                max_pub_pos_err=d_pos, max_pub_vel_err=d_vel,
                max_ttc_pub_vs_truth_instant=d_ttc,
                max_ttc_logged_vs_truth_instant=d_log,
                max_ttc_logged_vs_truth_near_band=d_log_near,
                n_logged_pairs=n_log_both,
                n_logged_inf_boundary=n_log_inf_boundary,
                max_onset_shift_s=d_onset,
                preempt_ctrl=_i(pre.get("preempt_ctrl")),
                preempt_truth_log=_i(pre.get("preempt_truth")),
                preempt_truth_instant=pre_inst,
                preempt_legacy=_i(row.get("preempt_ok")),
                n_recover_windows=pre.get("n_recover_windows"),
                ttc_source=pre.get("ttc_source"),
                wall_s=round(float(row.get("wall_s") or 0.0), 3),
                ok_A=bool(d_pos <= TOL and d_vel <= TOL),
                ok_B=bool(d_ttc <= TOL),
                ok_C=bool(pre_inst == _i(pre.get("preempt_truth"))))


def _preempt_from_series(res, v2cfg, ts, onset):
    """Truth-side preemption flag using onsets measured at the publish instant."""
    t_pre = float(v2cfg.ttc_pre)
    tags = [l.get("tag", "normal") for l in res["logs"]]
    risk = np.array([[np.isfinite(onset.get(k, np.nan)) and
                      (ts[i] >= onset.get(k, np.nan))
                      for k in range(len(onset))] for i in range(len(ts))])
    win, start = [], None
    for i, tg in enumerate(tags):
        if tg == "recover" and start is None:
            start = i
        elif tg != "recover" and start is not None:
            win.append((start, i - 1))
            start = None
    if start is not None:
        win.append((start, len(tags) - 1))
    for (s, e) in win:
        a, b = float(ts[s]), float(ts[e])
        for k, t_on in onset.items():
            if not np.isfinite(t_on):
                continue
            if a < float(t_on) <= b + 1e-9 and not bool(risk[s, k]):
                return 1
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cells", nargs="+", default=None, metavar="SCENE:METHOD:SEED")
    ap.add_argument("--out", default=str(common.CHECK_DIR))
    args = ap.parse_args()
    cells = DEFAULT_CELLS
    if args.cells:
        cells = []
        for c in args.cells:
            s, m, sd = c.split(":")
            cells.append((s, m, int(sd)))
    common.ensure_dirs()
    v2cfg = stage3_v2cfg("M1")
    rows = []
    for (s, m, sd) in cells:
        r = probe_cell(s, m, sd, v2cfg)
        rows.append(r)
        print(f"{s:7s} {m:13s} {sd}  ticks={r['n_ticks']:5d} "
              f"pairs={r['n_pairs']:5d}  A:pos={r['max_pub_pos_err']:.1e} "
              f"B:ttc={r['max_ttc_pub_vs_truth_instant']:.1e}  "
              f"C:log_dev={r['max_ttc_logged_vs_truth_instant']:6.3f}s "
              f"onset_shift={r['max_onset_shift_s']:.2f}s  "
              f"preempt(ctrl/truth_log/truth_inst/legacy)="
              f"{r['preempt_ctrl']}/{r['preempt_truth_log']}/"
              f"{r['preempt_truth_instant']}/{r['preempt_legacy']}", flush=True)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "G_gated_ttc.csv", "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    common.write_json(out / "G_gated_ttc.json", dict(
        config_id=v2cfg.config_id, tol=TOL, cells=rows,
        all_A=bool(all(r["ok_A"] for r in rows)),
        all_B=bool(all(r["ok_B"] for r in rows)),
        all_C=bool(all(r["ok_C"] for r in rows)),
        max_ttc_pub_vs_truth_instant=float(max(
            r["max_ttc_pub_vs_truth_instant"] for r in rows)),
        max_ttc_logged_vs_truth_instant=float(max(
            r["max_ttc_logged_vs_truth_instant"] for r in rows)),
        max_onset_shift_s=float(max(r["max_onset_shift_s"] for r in rows))))
    print(f"\nwrote {out / 'G_gated_ttc.csv'}")


if __name__ == "__main__":
    main()
