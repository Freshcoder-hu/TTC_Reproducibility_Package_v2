"""One Stage-5 cell: the accepted Stage-4 runner plus measurement columns.

``run_one`` calls ``v2.runner_s4.run_cell`` unchanged (same plant, same sensing,
same controller stack, same metrics) and adds:

* the two preemption readings of :mod:`v2.stage5.preempt` (item 1 of the fix
  list) and the RECOVER-window bookkeeping;
* ``arrival`` / ``joint_completion`` as explicit 0/1 columns;
* ``max_dev_norm`` (= max_dev / W, W = 8 m), ``censored``, ``wall_s``;
* ``preempt_assume_ok`` (animal channel full => the gated TTC equals the
  controller's);
* ``run_error``: a failed cell is **kept** as a row (NaN metrics, the error
  string) so that it stays in every completion denominator and in the
  acceptance count.  A cell is never silently dropped.
"""
from __future__ import annotations

import math
import time

import numpy as np

from v2.config_v2 import stage3_v2cfg
from v2.runner_s4 import ALL_KEYS as BASE_KEYS
from v2.runner_s4 import run_cell
from v2.stage5 import probe as _probe
from v2.stage5.common import W_HALF
from v2.stage5.preempt import analyse_recover, assumption_ok

S5_KEYS = [
    "method", "arrival", "joint_completion", "censored", "max_dev_norm",
    "wall_s", "run_error",
    # preemption (item 1)
    "preempt_ctrl", "preempt_ctrl_trunc", "preempt_truth", "preempt_legacy",
    "preempt_assume_ok", "preempt_ctrl_t", "preempt_ctrl_threat",
    "preempt_ctrl_win0", "preempt_ctrl_win1",
    # RECOVER bookkeeping
    "n_recover_windows", "n_recover_complete", "n_recover_preempted",
    "n_recover_censored", "recover_dur_med", "recover_dur_max",
    "recover_first_start", "recover_first_end", "recover_first_dur",
    "recover_first_end_tag", "recover_first_preempted",
    # gated vs truth onset per threat
    "onset_ctrl_1", "onset_ctrl_2", "onset_ctrl_3",
    "onset_truth_1", "onset_truth_2", "onset_truth_3", "ttc_source",
]

HEADER = (["method", "scenario", "scenario_name", "seed", "config_id",
           "geometry"] + [k for k in BASE_KEYS if k not in
                          ("method", "scenario", "scenario_name", "seed",
                           "config_id", "geometry")] +
          [k for k in S5_KEYS if k not in BASE_KEYS])


def _f(v, default=np.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def _failure_row(scenario, method, seed, v2cfg, err, wall):
    row = dict(method=method, scenario=scenario, scenario_name="",
               seed=int(seed), config_id=v2cfg.config_id, geometry="v2",
               alg=method, run_error=err, wall_s=wall,
               arrival=np.nan, joint_completion=np.nan, censored=1,
               max_dev_norm=np.nan, preempt_ctrl=np.nan,
               preempt_ctrl_trunc=np.nan, preempt_truth=np.nan,
               preempt_legacy=np.nan, preempt_assume_ok=np.nan)
    return row


def run_one(scenario, method, seed, v2cfg=None, controller=None,
            return_extra=False, t_max=None, probe=True):
    """One (scene, method, seed) cell with the Stage-5 columns attached."""
    v2cfg = v2cfg or stage3_v2cfg("M1")
    t0 = time.perf_counter()
    err = ""
    row = res = None
    try:
        with _probe.installed(enabled=probe):
            row, res = run_cell(scenario, method, seed, v2cfg=v2cfg,
                                controller=controller, return_res=True,
                                t_max=t_max)
    except (Exception, SystemExit) as e:          # noqa: BLE001 - keep the cell
        err = f"{type(e).__name__}: {e}"
    wall = time.perf_counter() - t0
    if res is None:
        row = _failure_row(scenario, method, seed, v2cfg, err, wall)
        if return_extra:
            return row, dict(ok=False, error=err)
        return row
    pre = analyse_recover(res, v2cfg)
    wins = pre.pop("_windows", [])
    pre.pop("_durations", None)
    pre.pop("_k_defined", None)
    row = dict(row)
    row.update(pre)
    row["method"] = method
    row["arrival"] = int(bool(row.get("success")))
    row["joint_completion"] = int(bool(row.get("completion")))
    row["censored"] = 0 if row["arrival"] else 1
    row["max_dev_norm"] = _f(row.get("max_dev")) / W_HALF
    row["wall_s"] = wall
    row["run_error"] = err
    row["preempt_legacy"] = int(bool(row.get("preempt_ok")))
    row["preempt_assume_ok"] = assumption_ok(row, v2cfg.n_sheep)
    if not np.isfinite(_f(row.get("preempt_ctrl"))):
        # no obstacles / no RECOVER state: preemption is undefined, not False
        row["preempt_ctrl"] = np.nan
        row["preempt_ctrl_trunc"] = np.nan
        row["preempt_truth"] = np.nan
    if return_extra:
        return row, dict(ok=True, windows=wins, pre=pre,
                         n_sheep=v2cfg.n_sheep,
                         ended=res.get("terminated_by"),
                         sense=(res.get("v2") or {}))
    return row


def task_entry(task):
    """Process-pool entry point (module level so it can be pickled)."""
    return run_one(*task["args"], **task["kw"])
