"""The ``preempt_ok`` fix (Stage-5 item 1) and RECOVER-window bookkeeping.

THE DEFECT
----------
``metrics.summarize`` (frozen V1 code, never edited) computes

    preempt_ok = (r_on[1] >= recover_start_1)

where ``r_on`` comes from the log's ``ttc_each`` series.  ``ttc_each`` is built
in ``v2.sim_v2`` from the **truth** obstacle positions of *every* obstacle,
whether or not the controller has detected it.  On the shipped 220 m scene
(V2S3, seed 1000) the truth onset of obstacle 2 is 68.5 s while the first
RECOVER window opens at 69.7 s, so the legacy column reports **False** -- even
though the *observable* threat (detected at 70.2 s, TTC 9.71 s) arrives inside
that window and really does preempt it.  The legacy column answers "would the
truth onset, had it been visible, have preceded RECOVER", not "did the
controller preempt".

THE FIX
-------
Preemption is a property of the controller's *information set*, so it is
computed from the TTC the controller could compute:

    gated TTC_k(t) = truth TTC_k(t)   if obstacle k is in the published
                                      sensing view at t, else UNDEFINED

The gated TTC is recorded **inside the run** at the publish instant by
:mod:`v2.stage5.probe` (one extra ``ttc_matrix`` call per control tick, outside
the controller-timing region, strictly read-only), and lands in
``res["sense_log"][i]["ttc_ctrl"]``.  Masking the *logged* ``ttc_each`` instead
would be wrong: ``sim_v2`` fills the log row one integration step *after* the
control instant, and ``v2.stage5.gated_probe`` measures the consequence -- the
logged truth TTC differs from the TTC at the publish instant by up to 5.83 s on
V2S3/1000, because near the closest approach d(TTC)/dt is large.  The probe also
shows that, at the *same* instant, the published track **is** the truth obstacle
(position and velocity bit-identical, animal channel full), so the probe's TTC
is the controller's information set exactly.

If ``ttc_ctrl`` is absent (a Stage-4 archive, or a run made without the probe),
the code falls back to masking the logged series and says so via
``ttc_source = "log_masked"``; every Stage-5 CSV row carries that flag.

Only the animal channel could break the equality (the controller sees the
animals within 25 m, not all 36).  Every run records
``sense_animal_obs_mean``; the preemption column is flagged as assumption-clean
only when that equals ``n_sheep`` (it does for every method that has a RECOVER
state -- the discrepancy is confined to B-Outmost / B-PCBF, which have no
RECOVER state at all, so the column is undefined there for a different reason).

DEFINITIONS (both reported, never mixed)
---------------------------------------
``preempt_ctrl``   (used in the paper)  a threat *k* whose **gated** risk onset
                   (first tick with the obstacle visible and TTC_k <= T_PRE)
                   falls strictly inside an existing RECOVER window
                   ``(a, b]`` and was **not** already in risk at the window
                   start ``a`` (i.e. it is new information, not the threat the
                   recovery was about).
``preempt_truth``  the same test on the ungated truth TTC (the read-out the
                   legacy column approximates).  It is the sensitivity
                   variant; it can be False where the controller really did
                   preempt, because the truth series contains threats the
                   controller could not see.
``preempt_legacy`` the frozen ``metrics.preempt_ok`` value, kept in the CSV for
                   provenance only.  **Not** a preemption statistic.

``preempt_ctrl_trunc`` additionally requires the window to have been *closed*
by the new threat (``end_tag`` in ``{pre, divert}``), which is the strongest
form ("the RECOVER window was truncated").  ``preempt_ctrl`` alone only
requires the overlap; both are reported so the weaker claim cannot be mistaken
for the stronger one.
"""
from __future__ import annotations

import numpy as np


def _vis_matrix(res, ts):
    """(T, K) visibility from ``res["sense_log"]`` (nearest-tick fallback)."""
    senses = res.get("sense_log") or []
    logs = res.get("logs") or []
    if not logs:
        return np.zeros((0, 0))
    K = len(logs[0].get("ttc_each") or [])
    vis = np.full((len(ts), K), np.nan)
    if not senses:
        return vis
    if len(senses) == len(ts):
        for i, sr in enumerate(senses):
            for k in sr.get("obs_index", []):
                if 0 <= int(k) < K:
                    vis[i, int(k)] = 1.0
            for k in range(K):
                if np.isnan(vis[i, k]):
                    vis[i, k] = 0.0
        return vis
    # fallback: align by time (the sense log is on the same control grid, but a
    # truncated/edited archive must not silently mis-align)
    s_ts = np.array([float(r["t"]) for r in senses], dtype=float)
    idx = np.array([int(np.argmin(np.abs(s_ts - t))) for t in ts])
    for i, j in enumerate(idx):
        for k in range(K):
            vis[i, k] = 1.0 if k in senses[j].get("obs_index", []) else 0.0
    return vis


def _ctrl_ttc_matrix(res, ts, K):
    """Controller-side TTC matrix from ``sense_log[i]["ttc_ctrl"]``.

    Returns ``(M, source)`` where ``M[i, k]`` is NaN when obstacle ``k`` was not
    in the controller's view at tick ``i``.  ``source`` is
    ``"publish_instant"`` when every logged tick carries the probe field, and
    ``"log_masked"`` when the (asynchronous) logged truth series had to be
    masked by the published obstacle indices instead.
    """
    senses = res.get("sense_log") or []
    M = np.full((len(ts), K), np.nan)
    have = sum(1 for sr in senses if "ttc_ctrl" in sr)
    if senses and have == len(senses) and have > 0:
        for i, sr in enumerate(senses[:len(ts)]):
            for k, v in (sr.get("ttc_ctrl") or {}).items():
                if 0 <= int(k) < K:
                    M[i, int(k)] = float(v)
        return M, "publish_instant"
    vis = _vis_matrix(res, ts)
    T, _ = _ttc_matrix(res)
    if T.size:
        M = np.where(np.nan_to_num(vis, nan=0.0) > 0, T, np.nan)
    return M, "log_masked"


def _ttc_matrix(res):
    logs = res.get("logs") or []
    if not logs:
        return np.zeros((0, 0)), 0
    K = len(logs[0].get("ttc_each") or [])
    T = np.full((len(logs), K), np.inf)
    for i, lg in enumerate(logs):
        row = lg.get("ttc_each") or []
        for k in range(min(K, len(row))):
            try:
                T[i, k] = float(row[k])
            except (TypeError, ValueError):
                T[i, k] = np.inf
    return T, K


def _first_true(mask):
    idx = np.where(mask)[0]
    return int(idx[0]) if len(idx) else None


def analyse_recover(res, v2cfg, t_pre=None):
    """RECOVER windows, gated/truth risk onset, and both preemption readings."""
    t_pre = float(v2cfg.ttc_pre if t_pre is None else t_pre)
    logs = res.get("logs") or []
    out = dict(n_recover_windows=0, n_recover_complete=0, n_recover_preempted=0,
               n_recover_censored=0, recover_dur_med=np.nan,
               recover_dur_max=np.nan, recover_first_start=np.nan,
               recover_first_end=np.nan, recover_first_dur=np.nan,
               recover_first_end_tag="", recover_first_preempted=np.nan,
               preempt_ctrl=np.nan, preempt_ctrl_trunc=np.nan,
               preempt_truth=np.nan, preempt_assume_ok=np.nan,
               preempt_ctrl_t=np.nan, preempt_ctrl_threat=np.nan,
               preempt_ctrl_win0=np.nan, preempt_ctrl_win1=np.nan,
               onset_ctrl_1=np.nan, onset_ctrl_2=np.nan, onset_ctrl_3=np.nan,
               onset_truth_1=np.nan, onset_truth_2=np.nan,
               onset_truth_3=np.nan, ttc_source="",
               _windows=[], _durations=[], _k_defined=0)
    if not logs:
        return out
    ts = np.array([float(l["t"]) for l in logs], dtype=float)
    tags = [l.get("tag", "normal") for l in logs]
    T, K = _ttc_matrix(res)
    out["_k_defined"] = int(K)
    if K == 0:
        # no obstacle at all: preemption is undefined, not False
        return out
    gated, src = _ctrl_ttc_matrix(res, ts, K)
    out["ttc_source"] = src
    risk_g = np.isfinite(gated) & (gated <= t_pre)
    risk_t = np.isfinite(T) & (T <= t_pre)

    # ---- onset per threat, both readings ---------------------------------
    onset_c, onset_t = {}, {}
    for k in range(K):
        i = _first_true(risk_g[:, k])
        onset_c[k] = ts[i] if i is not None else np.nan
        j = _first_true(risk_t[:, k])
        onset_t[k] = ts[j] if j is not None else np.nan
        if k < 3:
            out[f"onset_ctrl_{k + 1}"] = float(onset_c[k])
            out[f"onset_truth_{k + 1}"] = float(onset_t[k])
    # ---- RECOVER windows -------------------------------------------------
    win = []
    start = None
    for i, tg in enumerate(tags):
        if tg == "recover" and start is None:
            start = i
        elif tg != "recover" and start is not None:
            win.append((start, i - 1, tags[i]))
            start = None
    if start is not None:
        win.append((start, len(tags) - 1, "end"))
    windows = []
    for (s, e, end_tag) in win:
        a, b = float(ts[s]), float(ts[e])
        complete = end_tag == "normal"
        censored = end_tag == "end"
        # risk set at the window start, both readings
        risk0_c = {k for k in range(K) if bool(risk_g[s, k])}
        risk0_t = {k for k in range(K) if bool(risk_t[s, k])}

        def _intruder(onset, risk0):
            hit = []
            for k in range(K):
                t_on = onset[k]
                if not np.isfinite(t_on):
                    continue
                if not (a < float(t_on) <= b + 1e-9):
                    continue
                if k in risk0:
                    continue
                hit.append((float(t_on), k))
            return min(hit) if hit else None

        ic = _intruder(onset_c, risk0_c)
        it = _intruder(onset_t, risk0_t)
        windows.append(dict(
            start=a, end=b, dur=b - a, end_tag=end_tag, complete=complete,
            censored=censored, intruder_ctrl=ic, intruder_truth=it,
            preempted_ctrl=bool(ic is not None),
            truncated_ctrl=bool(ic is not None and end_tag in ("pre", "divert")),
            preempted_truth=bool(it is not None),
        ))
    out["_windows"] = windows
    out["n_recover_windows"] = len(windows)
    out["n_recover_complete"] = sum(1 for w in windows if w["complete"])
    out["n_recover_preempted"] = sum(1 for w in windows if w["preempted_ctrl"])
    out["n_recover_censored"] = sum(1 for w in windows if w["censored"])
    durs = [w["dur"] for w in windows if w["complete"]]
    out["_durations"] = durs
    if durs:
        out["recover_dur_med"] = float(np.median(durs))
        out["recover_dur_max"] = float(np.max(durs))
    if windows:
        w0 = windows[0]
        out["recover_first_start"] = w0["start"]
        out["recover_first_end"] = w0["end"]
        out["recover_first_dur"] = w0["dur"]
        out["recover_first_end_tag"] = w0["end_tag"]
        out["recover_first_preempted"] = int(w0["preempted_ctrl"])
    # ---- the run-level preemption readings -------------------------------
    out["preempt_ctrl"] = int(any(w["preempted_ctrl"] for w in windows))
    out["preempt_ctrl_trunc"] = int(any(w["truncated_ctrl"] for w in windows))
    out["preempt_truth"] = int(any(w["preempted_truth"] for w in windows))
    first = next((w for w in windows if w["preempted_ctrl"]), None)
    if first is not None and first["intruder_ctrl"] is not None:
        t_on, k = first["intruder_ctrl"]
        out["preempt_ctrl_t"] = t_on
        out["preempt_ctrl_threat"] = int(k + 1)
        out["preempt_ctrl_win0"] = first["start"]
        out["preempt_ctrl_win1"] = first["end"]
    return out


def assumption_ok(row, n_sheep):
    """Is the animal channel full (so the masked truth TTC *is* the controller's)?"""
    try:
        m = float(row.get("sense_animal_obs_mean"))
        fb = float(row.get("sense_animal_fallback_ticks") or 0.0)
    except (TypeError, ValueError):
        return np.nan
    if not np.isfinite(m):
        return np.nan
    return int(abs(m - n_sheep) < 1e-9 and fb == 0.0)
