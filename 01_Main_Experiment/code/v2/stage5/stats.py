"""Two statistical families: seed-clustered (primary) and run-level (descriptive).

WHY TWO FAMILIES
----------------
The 300 runs of one method are **not** 300 independent observations: the same
seed drives the same initial flock and the same random streams on all ten
scenes, so the ten runs of one seed share everything but the scene.  Treating
them as independent is pseudoreplication -- it inflates n from 30 to 300 and
shrinks the p-values without adding information.

* **Primary (seed-clustered).**  For each seed ``s`` (2000-2029) and endpoint
  ``e``, average ``e`` equally over the scenarios where it is defined, giving
  ``n = 30`` independent units per method (the scene is the repeated factor,
  averaged out).  Paired Wilcoxon signed-rank vs each baseline, Holm within the
  family.  Family size: 7 continuous endpoints x 7 baselines = **49** tests.
* **Binary endpoints** (arrival, joint completion) are tested with **exact
  McNemar** on the discordant pairs, never with Wilcoxon: seed-level (arrived
  on *all* ten scenarios, n = 30 units) and run-level (n = 300 pairs, reported
  as descriptive).  Family size: 2 x 7 = **14** tests per unit level.
* **Secondary (run-level, descriptive).**  The same 7 x 7 = 49 paired tests with
  the 300 (scenario, seed) pairs as units.  Reported with the pseudoreplication
  caveat and used only to show where it disagrees with the primary family.

An endpoint whose value is undefined on a scene (clearance / TTC on the two
obstacle-free scenes) is averaged over the 8 obstacle-bearing scenes; the number
of contributing scenes is reported next to every cell.  A censored run is never
dropped from the arrival / joint-completion denominators (it counts as a
non-arrival); for the time-to-goal endpoint the test is necessarily conditional
on both members of a pair having completed, and ``n_pairs`` says how many did.
"""
from __future__ import annotations

import numpy as np
from scipy import stats as _st

from v2.stage5 import common


# ---------------------------------------------------------------------------
# data access
# ---------------------------------------------------------------------------
def load_matrix(methods=None, scenarios=None, seeds=None, root=None):
    """Read the per-method Stage-5 CSVs into one long table (list of dicts)."""
    import csv
    from pathlib import Path
    root = Path(root or common.MATRIX_DIR)
    methods = methods or common.METHODS
    rows = []
    for m in methods:
        p = root / f"{m}.csv"
        if not p.exists():
            continue
        with open(p, encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                if scenarios and r.get("scenario") not in scenarios:
                    continue
                if seeds and int(r.get("seed")) not in seeds:
                    continue
                rows.append(r)
    return rows


def _num(v):
    try:
        x = float(v)
        return x if np.isfinite(x) else np.nan
    except (TypeError, ValueError):
        return np.nan


def endpoint_series(rows, method, col, scenarios=None, seeds=None):
    """``{seed: (mean over scenes, n_scenes)}`` for one method/endpoint."""
    out = {}
    for r in rows:
        if r.get("method") != method:
            continue
        sid = r.get("scenario")
        if scenarios is not None and sid not in scenarios:
            continue
        seed = int(r.get("seed"))
        if seeds is not None and seed not in seeds:
            continue
        v = _num(r.get(col))
        d = out.setdefault(seed, [])
        if np.isfinite(v):
            d.append(v)
    return {s: (float(np.mean(v)) if v else np.nan, len(v))
            for s, v in out.items()}


def binary_series(rows, method, col, mode="all10", scenarios=None):
    """Binary endpoint per unit.

    ``mode="all10"``  one unit per seed: 1 iff the endpoint holds on **every**
                      scenario of the set (the seed-level McNemar unit).
    ``mode="run"``    one unit per (scenario, seed).
    """
    out = {}
    for r in rows:
        if r.get("method") != method:
            continue
        sid = r.get("scenario")
        if scenarios is not None and sid not in scenarios:
            continue
        v = _num(r.get(col))
        b = 1 if (np.isfinite(v) and v >= 0.5) else 0
        if mode == "run":
            out[(sid, int(r.get("seed")))] = b
        else:
            out.setdefault(int(r.get("seed")), []).append(b)
    if mode == "run":
        return out
    return {s: (1 if (v and all(x == 1 for x in v)) else 0)
            for s, v in out.items()}


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
def _pair(a, b, keys=None):
    keys = sorted(set(a) & set(b)) if keys is None else keys
    x, y = [], []
    for k in keys:
        va, vb = a[k], b[k]
        if isinstance(va, tuple):
            va, vb = va[0], vb[0]
        if np.isfinite(va) and np.isfinite(vb):
            x.append(float(va))
            y.append(float(vb))
    return np.array(x), np.array(y)


def wilcoxon(x, y):
    """Paired two-sided Wilcoxon signed-rank (normal approximation)."""
    d = np.asarray(x, float) - np.asarray(y, float)
    n = len(d)
    if n == 0:
        return dict(n=0, median_diff=np.nan, hl=np.nan, p=np.nan,
                    n_better=0, n_worse=0, n_ties=0)
    nz = d[np.abs(d) > 0]
    out = dict(n=int(n), median_diff=float(np.median(d)),
               n_better=int((d < 0).sum()), n_worse=int((d > 0).sum()),
               n_ties=int((d == 0).sum()),
               hl=(float(np.median([(d[i] + d[j]) / 2.0
                                    for i in range(len(d))
                                    for j in range(i, len(d))]))
                   if n <= 200 else float(np.median(d))))
    if len(nz) == 0 or n < 6:
        out["p"] = 1.0 if len(nz) == 0 else np.nan
        return out
    try:
        res = _st.wilcoxon(d, zero_method="wilcox", correction=False,
                           alternative="two-sided", method="approx")
        out["p"] = float(res.pvalue)
    except ValueError:
        out["p"] = np.nan
    return out


def mcnemar(a, b, keys=None):
    """Exact McNemar: two-sided binomial test on the discordant pairs."""
    keys = sorted(set(a) & set(b)) if keys is None else keys
    n01 = n10 = n00 = n11 = 0
    for k in keys:
        x, y = int(a[k]), int(b[k])
        if x == 0 and y == 1:
            n01 += 1          # baseline only
        elif x == 1 and y == 0:
            n10 += 1          # ours only
        elif x == 1 and y == 1:
            n11 += 1
        else:
            n00 += 1
    n = n01 + n10
    p = 1.0 if n == 0 else float(_st.binomtest(min(n01, n10), n, 0.5,
                                               alternative="two-sided").pvalue)
    return dict(n_units=len(keys), n01=n01, n10=n10, n00=n00, n11=n11,
                n_discordant=n, p=p)


def holm(pvals):
    """Holm-Bonferroni adjusted p-values, monotone and capped at 1."""
    p = np.asarray(pvals, dtype=float)
    ok = np.isfinite(p)
    adj = np.full(len(p), np.nan)
    if not ok.any():
        return adj
    idx = np.where(ok)[0]
    order = idx[np.argsort(p[idx])]
    m = len(idx)
    running = 0.0
    for rank, i in enumerate(order):
        val = (m - rank) * p[i]
        running = max(running, val)
        adj[i] = min(1.0, running)
    return adj


# ---------------------------------------------------------------------------
# the two families
# ---------------------------------------------------------------------------
def continuous_family(rows, baseline_methods, endpoints=None, mode="seed",
                      scenarios=None, weight="all"):
    """Rows: one per (endpoint, baseline) with both raw and Holm-adjusted p."""
    endpoints = endpoints or common.ENDPOINTS
    if scenarios is None and weight == "risk":
        scenarios = common.RISK_SCENARIOS
    out = []
    for e in endpoints:
        scenes = None
        if scenarios is not None:
            scenes = [s for s in scenarios
                      if not (e["scenes"] == "obstacle"
                              and s not in common.OBSTACLE_SCENARIOS)]
        ours = endpoint_series(rows, "Ours_rollout", e["col"], scenes)
        for b in baseline_methods:
            base = endpoint_series(rows, b, e["col"], scenes)
            if mode == "seed":
                keys = sorted(set(ours) & set(base))
                x, y = _pair(ours, base, keys)
                r = wilcoxon(x, y)
            else:
                # run level: pair by (scenario, seed)
                a, bb = {}, {}
                for r0 in rows:
                    if r0.get("method") not in ("Ours_rollout", b):
                        continue
                    sid = r0.get("scenario")
                    if scenes is not None and sid not in scenes:
                        continue
                    k = (sid, int(r0.get("seed")))
                    v = _num(r0.get(e["col"]))
                    if r0["method"] == "Ours_rollout":
                        a[k] = v
                    else:
                        bb[k] = v
                x, y = _pair(a, bb)
                r = wilcoxon(x, y)
            out.append(dict(endpoint=e["key"], better=e["better"],
                            baseline=b, unit=mode, weight=weight, **r))
    return out


def binary_family(rows, baseline_methods, mode="all10", scenarios=None):
    out = []
    for e in common.BINARY_ENDPOINTS:
        ours = binary_series(rows, "Ours_rollout", e["col"], mode=mode,
                             scenarios=scenarios)
        for b in baseline_methods:
            base = binary_series(rows, b, e["col"], mode=mode,
                                 scenarios=scenarios)
            r = mcnemar(ours, base)
            out.append(dict(endpoint=e["key"], baseline=b, unit=mode, **r))
    return out


def apply_holm(rows, family_key=None):
    """Add ``p_holm`` per family (the whole list, or per ``family_key``)."""
    for r in rows:
        r.setdefault("p_holm", np.nan)
    if family_key is None:
        adj = holm([r.get("p", np.nan) for r in rows])
        for r, a in zip(rows, adj):
            r["p_holm"] = a
        return rows
    groups = {}
    for r in rows:
        groups.setdefault(r[family_key], []).append(r)
    for key, grp in groups.items():
        adj = holm([r.get("p", np.nan) for r in grp])
        for r, a in zip(grp, adj):
            r["p_holm"] = a
    return rows


def compare_families(seed_rows, run_rows):
    """Side-by-side view: where do the two families disagree at alpha = 0.05?"""
    key = lambda r: (r["endpoint"], r["baseline"])       # noqa: E731
    s = {key(r): r for r in seed_rows}
    u = {key(r): r for r in run_rows}
    out = []
    for k in sorted(set(s) & set(u)):
        a, b = s[k], u[k]
        sig_s = np.isfinite(a["p_holm"]) and a["p_holm"] < 0.05
        sig_u = np.isfinite(b["p_holm"]) and b["p_holm"] < 0.05
        out.append(dict(
            endpoint=k[0], baseline=k[1], better=a["better"],
            n_seed=a["n"], n_run=b["n"],
            med_seed=a["median_diff"], med_run=b["median_diff"],
            p_seed=a["p"], p_seed_holm=a["p_holm"],
            p_run=b["p"], p_run_holm=b["p_holm"],
            sig_seed=bool(sig_s), sig_run=bool(sig_u),
            disagree=bool(sig_s != sig_u),
            sign_flip=bool(np.isfinite(a["median_diff"])
                           and np.isfinite(b["median_diff"])
                           and a["median_diff"] * b["median_diff"] < 0
                           and min(abs(a["median_diff"]),
                                   abs(b["median_diff"])) > 1e-6)))
    return out


def family_sizes():
    return dict(
        continuous_seed=len(common.ENDPOINTS) * (len(common.METHODS) - 1),
        continuous_run=len(common.ENDPOINTS) * (len(common.METHODS) - 1),
        binary_seed=len(common.BINARY_ENDPOINTS) * (len(common.METHODS) - 1),
        binary_run=len(common.BINARY_ENDPOINTS) * (len(common.METHODS) - 1),
        n_seeds=len(common.SEEDS), n_scenarios=len(common.SCENARIOS),
        n_runs=len(common.SEEDS) * len(common.SCENARIOS))
