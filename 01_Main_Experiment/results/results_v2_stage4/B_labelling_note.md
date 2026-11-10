# B test — corrected labelling of the `[FAIL]` line in `ACCEPTANCE.txt`

**Scope: a test-design artefact only. Nothing was re-run; the numbers are the stored
`geometry`/`B_range_sweep.csv` and `ACCEPTANCE.txt` values.**

## What the stored log says

`results_v2_stage4/ACCEPTANCE.txt`, line 36:

```
[FAIL] B/and the trajectories differ (Stage-3 on the V1 geometry: bit-identical)
       -- 40/70: 263439 values; 70/100: 0 values; 40/100: 263439 values
```

The assertion was coded as `all(not p["identical"] for p in v2pairs)` over **all
three** V2S2 range pairs — 40/70, 70/100 and 40/100. The 70/100 pair is
bit-identical, so the `all(...)` is false and the line prints `[FAIL]`.

## What the requirement actually is

The design requirement of test B is **"40 m ≠ 70 m"**: the sweep has to show that
the 70 m sensing range is *binding* on the Stage-4 geometry, in contrast with the
V1 geometry where 40 m and 70 m were bit-identical. That pair passes:

| pair | differing state values | first divergence | max \|Δ\| | verdict |
|---|---|---|---|---|
| **V2S2 40 vs 70 m** | **263 439** | **tick 1098, t = 54.90 s** | 15.58 | **differs — requirement met** |
| V2S2 40 vs 100 m | 263 439 | tick 1098, t = 54.90 s | 15.58 | differs |
| V2S2 70 vs 100 m | 0 | — | — | **bit-identical — saturation** |

First detection on V2S2: **70 m 52.90 s → 40 m 59.00 s, +6.10 s** (and 100 m
46.75 s). The information is genuinely different and, for 40 vs 70 m, so is the
behaviour.

## Why 70 vs 100 m is identical — saturation, not an implementation failure

The 140 m single-threat scene is **saturated at 70 m**:

* at 70 m the obstacle is first detected at **52.90 s**, i.e. **1.0 s before**
  TTC crosses `T_PRE = 12 s` (53.90 s) and **5.1 s before** it crosses
  `T_DIVERT = 8 s` (58.00 s);
* at 100 m it is detected earlier (46.75 s), but the controller's switches are
  **TTC-threshold** events, not detection events: PRE still fires at 53.90 s and
  DIVERT at 58.00 s, so the control sequence is identical and therefore the
  trajectory is bit-identical;
* at 40 m the detection (59.00 s) happens **after** the TTC = 12 s crossing — the
  threat is already inside the divert band (`TTC < 8 s`) when first seen, PRE is
  skipped altogether (`PRE = nan`, DIVERT = 59.00 s = the detection tick), every
  switch is late, and the trajectory diverges from t = 54.90 s.

Saturation is therefore a property of the **140 m single-threat scene plus the
event layer's threshold structure**, and it does not generalise: on the 220 m
three-release scene **all three pairs differ**, including 70 vs 100 m (510 867
values, first divergence tick 1382, t = 69.10 s), because there the second release
is detected at TTC ≈ 9.4 s < `T_PRE` and extra range changes *when* the RECOVER
window is preempted. First detection there: 70 m 49.10 s → 40 m 57.65 s, and
100 m 41.95 s.

## Corrected verdict

* **B passes** on its two designed assertions:
  1. 40 m detects strictly later than 70 m on the 140 m scene (Δ = +6.10 s) — PASS;
  2. 40 vs 70 m trajectories differ (263 439 values from t = 54.90 s) — PASS;
  3. the V1-geometry control still reproduces bit-identical 40 vs 70 m
     (0 differing values, `S3` row of `B_range_sweep.csv`) — PASS;
  4. the 220 m scene differs on every pair (556 947 / 510 867 / 554 701) — PASS.
* The stored `[FAIL]` line is **withdrawn as mis-specified**: it asserted
  "all three pairs differ", which conflates "the range binds" with "more range
  always changes behaviour". The 70/100 m pair is reported as **saturation** and
  must not be counted as a failure.
* `ACCEPTANCE.txt` itself is left as the raw log of what ran; this note is the
  correction of record, and the addendum at the end of that file points here.
