# V2 Stage 4 — the 140 m / 220 m geometry, the release schedule, and the pilot screening

*Written 2026-10-27. Shipped configuration `config_id = 89fe865b2e1d`
(Stage-3 frozen tuple `T_PRE / T_DIVERT / T_REC / h = 12 / 8 / 15 / 3 s`, 70 m
obstacle range, 25 m animal range). Python 3.13.7, numpy 2.5.2.
Tests A, B and C are the **stored** evidence of `results_v2_stage4/ACCEPTANCE.txt`
(not re-run); D, E and F were completed on 2026-10-27 and are transcribed in
§5–§7 from `results_v2_stage4/ACCEPTANCE_DEF.txt`.*

---

## 0. Executive summary (to the author)

The Stage-4 geometry does the one job the V1 geometry could not do: it makes the
70 m sensing range **binding**. On the 140 m single-threat scene, dropping the
range to 40 m delays first detection by **+6.10 s** (70 m 52.90 s → 40 m 59.00 s)
and changes **263 439 state values** from t = 54.90 s; on the same scene the V1
geometry gives **0** differing values between 40 m and 70 m (bit-identical), which
is the Stage-3 result reproduced here as a control.

Three observations, stated with their limits:

**(a) On multi-threat geometry the rollout carries the benefit, not the event
layer.** In the pilot, on the 220 m sequential case, the event-only controller
(`Ours_event`) is **worse than the plain B1 baseline** — contact fraction
**C = 0.509 vs 0.296** and worst surface clearance **−1.51 vs −1.14 m** — while
the full method (`Ours_rollout`) is **best** (C = 0.046, clearance −0.22 m). The
event layer alone buys nothing on this geometry; the rollout is what buys the
**6.4× lower contact fraction against B1**. n = 3 seeds, so this is a screening
statement, not a significance claim.

**(b) The 25 m animal radius and the range gate do not create false triggers.**
On the no-obstacle 140 m scene all four methods fire **no** PRE / DIVERT /
RECOVER state, give **C = 0.000, ρ = 1.000, completion 1.00** (4/4 stored runs;
the pilot repeats it at 0/3 censored for each method).

**(c) There is no censoring in the pilot.** **0/3** censored for every
(case, method) — 36/36 runs reach the goal — so the completion denominators are
clean at these horizons. That is *not* a claim about the formal matrix: at 8
methods, three of them already hit the horizon in the Stage-4 timing sweep
(VH-Danger and B-PCBF on V2S2; B3 and B-PCBF on V2S3 — §9), so the censoring
machinery of test E will be exercised in Stage 5.

| test | status | evidence |
|---|---|---|
| A inertness | **PASS**, stored (7/7 checks) | `ACCEPTANCE.txt` §A, `A_inertness.csv` |
| B range sweep | **PASS after correcting one mis-specified assertion** | `ACCEPTANCE.txt` §B, `B_range_sweep.csv`, `B_labelling_note.md` |
| C false trigger | **PARTIAL** — 4/4 stored runs clean; the bent-path scene `V2S1N` was never reached before the suite died → **待确认** | `ACCEPTANCE.txt` §C |
| D sequential recovery | **PASS** (completed here, one seed re-run) | §5, `D_sequential_recovery.csv`, `geometry/D_state_trace_seed1000.csv` |
| E censoring | **PASS** (completed here, stored pilot + one positive control) | §6, `E_censoring.csv`, `E_censoring_control.csv` |
| F determinism / limits | **PASS** (completed here; limits read from stored detail) | §7, `F_determinism.json`, `F_limits.csv` |

---

## 1. What was reused, what was run, what was not touched

### 1.1 Reused (no re-simulation)

| artefact | use |
|---|---|
| `ACCEPTANCE.txt`, `acceptance.log`, `ACCEPTANCE.json`* | tests A, B, C as recorded (the suite was interrupted during C) |
| `A_inertness.csv` | A1/A2/A3 bit-equality rows |
| `B_range_sweep.csv` | the 40/70/100 m sweep, all three scenes |
| `pilot/pilot_runs.csv`, `pilot_summary.csv/.json`, `pilot_table.md`, `pilot.log` | the 36-run pilot, censoring counts, per-cell wall clock |
| `geometry/scene_design.csv` | Table 1 |
| `geometry/geometry_calibration.csv`, `reference_kinematics.json` | initial ranges, `T_obs`, horizon |
| `geometry/s3_spacing_sweep.csv` | independent 5/5 corroboration of test D and the release order |
| `geometry/seq_trace_V2S3_Ours_rollout_1000.csv` | first attempt at D — shown in §5.1 to be **stale** |
| `_smoke/detail.csv` | the limit-violation and binding counts of test F |
| `results_cea_timing/timing_table.csv` | the stored strict-serial V1 per-cycle cost table (cross-check) |

\* `ACCEPTANCE.json` does not exist: the suite never reached its epilogue.

### 1.2 Run here (all inside `results_v2_stage4/`)

| purpose | runs |
|---|---|
| D: one seed on the shipped 220 m scene (seed 1000) | 1 |
| E: positive control (V2S3 truncated at 100 s) | 1 |
| F: repeat cell ×2, and `runner_s4 --jobs 1` / `--jobs 4` | 4 |
| runtime: `v2.runtime_s4` (10 scenes × `Ours_rollout` + 2 × B2) | 12 |
| runtime: `v2.runtime_methods_s4` (6 remaining methods × 2 scenes) | 12 |
| runtime: `v2.parallel_efficiency_s4` (8 identical cells × jobs 1/4/12) | 24 |
| **total (second pass)** | **54** |

No V1 file was edited. No `results_*` directory other than `results_v2_stage4/`
was written. The D/E/F suite was executed twice: the first pass (6 of the runs
above) reported five "differing" pilot columns that were a **`bool` vs `"True"`
string comparison artefact in my own diff** (`success`, `completion`,
`recovery_success`, `preempt_ok`, `event_order_ok`); the comparison was fixed and
the suite re-run. The numbers below are from the second (stored) pass; the
simulations are deterministic, so the physics values are identical across the two
passes.

---

## 2. Scene design (Table 1)

16 m total width (`W = 8.0 m` half-width) everywhere; 140 m for S1/S1N/S2/S2R\*/S4\*,
220 m for S3. `e_s` is the intended encounter arc length, `lat` the normal offset,
`v` the obstacle speed, angle the encounter angle to the reverse path tangent
(0° = head-on, 90° = perpendicular), `T_enc` the intended encounter TTC. "range at
spawn" is the measured initial distance from the flock centroid
(`geometry_calibration.csv`, seed 1000).

| scenario | L [m] | 2W [m] | obs | e_s [m] | lat [m] | v [m/s] | angle [°] | T_enc [s] | t_spawn [s] | travel [m] | range at spawn [m] | outside 70 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| V2S2 | 140 | 16 | 1 | 70.0 | +5.0 | 4.0 | 29.4 | 66.65 | 0.00 | 266.60 | 315 | yes |
| V2S2R3 | 140 | 16 | 1 | 70.0 | +5.0 | 3.0 | 29.4 | 66.65 | 0.00 | 199.95 | 249 | yes |
| V2S2R2 | 140 | 16 | 1 | 70.0 | +5.0 | 5.0 | 29.4 | 66.65 | 0.00 | 333.25 | 382 | yes |
| V2S2R1 | 140 | 16 | 1 | 70.0 | +5.0 | 6.0 | 29.4 | 66.65 | 0.00 | 399.90 | 448 | yes |
| V2S3 | 220 | 16 | 1 | 70.0 | +5.0 | 3.0 | 29.4 | 46.67 | 20.27 | 140.00 | 183 | yes |
| V2S3 | 220 | 16 | 2 | 83.0 | −5.0 | 6.0 | 29.4 | 23.33 | 57.36 | 140.00 | 162 | yes |
| V2S3 | 220 | 16 | 3 | 145.0 | +5.0 | 3.0 | 29.4 | 46.67 | 98.27 | 140.00 | 184 | yes |
| V2S4S | 140 | 16 | 1 | 70.0 | +5.0 | 4.0 | 29.4 | 66.65 | 0.00 | 266.60 | 316 | yes |
| V2S4M | 140 | 16 | 1 | 70.0 | +5.0 | 4.0 | 29.4 | 66.65 | 0.00 | 266.60 | 315 | yes |
| V2S4L | 140 | 16 | 1 | 70.0 | +5.0 | 4.0 | 29.4 | 66.65 | 0.00 | 266.60 | 315 | yes |

Design notes (from `v2/scenarios_v2.py`, verified by `scene_design.csv`):

* the release rule is `start = E − û·v·T_enc` with **`T_enc ≥ 2R/v`**, so the
  obstacle is 2·70 = 140 m from its encounter point at release. Every row has
  `rule_ok = True`, and the measured range at spawn is 162–448 m, i.e. **outside
  the 70 m range by construction** (this is exactly what the V1 geometry got
  wrong: it spawned its obstacle 53 m from the flock, already inside 70 m);
* the single-obstacle family is released at `t = 0` and the obstacle is therefore
  already on its way when the run starts — the realistic case;
* **S3 uses the minimum legal lead**, `t_spawn_k = t_arr(e_k) − 2R/v_k`, so each
  obstacle starts exactly 140 m from `E_k` and the *release order* is a designed
  quantity;
* S3's three lateral offsets alternate sign (+5, −5, +5 m); 5 m is the flock's
  p95 lateral extent (4.33 m), so an unopposed crossing clips the outer 1–3 % of
  the flock;
* `V2S4S/M/L` are 18:6 / 36:12 / 60:20 animals:robots at the *same* 140 m × 16 m
  footprint (fixed-width scale family).
* Deviation from manuscript Table 14, by design: S1N is built at 140 m / 16 m
  (not 100 m / 12 m) so that width is not confounded; only the fixed-width S4
  family is built.

### 2.1 Release ordering verification

The order must be reproduced **from the trajectories**, not copied from the plan.
For the 220 m scene, per obstacle (gated, i.e. as the controller can see it):

| seed | t_spawn 1/2/3 [s] | first detect 1/2/3 [s] | risk onset (TTC≤12) 1/2/3 [s] | detect order | onset order |
|---|---|---|---|---|---|
| 1000 | 20.27 / 57.36 / 98.27 | 49.10 / 70.20 / 128.50 | 54.00 / 70.20 / 132.90 | 1<2<3 | 1<2<3 |
| 1001 | 20.27 / 57.36 / 98.27 | 49.20 / 70.90 / 129.90 | 54.70 / 70.90 / 133.70 | 1<2<3 | 1<2<3 |
| 1002 | 20.27 / 57.36 / 98.27 | 49.60 / 70.30 / 127.60 | 53.50 / 70.30 / 133.00 | 1<2<3 | 1<2<3 |
| 1003 | 20.27 / 57.36 / 98.27 | 49.30 / 70.50 / 128.90 | 53.20 / 70.50 / 132.90 | 1<2<3 | 1<2<3 |
| 1004 | 20.27 / 57.36 / 98.27 | 49.50 / 70.30 / 128.10 | 53.80 / 70.30 / 133.20 | 1<2<3 | 1<2<3 |

`order_ok = detect_order_ok = True` on **5/5** stored sweep seeds
(`geometry/s3_spacing_sweep.csv`), and `event_order_ok = True` on **12/12** pilot
sequential runs (4 methods × 3 seeds, `pilot/pilot_runs.csv`). The ordering is
therefore a property of the geometry, not of the controller.

Obstacle 2 is a hard case by design: its TTC at detection is **9.4 s**, already
below `T_PRE = 12 s`, so the event layer fires on the detection tick itself
(zero warning margin) — this is what makes the "threat during recovery" case of
§5 testable.

---

## 3. Test B — the 140/220 m geometry exercises the sensing range

### 3.1 The sweep (stored, `B_range_sweep.csv`)

| scene | R [m] | first detect [s] | PRE [s] | DIVERT [s] | C | min centre dist [m] | ticks |
|---|---|---|---|---|---|---|---|
| V2S2 (140 m, v2) | 40 | 59.00 | — (skipped) | 59.00 | 0.000 | 2.187 | 2580 |
| V2S2 | 70 | 52.90 | 53.90 | 58.00 | 0.000 | 1.731 | 2554 |
| V2S2 | 100 | 46.75 | 53.90 | 58.00 | 0.000 | 1.731 | 2554 |
| S3 (V1, 110 m, v1) | 40 | 10.85 | 17.20 | 21.40 | 0.028 | 1.540 | 2148 |
| S3 (V1) | 70 | 2.00 | 17.20 | 21.40 | 0.028 | 1.540 | 2148 |
| V2S3 (220 m, v2) | 40 | 57.65 | 58.20 | 59.60 | 0.056 | 0.643 | 4194 |
| V2S3 | 70 | 49.10 | 54.40 | 59.30 | 0.028 | 1.435 | 4220 |
| V2S3 | 100 | 41.95 | 54.40 | 59.30 | 0.000 | 2.155 | 4177 |

Pairwise trajectory comparison (state arrays, all scenes, all cells):

| pair | V2S2 (140 m) | S3 (V1 control) | V2S3 (220 m) |
|---|---|---|---|
| 40 vs 70 | **263 439 values, from tick 1098 (t = 54.90 s), max \|Δ\| 15.58** | **0 — bit-identical** | 556 947 values, tick 1136 (t = 56.80 s), max 25.98 |
| 70 vs 100 | **0 — bit-identical (saturation)** | — | 510 867 values, tick 1382 (t = 69.10 s), max 22.07 |
| 40 vs 100 | 263 439 values, tick 1098 (t = 54.90 s) | — | 554 701 values, tick 1136 (t = 56.80 s), max 28.26 |

### 3.2 Contrast with Stage 3

Stage 3 §5.5 measured the V1 geometry at 40/70/100 m: on S2 first detection moved
8.50 s → 0.05 s and on S3 10.85 s → 2.00 s, yet **C, min distance, ρ and
max_dev were identical and the trajectories were bit-identical** (verified
directly on S2/1000, 1274 ticks). The reason is the one the Stage-3 report gives:
the V1 obstacle spawns 53 m from the flock, so it is already inside 40 m at
release and the range never binds.

That is reproduced here as an explicit control: the `S3` row of
`B_range_sweep.csv` gives **0 differing values** between 40 m and 70 m on the V1
geometry (test B's fourth assertion, `[PASS]`), while the new 140 m scene gives
**263 439**. The contrast is therefore a property of the **scene**, not of the
code path — which is exactly what test A (bit-equality of the V1 scenes through
the Stage-4 runner) already established.

### 3.3 The corrected `[FAIL]` line

The stored log's line

```
[FAIL] B/and the trajectories differ (Stage-3 on the V1 geometry: bit-identical)
       -- 40/70: 263439 values; 70/100: 0 values; 40/100: 263439 values
```

is a **mis-specified assertion**, not an implementation failure. It demanded that
*all three* V2S2 pairs differ; the requirement is "**40 m ≠ 70 m**", which passes
(263 439 values, first divergence t = 54.90 s; first detection +6.10 s from 70 m
to 40 m). The 70 vs 100 m pair is **bit-identical because the 140 m single-threat
scene saturates at 70 m**: detection at 52.90 s is already 1.0 s before TTC
crosses `T_PRE` and 5.1 s before `T_DIVERT`, and the switches are TTC-threshold
events, so at 100 m (detection 46.75 s) PRE still fires at 53.90 s and DIVERT at
58.00 s — the same control sequence and therefore the same trajectory. At 40 m
detection (59.00 s) lands **after** the TTC = 12 s crossing, PRE is skipped
(`PRE = nan`, DIVERT = 59.00 s), and the run diverges.

Saturation does **not** generalise: on the 220 m three-release scene all three
pairs differ, including 70 vs 100 m (510 867 values), because there the second
release is detected at TTC ≈ 9.4 s and extra range changes *when* the RECOVER
window is preempted. The full correction, with the mechanism, is in
`results_v2_stage4/B_labelling_note.md`; an addendum at the end of
`ACCEPTANCE.txt` points to it. Corrected verdict: **B passes 4/4 of its designed
assertions.**

---

## 4. Test C — false-trigger control

Stored evidence (`ACCEPTANCE.txt` §C), no-obstacle **V2S1** (140 m × 16 m), seed
1000:

| method | PRE | DIVERT | states fired | switches | C | ended |
|---|---|---|---|---|---|---|
| B1 | — | — | none | — | 0.000 | goal |
| B2 | — | — | none | 0 | 0.000 | goal |
| Ours_event | — | — | none | 0 | 0.000 | goal |
| Ours_rollout | — | — | none | 0 | 0.000 | goal |

The pilot repeats the result on 3 seeds per method: `C = 0.000`, `ρ = 1.000`,
`max_dev/W = 0.204`, completion `1.00`, 0/3 censored
(`pilot/pilot_table.md`). The sensing gate (range-only, 25 m animal channel,
0.3 s track expiry) therefore produces **no spurious obstacle track and no
spurious PRE/DIVERT** on a scene with no obstacle — a true negative, and the
direct support for headline observation (b).

**Gap, reported honestly:** `v2/acceptance_s4.py` also sweeps the bent-path scene
`V2S1N`, but the process died before those four runs. The stored log contains
**4 of the 8 designed C runs**. Since the instruction for this pass was "do not
re-run C", the bent scene is **待确认**; the one-line command to fill it is in
§10 (hook 6). Assertion C is therefore recorded as *partial*, not as passed.

---

## 5. Test D — a new threat arrives while RECOVER is still active

Transcript: `results_v2_stage4/ACCEPTANCE_DEF.txt`; data:
`D_sequential_recovery.csv`, `geometry/D_state_trace_seed1000.csv`.

### 5.1 The stored trace cannot establish it (no re-simulation)

The stored `geometry/seq_trace_V2S3_Ours_rollout_1000.csv` (2077 control ticks)
was analysed with the same `v2.seq_trace_s4.analyse` code path:

```
first detection per obstacle: o1=49.10, o2=65.80, o3=128.00
risk onset (TTC<=T_PRE, visibility-gated): o1=54.00, o2=68.50, o3=132.10
RECOVER windows: [86.10, 87.10]; [148.20, 149.10]
new threat's risk onset inside a RECOVER window: NO
new threat detected inside a RECOVER window: NO
```

The trace file is timestamped **15:26**, i.e. **before** the 15:46 spacing sweep
that froze `e2 = 83 m` / `e3 = 145 m`; its second obstacle is detected at 65.80 s
with a first RECOVER at 86.10 s, whereas the shipped scene detects it at 70.20 s
with a first RECOVER at 69.70 s. It is a trace of a **superseded scene
definition**, so it cannot establish D — which is the documented fallback case
("if the stored trace cannot establish it, run one seed"). The file is preserved
and relabelled in the report as stale; the fresh trace is written separately by
`v2.acceptance_def_s4` and does not overwrite it.

### 5.2 One seed on the shipped scene — the case exists and preemption occurs

Run: `V2S3 / Ours_rollout / seed 1000`, config `89fe865b2e1d` (the only
simulation in test D).

```
ended goal, C=0.028, min_dist=1.44 m, rho=1.000, 2110 control ticks
first detection per obstacle: o1=49.10, o2=70.20, o3=128.50
risk onset (TTC<=T_PRE, visibility-gated): o1=54.00, o2=70.20, o3=132.90
RECOVER windows: [69.70, 70.70] (1.0 s); [82.70, 92.20] (9.5 s); [149.20, 151.20] (2.0 s)
release order (onset 1<2<3): True, (detect 1<2<3): True
o2 detected at 70.20 s in state 'recover'; risk onset 70.20 s in state 'recover'
o2 onset inside a RECOVER window: YES (69.70, 70.70)
state path after the onset: recover -> pre  => preemption by the new threat: True
```

State trace excerpt (full table in `geometry/D_state_trace_seed1000.csv`; `vis`
is per-obstacle visibility, `ttc` the per-obstacle truth TTC):

```
t=  49.10  normal   rho=1.000 ttc=( 15.48,   nan,   nan) vis=(1,0,0) detect_o1
t=  54.00  normal   rho=1.000 ttc=( 11.71,   nan,   nan) vis=(1,0,0) onset_o1
t=  69.70  recover  rho=1.000 ttc=(   nan, 10.00,   nan) vis=(1,0,0)
t=  70.20  recover  rho=0.972 ttc=(   nan,  9.71,   nan) vis=(1,1,0) detect_o2,onset_o2
t=  70.80  pre      rho=0.972 ttc=(   nan,  9.09,   nan) vis=(1,1,0)
t=  71.80  divert   rho=1.000 ttc=(   nan,  8.02,   nan) vis=(1,1,0)
t=  82.70  recover  rho=1.000 ttc=(   nan,   nan,   nan) vis=(0,1,0)
t=  92.30  normal   rho=1.000 ttc=(   nan,   nan,   nan) vis=(0,0,0)
t= 128.50  normal   rho=1.000 ttc=(   nan,   nan, 16.61) vis=(0,0,1) detect_o3
t= 132.90  pre      rho=1.000 ttc=(   nan,   nan, 11.59) vis=(0,0,1) onset_o3
```

Reading of the trace:

1. obstacle 1 clears and the controller enters **RECOVER at 69.70 s**;
2. **0.50 s later (70.20 s)** obstacle 2 enters the 70 m horizon **while the state
   is still `recover`** — and because its TTC is already 9.71 s (< `T_PRE`),
   detection and risk onset coincide: there is **no reaction margin**;
3. the next control tick leaves RECOVER for **PRE (70.80 s)** and then DIVERT
   (71.80 s): the RECOVER window is **truncated after 1.0 s** by the new threat —
   that is the *preemption*;
4. a second, full RECOVER follows once obstacle 2 is clear (82.70–92.20 s,
   9.5 s), and the third obstacle is the "recovery-complete" case (detected
   128.50 s, risk onset 132.90 s, DIVERT 137.00 s, third RECOVER 149.20 s).

So the answer to D is: **yes — a new threat arrives while RECOVER is still active
(by 0.5 s), and the controller preempts RECOVER rather than completing it.**

### 5.3 Independent 5/5 corroboration from the stored spacing sweep (not re-run)

`geometry/s3_spacing_sweep.csv` (written after the spacing was frozen, seeds
1000–1004, `e2 = 83`):

| seed | RECOVER windows [s] | detect o2 [s] | onset o2 [s] | onset inside RECOVER | order |
|---|---|---|---|---|---|
| 1000 | 69.70–70.70; 82.70–92.20; 149.20–151.20 | 70.20 | 70.20 | yes | 1<2<3 |
| 1001 | 70.60–71.60; 83.20–86.30; 150.00–153.30 | 70.90 | 70.90 | yes | 1<2<3 |
| 1002 | 69.70–70.70; 82.20–83.20; 148.50–151.40 | 70.30 | 70.30 | yes | 1<2<3 |
| 1003 | 70.30–71.30; 82.50–83.50; 148.70–150.80 | 70.50 | 70.50 | yes | 1<2<3 |
| 1004 | 69.50–70.50; 83.80–87.60; 148.90–152.50 | 70.30 | 70.30 | yes | 1<2<3 |

Risk-onset overlap **5/5**, detection overlap **5/5** — the case is not a
seed-1000 accident.

### 5.4 A metric defect this exposed (`preempt_ok`)

`metrics.summarize` computes `preempt_ok` from the **truth** TTC series
(`ttc_each`, ungated) and compares `r_on[1]` with `recover_start_1`. For seed
1000 the truth onset of obstacle 2 is **68.5 s** while the first RECOVER starts
at **69.7 s**, so `preempt_ok = False` in every pilot row — even though the
*observable* threat arrives at 70.2 s, inside the window, and really does preempt
it. The metric is measuring "would the truth onset, had it been visible, have
preceded RECOVER", not "did the controller preempt". **Stage 5 must not report
`preempt_ok` as the preemption statistic** (hook 2 in §10); `D_sequential_recovery.csv`
carries both readings.

---

## 6. Test E — censoring

Transcript: `results_v2_stage4/ACCEPTANCE_DEF.txt` §E; data `E_censoring.csv`,
`E_censoring_control.csv`.

Horizon used (scene `T_max`, equal to the reference-kinematics capture length —
i.e. the Stage-3 "≥ 120 s reference" hook is exceeded, not merely met):

| case | scene | T_max / reference capture [s] | last simulated time [s] | termination |
|---|---|---|---|---|
| no_obstacle | V2S1 (140 m) | **200** | 127.9 | goal |
| single_obstacle | V2S2 (140 m) | **200** | 133.2 | goal |
| sequential_obstacle | V2S3 (220 m) | **320** | 227.5 | goal |

The 220 m scene therefore gets 320 s, not 200 s: the no-intervention goal time is
206.1 s there, so a 200 s horizon would have censored every run.

Censored runs per (case, method) — the denominator is **all** runs, from
`pilot_runs.csv` (36 runs):

| case | method | censored / runs | ended | completion | exec_time [s] |
|---|---|---|---|---|---|
| no_obstacle | B1 / B2 / Ours_event / Ours_rollout | 0/3 each | goal | 1.00 | 123.4–125.8 (B1) |
| single_obstacle | B1 / B2 / Ours_event / Ours_rollout | 0/3 each | goal | 1.00 | — |
| sequential_obstacle | B1 / B2 / Ours_event / Ours_rollout | 0/3 each | goal | 1.00 | — |

```
36 pilot runs; 0 censored (0.0 %)
censored-with-a-completion-time or uncensored-but-unsuccessful: 0 rows
[PASS] E/the pilot has 0 censored runs and every run (36) is kept in its cell's
       denominator (n=3, from pilot_summary.json)
```

Denominators are clean *by construction*: `pilot_s4._summ` sets
`n_runs = len(sub)` over **all** rows of the cell and `n_censored = sum(censored)`
separately, and the markdown table prints `n_censored/n_runs`; `censored` is a
flag (`0 if success else 1`), never a filter. `pilot_table.md`'s `0/3` is
therefore literally "0 censored out of 3 kept".

Positive control (the pilot has nothing to demonstrate the machinery on, so one
run was deliberately truncated):

```
positive control (V2S3 truncated at 100 s): ended=time, success=False, exec_time=nan
[PASS] E/the censoring machinery works (a horizon-truncated run is flagged and
       keeps no completion time)
```

**Caveat for Stage 5:** zero censoring is a property of these 4 methods at these
horizons. In the Stage-4 timing sweep three of the eight formal methods already hit
the horizon: **VH-Danger** and **B-PCBF** on V2S2 (both `ended = time` at 200 s),
and **B3** and **B-PCBF** on V2S3 (`ended = time` at 320 s). The formal matrix
*will* censor, and the argument that censored runs stay in denominators must be
carried over verbatim.

---

## 7. Test F — determinism and joint limits

Transcript: `results_v2_stage4/ACCEPTANCE_DEF.txt` §F; data `F_determinism.json`,
`F_limits.csv`.

### 7.1 Determinism

```
F1 repeat one pilot cell twice: V2S2 / B2 / seed 1000
   2599 ticks, 634156 state values: IDENTICAL
   against the stored pilot row: all columns identical
   exec_time stored=127.59999999999489 rerun=127.59999999999489

F2 the same cell through runner_s4 at jobs=1 and jobs=4
   1 row x 183 columns (ctrl_ms excluded), mismatches=0
```

* two independent runs of the same cell are **bit-identical over 634 156 state
  values**, and every one of the stored pilot columns (including `exec_time`) is
  reproduced — the pilot archive is reproducible, not just repeatable;
* `jobs=1` vs `jobs=4` differ in **0 of 183 non-timing columns**, so the
  process-pool execution used for the pilot (jobs=8) and planned for Stage 5 does
  not perturb results. (Wall clock for the single cell: 17.86 s at jobs=1,
  18.34 s at jobs=4 — pool overhead only, as expected for one task.)

### 7.2 Limits, read from the stored `_smoke/detail.csv` (no re-run)

8 stored runs (V2S2, V2S3 × B1, B2, Ours_event, Ours_rollout, seed 1000):

| scene / method | C3 | R4 | lat | C1lo | C1hi | C2 | bind C3 | bind R4 | bind lat | bind C2 | so clr [m] | rr [m] | ρ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| V2S2 / Ours_rollout | 0 | 0 | 0 | 0 | 0 | 0 | 21 383 | 29 203 | 10 224 | 64 457 | +0.18 | 0.65 | 1.000 |
| V2S2 / B1 | 0 | 0 | 0 | 0 | 0 | 0 | 22 031 | 29 735 | 9 818 | 65 001 | −0.47 | 1.30 | 1.000 |
| V2S2 / B2 | 0 | 0 | 0 | 0 | 0 | 0 | 21 940 | 29 886 | 10 100 | 66 374 | +0.07 | 1.33 | 1.000 |
| V2S2 / Ours_event | 0 | 0 | 0 | 0 | 0 | 0 | 22 227 | 30 141 | 9 701 | 65 787 | −0.38 | 1.05 | 0.999 |
| V2S3 / Ours_rollout | 0 | 0 | 0 | 0 | 0 | 0 | 37 241 | 46 984 | 15 449 | 105 137 | +1.20 | 1.33 | 1.000 |
| V2S3 / B1 | 0 | 0 | 0 | 0 | 0 | 0 | 33 153 | 48 175 | 14 994 | 103 981 | −1.42 | 1.25 | 0.995 |
| V2S3 / B2 | 0 | 0 | 0 | 0 | 0 | 0 | 35 274 | 50 319 | 17 577 | 108 487 | −1.26 | 1.20 | 0.996 |
| V2S3 / Ours_event | 0 | 0 | 0 | 0 | 0 | 0 | 36 484 | 48 394 | 16 798 | 107 355 | −1.55 | 1.00 | 0.994 |
| **total** | **0** | **0** | **0** | **0** | **0** | **0** | **229 733** | **312 837** | **104 661** | **686 579** | | | |

* C3, R4, lateral, C1-low, C1-high and C2 violations are **0 in all 8 runs**
  (and 0 in all 36 pilot runs, `pilot_runs.csv` column check);
* the bounds are **not vacuous**: C3 binds on 229 733 animal-steps, R4 on
  312 837 robot-steps, the lateral envelope on 104 661 robot-steps, C2 on
  686 579 animal-steps;
* **C1 never binds** (`bind_C1_low_sheep = bind_C1_high_sheep = 0` in all 8 runs),
  so "C1 violations = 0" is *vacuous* on this geometry — the speed window is
  simply never active. That is reported as such, not folded into a pass claim.
* the no-contact test is not a clearance guarantee: B1/B2/Ours_event drive the
  animal–obstacle surface clearance negative on the sequential scene
  (−1.42 / −1.26 / −1.55 m), while the full method keeps it at **+1.20 m** —
  consistent with the pilot's contact fractions in §8.

---

## 8. The pilot (reproduced verbatim from `pilot/pilot_table.md`)

4 methods × 3 cases × 3 calibration seeds (1000–1002) = 36 runs,
`config_id = 89fe865b2e1d`, wall clock 196.4 s at `jobs = 8`.

| case | method | C | any-contact | min centre dist [m] | min clearance [m] | rho | max_dev/W | completion | censored |
|---|---|---|---|---|---|---|---|---|---|
| no_obstacle | B1 | 0.000 | 0.00 | nan | nan | 1.000 | 0.204 | 1.00 | 0/3 |
| no_obstacle | B2 | 0.000 | 0.00 | nan | nan | 1.000 | 0.204 | 1.00 | 0/3 |
| no_obstacle | Ours_event | 0.000 | 0.00 | nan | nan | 1.000 | 0.204 | 1.00 | 0/3 |
| no_obstacle | Ours_rollout | 0.000 | 0.00 | nan | nan | 1.000 | 0.204 | 1.00 | 0/3 |
| single_obstacle | B1 | 0.056 | 0.67 | 1.03 | -0.51 | 1.000 | 0.313 | 1.00 | 0/3 |
| single_obstacle | B2 | 0.046 | 0.67 | 0.93 | -0.65 | 1.000 | 0.349 | 1.00 | 0/3 |
| single_obstacle | Ours_event | 0.009 | 0.33 | 2.00 | 0.44 | 1.000 | 0.499 | 1.00 | 0/3 |
| single_obstacle | Ours_rollout | 0.000 | 0.00 | 2.25 | 0.70 | 1.000 | 0.377 | 1.00 | 0/3 |
| sequential_obstacle | B1 | 0.296 | 1.00 | 0.42 | -1.14 | 1.000 | 0.452 | 1.00 | 0/3 |
| sequential_obstacle | B2 | 0.241 | 1.00 | 0.44 | -1.14 | 1.000 | 0.395 | 1.00 | 0/3 |
| sequential_obstacle | Ours_event | 0.509 | 1.00 | 0.06 | -1.51 | 0.993 | 0.685 | 1.00 | 0/3 |
| sequential_obstacle | Ours_rollout | 0.046 | 0.67 | 1.34 | -0.22 | 1.000 | 0.523 | 1.00 | 0/3 |

Paired contrasts vs `Ours_rollout` (same seed, same case — from `pilot.log`):

```
single_obstacle     Ours - B1             dC = -0.056 (better on 2/3 seeds, ties 1), d(min centre dist) = +1.22 m
single_obstacle     Ours - B2             dC = -0.046 (better on 2/3 seeds, ties 1), d(min centre dist) = +1.31 m
single_obstacle     Ours - Ours_event     dC = -0.009 (better on 1/3 seeds, ties 2), d(min centre dist) = +0.25 m
sequential_obstacle Ours - B1             dC = -0.250 (better on 3/3 seeds, ties 0), d(min centre dist) = +0.92 m
sequential_obstacle Ours - B2             dC = -0.194 (better on 3/3 seeds, ties 0), d(min centre dist) = +0.90 m
sequential_obstacle Ours - Ours_event     dC = -0.463 (better on 3/3 seeds, ties 0), d(min centre dist) = +1.28 m
```

What the table says, and what it does not:

* **single threat**: the full method removes the contact entirely
  (C = 0.000 vs 0.056 for B1) and keeps +0.70 m clearance; the event layer alone
  gets close (0.009) — here the event layer *does* carry most of the benefit;
* **multiple threats** (headline (a)): the event layer alone **degrades below the
  plain baseline** (C 0.509 vs 0.296 for B1; clearance −1.51 vs −1.14 m; ρ drops
  to 0.993 and the worst deviation rises to 0.685 W), while the full method is
  **best on contacts, clearance and centre distance** (C 0.046, clearance −0.22 m,
  centre distance 1.34 m). It is *not* best on every column: its worst deviation
  (0.523 W) is larger than B1's (0.452 W) and B2's (0.395 W) — the rollout buys
  the clearance with lateral room. On this geometry the rollout — not the event
  layer — is what converts detection into avoidance;
* **caveat**: n = 3 seeds; "better on 3/3" is a sign test with p = 0.125, not
  significance. This is a screening stage; the formal matrix (seeds 2000–2029) is
  Stage 5.

---

## 9. Runtime projection for Stage 5

### 9.1 Basis (all measured on this machine, single thread, BLAS pinned)

| quantity | how measured | value |
|---|---|---|
| per-scene wall clock of `Ours_rollout` | `v2.runtime_s4`, seed 1000 | V2S1 15.92, V2S1N 9.89, V2S2 17.79, V2S2R3 17.71, V2S2R2 17.61, V2S2R1 17.52, V2S3 30.65, V2S4S 8.94, V2S4M 17.51, V2S4L 37.40 s |
| per-method per-tick cost ratio | `v2.runtime_methods_s4`, V2S2 and V2S3 | see Table 9.2 (stable across the two scenes) |
| parallel speed-up | `v2.parallel_efficiency_s4`, 8 identical cells | jobs=1 1.00×, jobs=4 **3.78×**, jobs=12 **5.58×** |

### 9.2 Per-method cost and the formal-matrix projection (2400 runs)

`per_run(scene, method) = wall(scene, Ours_rollout) × r_method`; 30 seeds × 10
scenes × 8 methods. The columns are **300 runs per method** (10 scenes × 30
seeds).

| method | wall 140 m [s] | wall 220 m [s] | ratio 140 | ratio 220 | ratio V1 (stored) | 300-run [h] | share |
|---|---|---|---|---|---|---|---|
| B-PCBF | 37.45 | 59.43 | 2.103 | 1.939 | 8.524 | **3.30** | **23.2 %** |
| VH-Path | 19.85 | 34.65 | 1.114 | 1.131 | 1.589 | 1.78 | 12.5 % |
| Ours_rollout | 17.81 | 30.65 | 1.000 | 1.000 | 1.000 | 1.59 | 11.2 % |
| VH-Danger | 17.58 | 30.49 | 0.987 | 0.995 | 0.861 | 1.57 | 11.1 % |
| B3 | 17.35 | 29.18 | 0.975 | 0.952 | — | 1.54 | 10.9 % |
| B1 | 17.00 | 29.49 | 0.954 | 0.962 | 0.460 | 1.52 | 10.7 % |
| B2 | 16.91 | 29.19 | 0.949 | 0.953 | 0.482 | 1.51 | 10.6 % |
| B-Outmost | 15.72 | 27.08 | 0.883 | 0.884 | 0.175 | 1.40 | 9.9 % |

| matrix | runs | single thread | jobs=4 (ideal) | jobs=4 (measured 3.78×) | jobs=12 (ideal) | jobs=12 (measured 5.58×) |
|---|---|---|---|---|---|---|
| formal 8 × 10 × 30 | **2400** | **14.2 h** | 3.56 h | **3.76 h** | 1.19 h | **2.55 h** |

**Assumption (explicit):** the method-to-method per-tick cost ratio is
scene-independent. The scene and team-size factors are *measured*, not assumed
(`Ours_rollout` on all 10 scenes, including the 18:6 and 60:20 scale family); the
ratio is checked on two scenes (140 m vs 220 m) and agrees within 1 % for seven of
the eight methods (worst non-PCBF: B1 0.954 vs 0.962) and within 8 % for B-PCBF
(2.103 vs 1.939), which supports it. What is *not* modelled: a
method's cost could scale differently with team size (untested for the 60:20
scene, i.e. V2S4L); that is the main uncertainty in the projection.

**The per-method difference is small, the plant dominates.** Per-tick wall clock
is ~13 ms for *every* method on V2S2 (B1 13.3, B2 13.2, B3 13.6, VH-Path 15.5,
VH-Danger 13.8, B-Outmost 12.3, Ours_rollout 13.9, B-PCBF 29.3 ms), because the
simulator + M1 animal model + sensing gate dominate and the controller is a small
part. The exceptions are **B-PCBF (2.1× the full method, 23 % of the whole
matrix)** and, second, **VH-Path (1.11×)**.

**Correction to the common assumption that "the rollout is the expensive one".**
It is not the most expensive method on this matrix. In controller-cycle cost
(stored V1 table) `Ours_rollout` is 2.27 ms vs B1's 1.04 ms — i.e. **2.2× the
cheap baselines** — but **VH-Path (3.60 ms) and B-PCBF (19.31 ms) are more
expensive**, and in wall-clock terms B-PCBF is **2.1× the rollout**. What is true
is that the rollout is the expensive *proposed* variant: it is 2.2× B1 in
controller cost, and the pilot's benefit on the sequential case comes precisely
from that planner (§8). The event-only variant `Ours_event` was not timed in this
pass (**待确认**); its cost lies between B1 and the rollout, so the 8-method
projection (which contains `Ours_rollout` but not `Ours_event`) is the
conservative one.

**The B-PCBF outlier, stated precisely.** B-PCBF is **2.10× the full method** and
2.20× B1 in Stage-4 wall-clock terms; the stored V1 strict-serial table puts it at
**18.5× B1** (19.31 vs 1.04 ms/cycle) and 8.5× `Ours_rollout`, because that table
times the *controller only*, where the CBF-QP solve is the whole cost. Both
statements are true and must be quoted with their denominator:
per **controller cycle** B-PCBF is ~18× B1; per **wall-clock run** on this
geometry it is ~2.2×, because the plant hides most of it. It is also the method
that fails to finish (V2S2 and V2S3 both `ended = time`), so its 3.30 h is
partly time spent *not* solving the task.

**Recommendation:** run the formal matrix at **jobs = 4** (~3.8 h wall,
94 % parallel efficiency measured) rather than jobs = 12: on this 6P+4E laptop
12 workers give at most 5.6× (and that number is a *lower bound*, since the probe
submitted only 8 tasks), so the 8-fold-larger worker count buys ~1.5× wall-clock
at the cost of a much noisier timing environment. Budget **4–5 h** for the
2400-run matrix, with ~20 % slack for re-runs.

**Do not quote `runtime.json → matrices`.** `v2.runtime_s4`'s built-in projection
multiplies the per-seed cost by the run count *and* divides by the worker count,
so its printed `x4 28.05 h` is inflated by 80× (2400/30). Its **measured** block
(`mean_wall_s_planning_method`, `mean_wall_s_baseline_method`, `measured`) is
correct and is what this section uses; the projection is recomputed by
`v2.project_runtime_s4` (`runtime_projection.csv/.json`).

---

## 10. Hooks Stage 5 needs

1. **Run the formal matrix on evaluation seeds 2000–2029** (untouched by Stages
   1–4): 8 methods × 10 scenes × 30 seeds = 2400 runs, **jobs = 4**, ~4 h wall
   (§9). Two smoke cells per method on V2S2 first — five of the eight methods have
   **never been run on the 140/220 m geometry** (`B3`, `VH-Path`, `VH-Danger`,
   `B-Outmost`, `B-PCBF`; the other three are `Ours_rollout`, `B1`, `B2`).
2. **Fix `preempt_ok`** (`metrics.summarize`): it compares the *truth* risk onset
   with `recover_start_1`, so it reports `False` even when the observable threat
   preempts RECOVER (§5.4). Use the visibility-gated onset (as
   `v2.seq_trace_s4.analyse` does) or report both readings; do not publish the
   current column as the preemption statistic.
3. **Re-run rule R3 / the RECOVER duration check at 220 m.** The measured RECOVER
   windows are 1.0 / 9.5 / 2.0 s (seed 1000); the 1.0 s window is a *preempted*
   window, not a recovery, so the Stage-3 `T_REC = 15 s` calibration must be
   re-derived on windows that actually closed (the 9.5 s one is the longest
   observed, still < 15 s).
4. **Keep censoring in the denominators.** The pilot has none, but at 8 methods
   VH-Danger / B-PCBF / B3 already hit the horizon (§6, §9). Report
   `n_censored/n_runs` per cell and never drop a run; completion time of a
   censored run stays NaN.
5. **Re-generate the D trace after any scene edit** (or add a scene-hash guard to
   the trace filename). The stored trace predates the spacing sweep and cost a
   fallback run; a `sha1(scene_design_row)` prefix would have caught it.
6. **Complete test C for the bent scene** `V2S1N` (4 runs, ~1 min):
   `python -X utf8 -m v2.acceptance_s4 --tests C` (it re-runs V2S1 too; the
   stored V2S1 rows are already clean, so the only new information is V2S1N).
7. **Split the B assertions**: assert "40 ≠ 70" (the design requirement) and
   *report* the 70 vs 100 pair as a saturation check, separately per scene. The
   single `all(pairs differ)` assertion is what produced the spurious `[FAIL]`
   (§3.3).
8. **Report the B-PCBF cost with an explicit denominator** (controller cycle vs
   wall-clock run, §9) and flag its non-completion; otherwise the 18× / 2.2×
   discrepancy looks like an error.
9. **Guard `runtime_s4`'s printed projection** (§9): its `matrices` block is
   arithmetically wrong; either fix it or point the manuscript at
   `v2.project_runtime_s4`.
10. **Scene-design 待确认 items carried from Stage 4**: the 25 m animal radius and
    the 70 m obstacle range are still engineering values from Table 13 with no
    ecological/hardware basis; the 5 m graze offset is calibrated to the p95
    lateral extent (4.33 m) of *this* flock at 36:12, so it must be re-measured
    for the 18:6 and 60:20 scale scenes if those are compared head-on.
11. **Metrics flow automatically**: new columns go into `v2/metrics_v2.V2_KEYS`
    and reach the CSV; `D_sequential_recovery.csv` shows the pattern for
    per-threat columns.
12. **Do not re-use the V1 geometry for any range-sensitivity claim** — the V1
    control in §3.2 reproduces bit-identical 40 vs 70 m, which is exactly why the
    claim must be made on the Stage-4 geometry.

---

## 11. Files, commands, provenance

### 11.1 New code (this pass)

```
v2/acceptance_def_s4.py        tests D, E, F (completion run; reuses stored A/B/C)
v2/runtime_methods_s4.py       per-method wall clock on V2S2 / V2S3
v2/parallel_efficiency_s4.py   jobs = 1 / 4 / 12 speed-up measurement
v2/project_runtime_s4.py       corrected 2400-run projection (replaces the broken
                               matrices block of runtime_s4)
```

`v2/acceptance_s4.py`, `v2/pilot_s4.py`, `v2/seq_trace_s4.py`, `v2/runtime_s4.py`,
`v2/calibrate_geometry_s4.py`, `v2/scenarios_v2.py`, `v2/runner_s4.py` were **not
modified**. No V1 file was modified.

### 11.2 New results (all under `results_v2_stage4/`)

```
ACCEPTANCE_DEF.txt / .json     D/E/F transcript + machine-readable verdicts
D_sequential_recovery.csv      stored trace, fresh seed, sweep corroboration
F_determinism.json             repeat + jobs diffs + limit summary
F_limits.csv                   limits read from _smoke/detail.csv
E_censoring.csv                per-run censoring table
E_censoring_control.csv        horizon-truncated positive control
B_labelling_note.md            the corrected B labelling (see also the addendum
                               appended to ACCEPTANCE.txt)
runtime.json                   per-scene Ours_rollout + B2 timing (runtime_s4)
runtime_methods.csv / .json    per-method per-tick cost + ratios
parallel_efficiency.csv/.json  jobs 1/4/12 speed-up
runtime_projection.csv/.json   the 2400-run projection
geometry/D_state_trace_seed1000.csv   the D state trace excerpt
geometry/seq_trace_V2S3_Ours_rollout_1000.csv   STALE (pre-15:46 scene) -- kept
                               for provenance, do not quote for D
```

### 11.3 Commands

```powershell
cd 01_Main_Experiment/code
$env:OMP_NUM_THREADS=1; $env:OPENBLAS_NUM_THREADS=1; $env:MKL_NUM_THREADS=1

python -X utf8 -m v2.acceptance_def_s4          # D + E + F  (149 s)
python -X utf8 -m v2.runtime_s4                 # 10-scene timing
python -X utf8 -m v2.runtime_methods_s4         # the other 6 methods
python -X utf8 -m v2.parallel_efficiency_s4     # jobs 1 / 4 / 12
python -X utf8 -m v2.project_runtime_s4         # the 2400-run projection
```

Tests A/B/C are reproduced only by the stored log (`python -m v2.acceptance_s4
--tests A B C`, ~15 min) and were deliberately not re-run here.

---

## 12. 待确认 (explicit gaps)

| item | status |
|---|---|
| Test C on the bent scene `V2S1N` | **待确认** — the stored log has 4 of the 8 designed C runs (V2S1 only); the suite died before V2S1N. Command in §10 hook 6. |
| Censoring in the formal matrix | **待确认 by design** — 0/3 in the pilot, but VH-Danger/B-PCBF/B3 already time out on these scenes; the true censoring rate must be measured, not assumed to be zero. |
| B-PCBF on the 220 m scene | **待确认** — `ended = time` at 320 s in the timing sweep; its behaviour (not just its cost) on S3 is unknown. |
| Method cost on the 60:20 scene (`V2S4L`) | **待确认** — only `Ours_rollout` was timed there (37.4 s, the most expensive scene); the per-method ratios were measured on 36:12 scenes only. |
| `Ours_event` runtime | **待确认** — not timed in this pass (the formal 8 do not include it); its cost is bounded by B1 and `Ours_rollout`. |
| `preempt_ok` semantics | **待确认** — the column is currently truth-based and reports `False` for a real preemption; which definition the manuscript should ship is an author decision (§5.4). |
| RECOVER-duration rule at 220 m | **待确认** — the measured windows (1.0 / 9.5 / 2.0 s) include a preempted window; `T_REC = 15 s` was frozen on the V1 geometry. |
| Pilot sample size | n = 3 seeds, no significance claimed; the formal matrix uses seeds 2000–2029. |
| The stale seq trace | kept as provenance; a scene-hash guard is proposed (§10 hook 5). |
