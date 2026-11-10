# V2 credibility checks — path-bias ablation, robot–animal penetration forensics and sensing-coverage verification

*Diagnostics, **not tuning**.  Every new run uses calibration seeds
**1005–1009** only.  The evaluation block 2000–2029 is read from the frozen
Stage-5 CSVs and is never re-simulated.  No V1 file, no `results_v2_stage1..5`
file and no shipped `v2/` module was edited; the only additions are
`v2/credibility/*` analysis code and `results_v2_credibility/`.  The shipped
record is the frozen Stage-3 tuple (`config_id = 89fe865b2e1d`, 36:12, M1,
animal 25 m / obstacle 70 m, exact quality, TTC 12/8/15/3 s, W = 8 m).*

## 0. Answers in one page

1. **Check A — the flock does have a preset navigation ability, but the shipped
   controller does not depend on it.**  With **zero robots**, removing the
   path-tangent bias (`goal_bias` 0.25 → 0) collapses passive path-following:
   arrival 5/5 → 1/5, mean arc length at 60 s 60.1 → 22.9 m,
   net along-path displacement 120.8 → 34.5 m.
   With **12 robots**, the same ablation leaves arrival and joint completion at
   5/5, changes 60 s progress by 0.62 m on V2S1 and
   0.73 m on V2S2 (≈1 %), leaves ρ at 1.000, and costs
   1.56 s / 1.97 s of goal time.
   On V2S2 it does weaken safety: animal–obstacle contact appears in 2/5 seeds
   (worst clearance 0.30 → -0.35 m).  The 3-point
   sweep (0.00 / 0.10 / 0.25) finds no collapse transition.  **Verdict: the bias
   is a realism limitation on what the animal model already knows, not a
   between-method confound** — all methods share the same animals — and the
   full method still herds without it on every scene tested, including the bent
   V2S1N scene.

2. **Check B — −0.48 m is not a t = 0 initialisation overlap, and it is not
   steady-state herding.**  In the frozen detail, **0 of 300** shipped-method
   runs have a robot–animal overlap at t = 0, while **272/300** have at least one
   in-run overlap; the per-run worst clearance has median 0.617 m and maximum
   0.800 m.  In the calibration replay the deep overlaps live only in the first
   **1.5–3.7 s**, while the robots cross the flock to acquire formation slots;
   overlaps during the obstacle encounter (t ≈ 67–69 s) are shallow (≤ 0.20 m).
   The robot does **not** stop or deflect in 2/3 traced pairs: it closes at
   4.2–5.7 m/s, keeps 4.1–4.3 m/s through a 0.67–0.71 m overlap, and exits the
   far side.  The execution layer checks robot–animal separation only over one
   control period with a **zero-margin** threshold, the braking supervisor is
   robot–robot only (`brake_classes = ("rr",)`), the command guard is rr/ro only,
   and the rollout cost has no robot–animal term.  **Verdict: the −0.48 m figure
   is dominated by a start-up formation-acquisition transient — an
   initial-condition artefact in origin, not t = 0 overlap and not herding — while
   the depth of the overlap is a model limitation (no robot–animal contact
   response).**

3. **Check C — most of the sensing grid was never run in a closed loop.**  The
   only non-zero quality setting ever exercised with a controller is the single
   confounded arm `S3_gated_realQ` (position error 0.2 m **and** delay 0.2 s
   **and** loss 5 %, 2 seeds × 2 scenes); 0.5 m, 0.5 s and 10 % exist only in the
   analytic observable-time sweep, and position error 0.5 m was never run at all
   (待确认).  A matched-vs-mismatched predictor comparison did not exist; the new
   2-scene × 5-seed full-loop run gives a small, direction-inconsistent delta
   (M0 vs matched M1: joint completion 0.90 vs 1.00, mean C 0.014 vs 0.022,
   worst clearance −0.62 vs −0.77 m, max_dev/W 0.459 vs 0.385).  **The
   implementation's “M2” predictor is bit-identical to “M1”** — M2 features are
   not in the predictor — so there is only one genuinely different alternative
   (M0).

---

## 1. Provenance and rules

| item | value |
|---|---|
| frozen configuration | `config_id = 89fe865b2e1d` (Stage-3/4/5 tuple) |
| new-run seeds | **1005, 1006, 1007, 1008, 1009** |
| evaluation seeds | 2000–2029, **read-only** (`results_v2_stage5/matrix/*.csv`) |
| new runs | A: 50; A supplement V2S1N: 10; B: 10 traces + 3 kinematics + 3 separation pairs (6 runs); C: 30 |
| code added | `v2/credibility/{common,check_a_bias,check_a_s1n,check_b_contacts,check_c_sensing,make_report}.py` |
| results added | `results_v2_credibility/{check_a,check_b,check_c}/` |
| untouched | all V1 files; `results_v2_stage1..5`; every shipped `v2/` module |

Commands (PowerShell, from `01_Main_Experiment/code`):

```powershell
$env:OMP_NUM_THREADS=1; $env:OPENBLAS_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
python -X utf8 -m v2.credibility.check_a_bias --jobs 4
python -X utf8 -m v2.credibility.check_a_s1n --jobs 4
python -X utf8 -m v2.credibility.check_b_contacts --jobs 5
python -X utf8 -m v2.credibility.check_b_contacts --kinematics-only
python -X utf8 -m v2.credibility.check_b_contacts --cf-only
python -X utf8 -m v2.credibility.check_c_sensing --jobs 5
python -X utf8 -m v2.credibility.make_report
```

Nothing here changes a parameter, selects a seed, or re-runs a cell to improve a
number.  The only intervention is the *diagnostic* counterfactual in Check B,
which is reported as such.

---

## 2. Check A — is the flock given a preset navigation ability?

### 2.1 Design

The only changed knob is `SimCfg.goal_bias`, the Couzin path-tangent term
(V1 value 0.25).  It is a property of the **animal model**, identical for every
method, not of the robot controller.  The 2×2 is:

| condition | path bias | robots |
|---|---|---|
| A | ON (0.25) | 0 |
| B | OFF (0.00) | 0 |
| C | ON (0.25) | 12 (shipped full method) |
| D | OFF (0.00) | 12 |

Scenes V2S1 (140 m, no obstacle) and V2S2 (140 m, single obstacle); seeds
1005–1009; plus a 3-point sweep at 12 robots (0.00 / 0.10 / 0.25) and a
supplementary C/D pair on the bent 140 m scene V2S1N (75° bend at s = 60 m).
Metrics are read from a read-only state trace: arrival, per-animal goal-region
fraction, mean along-path arc length at 60 s and at termination, net centroid
displacement, ρ, max_dev/W, and goal time (right-censored; NaN if the horizon is
hit).

### 2.2 The 2×2 table (mean over 5 seeds)

| scene | condition | bias | arrival | joint compl. | mean s @60 s [m] | mean s end [m] | net Δs [m] | animal goal frac. | ρ | max_dev/W | goal time [s] | worst clr_so [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| V2S1 | A  ON / 0 robots | 0.25 | 5/5 | 0/5 | 60.1 | 128.8 | 120.8 | 0.511 | 0.445 | 4.219 | 150.0 | n/a |
| V2S1 | B  OFF / 0 robots | 0.00 | 1/5 | 0/5 | 22.9 | 42.5 | 34.5 | 0.100 | 0.151 | 11.938 | 183.0 | n/a |
| V2S1 | C  ON / 12 robots | 0.25 | 5/5 | 5/5 | 63.5 | 128.8 | 120.8 | 0.467 | 1.000 | 0.232 | 128.2 | n/a |
| V2S1 | D  OFF / 12 robots | 0.00 | 5/5 | 5/5 | 62.9 | 128.8 | 120.8 | 0.478 | 1.000 | 0.305 | 129.8 | n/a |
| V2S1 | sweep  0.10 / 12 robots | 0.10 | 5/5 | 5/5 | 63.7 | 128.8 | 120.7 | 0.506 | 1.000 | 0.244 | 128.2 | n/a |
| V2S2 | A  ON / 0 robots | 0.25 | 5/5 | 0/5 | 60.1 | 128.8 | 120.8 | 0.500 | 0.380 | 4.572 | 150.3 | -1.51 |
| V2S2 | B  OFF / 0 robots | 0.00 | 1/5 | 1/5 | 22.9 | 42.5 | 34.5 | 0.089 | 0.164 | 11.041 | 138.5 | -1.34 |
| V2S2 | C  ON / 12 robots | 0.25 | 5/5 | 5/5 | 63.5 | 128.8 | 120.7 | 0.494 | 1.000 | 0.392 | 128.8 | 0.30 |
| V2S2 | D  OFF / 12 robots | 0.00 | 5/5 | 5/5 | 62.8 | 128.8 | 120.8 | 0.494 | 1.000 | 0.417 | 130.8 | -0.35 |
| V2S2 | sweep  0.10 / 12 robots | 0.10 | 5/5 | 5/5 | 63.7 | 128.8 | 120.8 | 0.483 | 1.000 | 0.387 | 129.2 | 0.17 |

`arrival` is the run-level goal-reached flag; `animal goal frac.` is the fraction
of animals with arc length ≥ 0.92 L **at termination** (a run ends when the
centroid reaches 0.92 L, so this is not an “ever reached” count); `worst clr_so`
is the worst animal–obstacle surface clearance (V2S2 only).  Goal time averages
the seeds that reached the goal; A/B runs that hit the horizon are right-censored.

Reading the table:

* **A vs B is the decisive passive test.**  Without robots, bias ON reaches the
  goal in 5/5 seeds and bias OFF in **1/5**; the flock covers 60.1 m vs 22.9 m by
  t = 60 s and 128.8 m vs 42.5 m by the end.  The passive flock is *not* a blank
  slate: the path-tangent term is what makes it traverse the corridor.  (The one
  B run that arrives takes 183 s on V2S1, vs a 150 s A mean, and its animals are
  spread across the corridor: ρ 0.15 vs 0.45, max_dev 11.9 W vs 4.2 W; on V2S2
  the one B arrival takes 138.5 s but records an animal–obstacle contact.)
* **C vs D is the herding test.**  With 12 robots both arms arrive 5/5 and
  complete 5/5; the 60 s progress differs by 0.62 m (V2S1) and
  0.73 m (V2S2), and the end-state progress and net displacement
  are indistinguishable.  The controller is doing the navigation, not the bias.
* On V2S2 the passive bias-ON flock actually collides with the obstacle in 2/5
  seeds (worst clearance -1.51 m) because animals do not avoid
  obstacles on their own; the robots are what convert the navigation prior into
  an obstacle-free run.

### 2.3 C vs D — the degradation is practically zero on the task, small on safety

| scene | Δarrival (C−D) | Δjoint (C−D) | Δs60 [m] (C−D) | Δnet [m] | Δρ | Δmax_dev/W | Δgoal time [s] (D−C) mean | max | ΔC (D−C) | Δworst clr_so [m] (D−C) |
|---|---|---|---|---|---|---|---|---|---|---|
| V2S1 | 0.000 | 0.000 | 0.62 | -0.005 | 0.0000 | -0.073 | 1.56 | 3.90 | 0.0000 | — |
| V2S2 | 0.000 | 0.000 | 0.73 | -0.003 | 0.0001 | -0.025 | 1.97 | 3.80 | 0.0111 | -0.65 |
| V2S1N | 0.000 | 0.000 | 0.56 | -0.006 | 0.0000 | -0.051 | 0.72 | 2.55 | 0.0000 | — |

* Δarrival = 0 and Δjoint completion = 0 on both scenes (5/5 both arms).
* D loses **0.62 m / 0.73 m** of 60 s progress (≈1 % of a 63 m half-run); Δnet
  along-path displacement is < 0.01 m.
* Δgoal time is **+1.56 s / +1.97 s** (D slower; 1.2–1.5 % of a ~128 s run),
  with per-seed differences of −1.05 to +3.90 s.
* ρ stays 1.000; D's max_dev/W is 0.073 (V2S1) / 0.025 (V2S2) larger.
* The one material safety difference is V2S2: with bias OFF, 2/5 seeds have an
  animal–obstacle contact (C = 0.028 each), and the worst clearance falls from
  0.30 m to -0.35 m.  On V2S1 (no obstacle) there is
  no safety difference to measure.

### 2.4 The 3-point sweep, and the bent-path supplement

At 12 robots the sweep (0.00, 0.10, 0.25) is non-monotone but flat: 60 s progress
63.68 m (0.10) vs 63.48 m (0.25) vs 62.86 m (0.00), arrival 5/5 at every level,
ρ = 1.000 at every level.  **There is no bias level at which herding collapses
while the robots are present**; the collapse is between 0 and 12 robots, not
between 0 and 0.25 bias.

The straight scenes cannot show whether the controller needs the bias to steer,
because the animals start with their headings already aligned with the path
tangent.  The bent-path supplement closes that hole:

| cond. | bias | arrival | joint compl. | mean s @60 s [m] | net Δs [m] | ρ | max_dev/W | goal time [s] |
|---|---|---|---|---|---|---|---|---|
| C | 0.25 | 5/5 | 5/5 | 65.6 | 74.7 | 1.000 | 0.217 | 78.1 |
| D | 0.00 | 5/5 | 5/5 | 65.0 | 74.8 | 1.000 | 0.268 | 78.8 |

Both arms arrive and complete 5/5, keep ρ = 1.000, and differ by 0.56 m at 60 s
and 0.72 s in goal time.  The robots herd the flock around a 75° bend without any
path preference in the animals.

### 2.5 Does the bias make the comparison unfair, or is it a realism limitation?

**It is a realism limitation, not a between-method confound — and the C-vs-D
numbers are what say so.**  Every method in the comparison sees exactly the same
M1 animals with the same `goal_bias`; no method gets a different animal model, so
the *relative* comparison is fair by construction.  What the bias does change is
the interpretation of the absolute result: the passive A-vs-B contrast proves
that the animal model already knows how to walk the corridor, so the paper may
not claim that the robots teach a naive flock to navigate.  But the C-vs-D
contrast shows the herding result survives removing that prior (arrival 5/5,
completion 5/5, progress within 1 %, ρ = 1.000, goal time +1.6–2.0 s), including
on the bent scene.  The one real cost is a small V2S2 safety degradation
(2/5 seeds with animal–obstacle contact, worst clearance −0.35 m).

---

## 3. Check B — robot–animal penetration forensics

### 3.1 What the frozen Stage-5 detail contains

The frozen artefact is per **run**, not per contact episode: it stores the worst
surface clearance, the number of in-run episodes, the number of t = 0
pre-existing overlaps and the first-contact time.  It does **not** store episode
durations, so the duration split below is reconstructed on the calibration block
and the per-episode durations for the evaluation seeds are marked 待确认.

| method | runs | runs with overlap | t=0 overlap runs | episodes | depth min [m] | depth median [m] | depth max [m] | first t median [s] | first t max [s] |
|---|---|---|---|---|---|---|---|---|---|
| Ours_rollout | 300 | 272 | 0 | 2383 | 0.054 | 0.617 | 0.800 | 1.84 | 2.64 |
| B1 | 300 | 272 | 0 | 2149 | 0.054 | 0.612 | 0.800 | 1.84 | 2.64 |
| B2 | 300 | 272 | 0 | 2562 | 0.054 | 0.617 | 0.800 | 1.84 | 2.64 |
| B-Outmost | 300 | 299 | 0 | 5051 | 0.106 | 0.684 | 0.800 | 1.47 | 4.67 |
| B-PCBF | 300 | 300 | 0 | 4796 | 0.059 | 0.529 | 0.799 | 13.33 | 156.55 |
| VH-Path | 300 | 0 | 0 | 0 | — | — | — | — | — |
| VH-Danger | 300 | 0 | 0 | 0 | — | — | — | — | — |
| VH-Danger-path | 300 | 0 | 0 | 0 | — | — | — | — | — |

Depth columns are surface penetration = −(min surface clearance), over runs with
a negative value (min = shallowest, max = deepest).  Key facts:

* **t = 0 initialisation overlap: 0 pairs in 300/300 shipped-method runs** (and
  0 for every method in the matrix).  The negative mean is not an initial
  overlap.
* **In-run overlap: 272/300 runs**; 2 383 episodes in total, mean 8.8 per
  affected run.  The per-run worst clearance is 0.054 m at the shallow end,
  **0.617 m median and 0.800 m maximum** — the deepest case has the two body
  centres 0.0002 m apart.
* Ours_rollout, B1 and B2 share the **same first-contact times (to 1e-9) and the
  same depth extremes** (episode counts differ: 2 383 / 2 149 / 2 562): the event
  is a property of the shared initial formation/controller start-up, not of the
  method's risk logic.
* The VH family has zero robot–animal overlap because it never closes on the
  flock (and does not do the task: ρ 0.88–0.93, joint completion 0.39–0.42).

### 3.2 Duration and phase — reconstructed on seeds 1005–1009

Across the ten calibration replays there are **59 robot–animal episodes**: 4 last ≤ 0.1 s, **55 last > 0.1 s** (i.e. persistent by the control-period definition), 27 last > 0.3 s. Durations run 0.045–0.605 s (median 0.295, p90 0.415); depths run 0.008–0.734 m (median 0.227).

| overlap class | frozen evaluation detail (300 runs) | calibration replay (59 episodes) |
|---|---|---|
| (i) initialisation at t = 0 | **0 runs / 0 pairs** (`n_preexisting_contact_rs = 0` in all 300) | **0 episodes** |
| (ii) transient, ≤ 1 control period (0.1 s) | 待确认 — per-episode durations are not stored | **4 episodes (6.8 %)** |
| (iii) persistent, > 1 control period | 待确认 for the duration; every one of the 272 affected runs has at least one in-run episode | **55 episodes (93.2 %)**, 27 of them > 0.3 s |

| scene | phase | episodes | depth median [m] | depth max [m] | duration median [s] | duration max [s] | t min [s] | t max [s] |
|---|---|---|---|---|---|---|---|---|
| V2S1 | startup (<5 s) | 27 | 0.296 | 0.734 | 0.295 | 0.605 | 1.53 | 3.31 |
| V2S2 | in-run (>=5 s) | 5 | 0.078 | 0.195 | 0.320 | 0.525 | 67.08 | 69.06 |
| V2S2 | startup (<5 s) | 27 | 0.296 | 0.734 | 0.295 | 0.605 | 1.53 | 3.31 |

The phase split is the important one.  The **start-up** phase (t < 5 s) carries
27 episodes per scene with median depth 0.296 m and max 0.734 m.  The **in-run**
phase on V2S2 (t ≈ 67–69 s, during the obstacle encounter) has only 5 episodes
across 5 seeds, all shallow (median 0.078 m, max 0.195 m).  The per-run worst is
therefore a start-up event, not a herding event; the stored first-contact times
(median 1.84 s, max 2.64 s across all 300 runs) are consistent with that reading.

Per-run reconstruction (10 calibration runs):

| scene | seed | episodes | ≤0.1 s | >0.1 s | depth max [m] | last episode [s] |
|---|---|---|---|---|---|---|
| V2S1 | 1005 | 6 | 1 | 5 | 0.715 | 3.36 |
| V2S1 | 1006 | 5 | 0 | 5 | 0.734 | 3.67 |
| V2S1 | 1007 | 4 | 0 | 4 | 0.548 | 3.12 |
| V2S1 | 1008 | 8 | 1 | 7 | 0.440 | 3.24 |
| V2S1 | 1009 | 4 | 0 | 4 | 0.532 | 3.18 |
| V2S2 | 1005 | 6 | 1 | 5 | 0.715 | 3.36 |
| V2S2 | 1006 | 5 | 0 | 5 | 0.734 | 3.67 |
| V2S2 | 1007 | 5 | 0 | 5 | 0.548 | 69.26 |
| V2S2 | 1008 | 10 | 1 | 9 | 0.440 | 68.91 |
| V2S2 | 1009 | 6 | 0 | 6 | 0.532 | 69.23 |

### 3.3 Does the robot travel through the animal?

| seed | pair r–s | t0 [s] | duration [s] | depth [m] | min centre [m] | robot speed pre [m/s] | robot speed min [m/s] | Δheading [deg] | closing pre [m/s] | closing post [m/s] | passes through |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1005 | 10–8 | 2.64 | 0.35 | 0.715 | 0.085 | 4.17 | 4.07 | -3.0 | 5.21 | -4.90 | yes |
| 1006 | 10–8 | 2.71 | 0.30 | 0.734 | 0.066 | 4.34 | 4.24 | 1.4 | 5.70 | -4.78 | yes |
| 1008 | 2–23 | 2.12 | 0.36 | 0.440 | 0.360 | 2.84 | 1.90 | -14.9 | 4.24 | -2.86 | yes |

The pattern is not a graze and not a stop.  In seeds 1005/1006 the robot is at
4.17–4.34 m/s before contact, slows only to 4.07–4.24 m/s (no braking), changes
heading by −3°/+1.4° (no deflection), closes at 5.2–5.7 m/s, reaches a
centre distance of 0.085–0.089 m (0.67–0.71 m surface penetration), and leaves
the far side with a **negative** closing rate of −4.8/−4.9 m/s.  In seed 1008 the
robot does slow (2.84 → 1.90 m/s) and turn (−14.9°), but still passes through a
0.44 m overlap.  So: no contact response in 2/3, partial response in 1/3, and a
through-pass in all three.

### 3.4 Code path — is robot–animal contact constrained, penalised, or ignored?

**Constrained only by a one-step, zero-margin hard check; not penalised in the
planner; not brake-supervised; not in the command-level guard.**  The relevant
paths:

1. `v2/config_v2.py` hard-contact geometry (lines 322–325):
   `d_rs_contact = r_robot + r_sheep` = 0.80 m — i.e. the “hard” robot–animal
   limit is exactly surface touching, with no margin.
2. `v2/feasibility_v2.py` (lines 57, 69, 140–147, 169–175): the hard-separation
   check loops over all three `CAUSE_CLASSES = ("rr", "ro", "rs")` and predicts
   one control period (0.1 s) ahead, with the animals extrapolated at constant
   velocity.
3. The **braking supervisor is robot–robot only**.  `v2/config_v2.py` lines
   177–190:
   ```
   brake_classes: tuple = ("rr",)
   #    Robot-robot only. ...
   #    Robot-animal separation is deliberately NOT braking-supervised either:
   #    herding *requires* deliberate approach to animals ...
   ```
   and `v2/feasibility_v2.py` lines 201–212 skip any class not in that tuple:
   ```
   classes = tuple(self.cfg.brake_classes)
   for cls, q, vel, d_body in (
           ("ro", ...), ("rr", ...), ("rs", ...)):
       if cls not in classes or len(q) == 0:
           continue
   ```
   so the `rs` branch is never entered.
4. The **command-level guard** (`v2/robot_safety_v2.py` lines 86–117) contains
   `guard_rr` and `guard_ro` counters only — no `guard_rs`.
5. The **rollout cost** (`v2/predictor_v2.py` lines 346–354) has one contact
   penalty, and it is animal–obstacle only:
   ```
   dok = np.linalg.norm(p[:, :, None, :] - op[:, None, :, :], axis=3)
   m_ik = (dok - r_sheep - orad[None, None, :]).min(axis=2)
   margin = np.minimum(margin, m_ik)
   ...
   contact = (margin < 0).sum(axis=1) / N
   ```
   There is no robot–animal term in the planner's objective; the robot safety
   filter in the same file (lines 242–251) handles `d_ro` only.

**Plain answer:** robot–animal contact is *nominally constrained* by the
feasibility layer's one-control-period hard check (0.8 m, zero margin), but it is
*not brake-supervised, not in the command guard, and not penalised by the
planner*.  In the first 2–3 s that check is too late: the robots are already
closing on the flock at 5 m/s, so the predicted breach is flagged only when a
stop is no longer possible (in the traced run the first `n_infeasible_rs` event
is at t = 1.5 s and most are coded S/P rather than resolved).

### 3.5 Counterfactual — does the penetration change the outcome?

Three V2S2 runs (seeds 1005, 1007, 1008).  The intervention is applied **after
the robot step and before the animal step**: every overlapping robot–animal pair
is separated by displacing the *animal* radially to a 0.05 m surface gap.  The
robot command, controller, sensing and obstacle are unchanged; the displacement
is non-physical and is reported (total 7.3–12.6 m over 8–14 correction events,
max ≈ 1.0 m per event).

| seed | arrival | joint completion | C base | C CF | clr_so base [m] | clr_so CF [m] | clr_rs base [m] | clr_rs CF [m] | rs episodes | max centroid div [m] | RMS div [m] | corrections | disp. sum [m] | max disp. [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1005 | 1→1 | 1→0 | 0.000 | 0.028 | 1.45 | -0.96 | -0.716 | -0.030 | 6→1 | 7.86 | 6.03 | 14 | 12.6 | 1.01 |
| 1007 | 1→1 | 1→1 | 0.000 | 0.000 | 1.21 | 1.00 | -0.548 | 0.001 | 5→0 | 2.54 | 1.20 | 10 | 9.0 | 0.98 |
| 1008 | 1→1 | 1→1 | 0.000 | 0.000 | 0.91 | 0.47 | -0.440 | -0.028 | 10→1 | 2.53 | 1.38 | 8 | 7.3 | 1.04 |

* **Trajectory:** 3/3 runs diverge; the flock-centroid maximum divergence is
  2.5–7.9 m and the RMS is 1.2–6.0 m.  The perturbation is not local.
* **Completion:** joint completion is unchanged in 2/3 and **lost in 1/3**
  (seed 1005: `exec_time` 126.1 s → right-censored).
* **Animal–obstacle contact:** unchanged (0) in 2/3; **appears in 1/3**
  (seed 1005: C 0 → 0.028, worst clearance +1.45 m → −0.97 m).  In the other two
  the clearance falls but stays positive (+1.21 → +1.00 m; +0.91 → +0.47 m).
* **The overlaps themselves disappear:** robot–animal episodes 6 → 1, 5 → 0,
  10 → 1.

So penetration **is causally entangled with the run** — separating the bodies
changes the trajectory in every case and changes the task outcome in one of
three — but the sign is not “penetration is harmful”: the intervention *caused*
the one contact in seed 1005.  A 1 m displacement in a 36-animal flock is a
chaotic perturbation, so this experiment establishes sensitivity, not a
benefit/harm direction.  It does rule out the strong claim that the overlaps are
causally inert.

### 3.6 Verdict on −0.48 m

* The figure is the **mean over 300 runs of each run's worst robot–animal
  surface clearance** (−0.4805 m; median −0.600 m; 197/300 worse than −0.5 m).
* It is **not** a t = 0 initial-condition overlap: 0 pre-existing pairs.
* It is **not** steady-state herding: the deep events are confined to the
  1.5–3.7 s slot-acquisition transient.  The robots are initialised on a
  variable-radius ring (`robots.slot_positions(form="ring")`: 3.0 m +
  up to 1.7·r_avo ≈ 9.8 m), so robots that start ahead of the 2.8 m-radius flock
  must drive back past it; they reach 4.1 m/s by t ≈ 2 s and contact animals
  from t ≈ 1.5 s.  Overlap episodes during the obstacle encounter are ≤ 0.20 m.
* It **is** an initial-transient artefact in origin, and a model limitation in
  depth: the plant has no robot–animal contact response and the execution layer
  only checks one control period with zero margin.  A 0.80 m penetration (body
  centres 0.0002 m apart) is not a plausible physical robot–animal interaction.
* The attribution is quantitative: **255/300 runs have a per-run worst deeper
  than 0.30 m**, while no obstacle-phase episode in the calibration replay
  exceeded 0.195 m; this attributes the per-run worst to the start-up phase for
  the bulk of the matrix.  The exact per-run attribution is not stored, so the
  shallow tail remains 待确认.

---

## 4. Check C — what the sensing experiments actually covered

### 4.1 Coverage table

| factor | value | exercised in a full controller loop? |
|---|---|---|
| position error [m] | 0.0 | yes |
| position error [m] | 0.2 | yes (confounded) |
| position error [m] | 0.5 | no (待确认) |
| delay [s] | 0.0 | yes |
| delay [s] | 0.2 | yes (confounded) |
| delay [s] | 0.5 | no (analytic only) |
| loss | 0 % | yes |
| loss | 5 % | yes (confounded) |
| loss | 10 % | no (analytic only) |
| range [m] | 40 / 70 / 100 | yes |
| range [m] | 140 | no (analytic only) |
| predictor model (env M1) | M0 (mismatched) | yes (shipped arm) |
| predictor model (env M1) | M1 (matched) | yes (this report) |
| predictor model (env M1) | M2 (mismatched) | yes (this report) — aliases M1 |
| predictor model | M0 vs M2 (shadow, environment fixed) | no (predictions only) |

The stored full-controller artefacts contain exactly one non-zero quality
setting: `S3_gated_realQ` = position error 0.2 m **+** delay 0.2 s **+** loss 5 %,
on 2 seeds × 2 scenes × 2 methods = 8 runs.  The three knobs are varied together,
never one at a time, so the 8 runs cannot separate them.  `sense_err` takes only
{0.0, 0.2} and `sense_loss` only {0.0, 0.05} in every stored full-loop CSV.

Plainly:

* **delay 0.5 s — not run in a full controller loop (待确认).**  It exists only in
  the analytic observable-time sweep (`d0.5` cells).
* **loss 10 % — not run in a full controller loop (待确认).**  It exists only in
  the analytic observable-time sweep (`l0.1` cells); the Stage-3 report already
  notes that i.i.d. 10 % loss costs only 0.23 s of warning time because 12 robots
  broadcast independently.
* **position error 0.5 m — never run at all (待确认).**  It is implemented as a
  configurable Gaussian σ, and the Stage-3 report discusses it, but no stored
  result at any seed uses it; even the analytic sweep does not store a
  position-error dimension.
* Stage-5's formal 2 400-run matrix and the Stage-4 pilot use **exact** quality
  (`sense_err = sense_delay = sense_loss = 0`).  The matrix therefore says
  nothing about quality robustness.

### 4.2 Matched vs mismatched predictor in a full controller loop

The shipped controller predicts with **M0** while the plant integrates **M1**
(`predictor_engine = "dll"`, `predictor_model = "M0"`).  No matched comparison
existed: Stage 2 only ran the predictor in shadow mode (predictions discarded)
and a qualitative live test with the plant also at M0.  We therefore ran the
full loop with the environment fixed at **M1** and the engine fixed at `v2`
(which supports M0/M1/M2), on V2S2 and V2S3 × seeds 1005–1009.

| predictor | runs | arrival | joint compl. | mean C | worst clr_so [m] | ρ | max_dev/W | censored |
|---|---|---|---|---|---|---|---|---|
| M0 | 10 | 1.00 | 0.90 | 0.0139 | -0.618 | 0.9998 | 0.4587 | 1 |
| M1 | 10 | 1.00 | 1.00 | 0.0222 | -0.768 | 0.9999 | 0.3848 | 0 |
| M2 | 10 | 1.00 | 1.00 | 0.0222 | -0.768 | 0.9999 | 0.3848 | 0 |

Per scene:

| scene | predictor | joint compl. | mean C | worst clr_so [m] | max_dev/W | median goal time [s] | censored |
|---|---|---|---|---|---|---|---|
| V2S2 | M0 | 5/5 | 0.0000 | 0.727 | 0.4163 | 126.9 | 0 |
| V2S2 | M1 | 5/5 | 0.0000 | 0.052 | 0.3276 | 127.1 | 0 |
| V2S2 | M2 | 5/5 | 0.0000 | 0.052 | 0.3276 | 127.1 | 0 |
| V2S3 | M0 | 4/5 | 0.0278 | -0.618 | 0.5011 | 206.3 | 1 |
| V2S3 | M1 | 5/5 | 0.0444 | -0.768 | 0.4420 | 206.8 | 0 |
| V2S3 | M2 | 5/5 | 0.0444 | -0.768 | 0.4420 | 206.8 | 0 |

Delta, mismatched M0 minus matched M1:

| scene | Δjoint (M0−M1) | ΔC | Δworst clr_so [m] | Δmax_dev/W | Δgoal time [s] |
|---|---|---|---|---|---|
| V2S2 | 0.00 | 0.0000 | 0.675 | 0.0887 | -0.2 |
| V2S3 | -0.20 | -0.0167 | 0.150 | 0.0591 | -0.6 |

* Arrival is 1.00 in every arm.
* Joint completion is 0.90 (M0) vs 1.00 (M1) pooled: one V2S3 seed (1005) is
  right-censored under M0.
* Mean C is equal on V2S2 (0 vs 0) and *lower* for M0 on V2S3
  (0.028 vs 0.044), while the worst clearance is *better* for M0
  (−0.62 vs −0.77 m) — the two safety indicators disagree.  These are
  single-seed events, not a trend.
* The only consistent signal is lateral deviation: M0 is 0.074 W worse than
  matched M1 pooled (V2S2 +0.089, V2S3 +0.059).
* Median goal time differs by ≤ 0.6 s per scene (V2S2 −0.2 s, V2S3 −0.6 s).

**Reading:** at n = 5 per scene the matched/mismatched predictor delta is small
and not directionally consistent; the data neither establish equivalence nor
show a material bias.  Because the comparison holds the engine at `v2`, it does
not measure the shipped DLL/M0 predictor's own error (Stage 2 measured that the
NumPy and DLL M0 engines agree to ~1e-14 over 8 steps but can diverge over the
80-step horizon).

### 4.3 Finding: the “M2” predictor is an alias of “M1”

The M1 and M2 rows above are **bit-identical on every metric**.  This is not a
coincidence: `v2/predictor_v2.py` selects the obstacle term for both

```python
use_obs = predictor_model in ("M1", "M2") and mcfg is not None and K > 0
```

and then integrates the predicted animals with the M0/M1 speed rule; no
M2-specific gain, reaction delay or stimulus-speed map is applied anywhere in
the file (the module docstring's “M2's per-animal gain is taken at its mean”
describes an intention, not code).  **Therefore the experiment supports only one
genuinely mismatched arm (M0), and any claim about an “M2 predictor” sensitivity
is unsupported (待确认).**

---

## 5. What the paper may and may not claim

### May claim

1. Under a shared M1 animal model, the shipped method's **relative** comparison
   with B1/B2 is not confounded by the path-tangent bias: removing it leaves
   arrival and joint completion at 5/5, 60 s progress within 1 %, ρ = 1.000, and
   costs only 1.6–2.0 s of goal time, on both straight scenes and on the bent
   V2S1N scene.
2. The method **still herds without the animals' path preference** — the
   controller, not the bias, is doing the navigation in C/D.
3. Robot–animal overlaps in the frozen matrix are **not t = 0 initialisation
   overlaps** (0/300), and the deep ones are confined to a **start-up
   formation-acquisition transient**; obstacle-phase overlaps are ≤ 0.20 m.
4. Sensing quality was exercised end-to-end only at the single combined setting
   (0.2 m, 0.2 s, 5 %) on a 2-seed pilot; this is a **screening** result.
5. The matched-vs-mismatched predictor full-loop delta at n = 5 is small and
   direction-inconsistent; the shipped M0/M1 mismatch is not shown to bias the
   headline result.

### May not claim (and the exact limitation wording)

**L1 — preset navigation.**
> “The animal model contains a weak path-tangent bias (`goal_bias = 0.25`).  In a
> zero-robot control this bias is what makes the flock traverse the corridor
> (arrival 1.00 vs 0.20 when it is removed; mean arc length at 60 s 60.1 m vs
> 22.9 m).  With the full 12-robot controller the same ablation leaves arrival
> and joint completion unchanged (5/5), changes 60 s progress by 0.6–0.7 m
> (≈1 %), leaves ρ = 1.000, and costs 1.6–2.0 s of goal time; on the
> single-obstacle scene it produces animal–obstacle contact in 2/5 seeds (worst
> clearance −0.35 m vs +0.30 m).  The bias does not confound the between-method
> comparison because all methods share the same animals, but the animals are not
> a blank slate, and absolute claims that the robots alone teach the flock to
> navigate are not supported by this model.”

**L2 — robot–animal contact.**
> “Robot–animal body overlap is not prevented.  In the frozen Stage-5 detail
> 272/300 runs contain at least one overlap (0 at t = 0; 2 383 episodes); the
> per-run worst surface clearance has median 0.62 m and maximum 0.80 m.  The deep
> overlaps occur only during the first 1.5–3.7 s, while the robots cross the
> flock to acquire their formation slots; the overlaps observed during obstacle
> avoidance (t ≈ 67–69 s) are shallow (≤ 0.20 m).  The −0.48 m census figure is
> the mean of per-run worst-case clearance and is dominated by this start-up
> transient, not by steady-state herding.  The execution layer checks
> robot–animal separation over one 0.1 s interval with a zero-margin threshold;
> the braking supervisor covers robot–robot only; and the rollout cost contains
> no robot–animal contact term.  The overlaps should therefore be reported as a
> model/initial-formation limitation, not as a measured herding risk.”

**L3 — sensing robustness.**
> “The observation-quality grid was not exercised end-to-end.  Position error
> 0.2 m, delay 0.2 s and loss 5 % were run only as one combined arm on 2 seeds ×
> 2 scenes; delay 0.5 s and loss 10 % exist only in the analytic observable-time
> sweep; position error 0.5 m was never simulated.  The paper must not claim
> robustness at 0.5 m / 0.5 s / 10 %, and the 0.2 m / 0.2 s / 5 % arm is a
> screening result, not a factorial sensitivity analysis (待确认).”

**L4 — predictor mismatch.**
> “The shipped controller predicts with M0 while the plant is M1.  A matched
> (M1) vs mismatched (M0) full-loop comparison on 2 scenes × 5 calibration seeds
> gives a small, direction-inconsistent delta (joint completion 0.90 vs 1.00;
> mean C 0.014 vs 0.022; worst clearance −0.62 vs −0.77 m; max_dev/W 0.459 vs
> 0.385).  At this sample size the data do not establish equivalence, and the
> implementation's ‘M2 predictor’ is bit-identical to M1, so no M2 sensitivity
> claim can be made (待确认).”

**L5 — what the −0.48 m is not.**
> “The −0.48 m figure must not be presented as evidence of steady-state
> robot–animal collision avoidance performance, and it must not be presented as
> an initial-condition artefact either: it is zero at t = 0, and it is a start-up
> transient caused by the formation-acquisition geometry.  The honest description
> is ‘robot–animal overlap during the initial slot-acquisition transient, with a
> zero-margin one-step constraint and no contact response’.”

---

## 6. 待确认 (explicit gaps)

| item | status |
|---|---|
| Per-episode duration of every evaluation-seed overlap | 待确认 — the frozen detail stores only per-run counts and worst clearance; durations were reconstructed on seeds 1005–1009 (59 episodes) and the evaluation-seed split is inferred, not observed |
| Position error 0.5 m in a full loop | 待确认 — never run at any seed |
| Delay 0.5 s / loss 10 % in a full loop | 待确认 — analytic observable-time sweep only |
| M2 predictor sensitivity | 待确认 — the M2 predictor is bit-identical to M1 in `v2/predictor_v2.py` |
| Full factorial quality grid | 待确认 — only the combined 0.2/0.2/5 % arm exists |
| Counterfactual sign | 待确认 — 3 runs show causal sensitivity but the intervention is non-physical (up to ~1 m animal displacement), so it cannot establish whether penetration helps or harms |
| Bias on non-straight, non-V2S1N geometries | not tested here; V2S1N is the only bent scene |
| Shipped DLL/M0 predictor vs v2/M0 | not measured here; Stage 2 reports engine agreement to 1e-14 over 8 steps and divergence over 80 steps |

## 7. Files

| path | what |
|---|---|
| `v2/credibility/common.py` | run helper, NoOp zero-robot controller, progress metrics |
| `v2/credibility/check_a_bias.py` | the 2×2 + 3-point sweep (50 runs) |
| `v2/credibility/check_a_s1n.py` | bent-path C/D supplement (10 runs) |
| `v2/credibility/check_b_contacts.py` | frozen-detail classification, episode reconstruction, kinematics, counterfactual |
| `v2/credibility/check_c_sensing.py` | coverage table + matched/mismatched predictor (30 runs) |
| `v2/credibility/make_report.py` | this document |
| `results_v2_credibility/check_a/runs.csv` | Check A rows |
| `results_v2_credibility/check_a/supplement_v2s1n.csv` | bent-path rows |
| `results_v2_credibility/check_b/existing_summary.csv` | per-method frozen-detail classification |
| `results_v2_credibility/check_b/phase_distribution.csv` | start-up vs in-run episodes |
| `results_v2_credibility/check_b/episodes.csv` | 59 calibration episodes |
| `results_v2_credibility/check_b/worst_pair_kinematics.csv` | 3 traced worst pairs |
| `results_v2_credibility/check_b/counterfactual.csv` | 3 separation counterfactuals |
| `results_v2_credibility/check_c/coverage.csv` | sensing coverage |
| `results_v2_credibility/check_c/predictor_runs.csv` | 30 matched/mismatched runs |
| `results_v2_credibility/check_c/predictor_summary.csv` | predictor deltas |
