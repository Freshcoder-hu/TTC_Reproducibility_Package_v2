# 05 — Confirmatory three-turn panel (240 runs)

This folder carries the experiment behind **Fig. 8** and **Table 11** of the
submitted article: the curved-corridor ("three-turn") confirmation panel. The
data files were copied byte-for-byte from the submission package
(`04_数据/三弯道_240/`); no value was recomputed or rounded on the way in.

## Design

| item | value |
|---|---|
| runs | **240** = 2 scenes × 4 controller rows × 30 seeds |
| seeds | **5000–5029** (30 untouched seeds, paired across methods; disjoint from every other seed block) |
| scenes | `V2S5A` — three-turn corridor, **no obstacle** (control); `V2S5B` — same corridor **with** one 4 m/s obstacle, +5 m offset at s = 120 m |
| methods | `Ours_rollout` (proposed), `B1` (path-only), `B2` (distance-reactive) — the three main rows; `VH-Path` — external reference, labelled "extension" in Table 11 |
| configuration | `config_id = b35d5401c119`, fleet 36:12, encounter fixed at `t_enc = 124.0 s` and *not* re-aligned on these seeds |
| censoring | 0 — 240/240 arrived; runs without an effective encounter stay in every denominator |

**Why 240 and not 180.** The article's confirmatory comparison is 3 methods ×
2 scenes × 30 seeds = 180 runs; the file also contains the pre-registered
external reference `VH-Path` on both scenes, which is what makes the panel
4 × 2 × 30 = **240**. `VH-Path` is reported as an extension, not as a fourth
main method. A "3 methods × 2 scenes" description of the folder is therefore
incomplete: it is 4 method rows (3 main + 1 extension).

`VH-Path` was selected as the external reference by *maximum joint completion
on the de-duplicated nine-scenario matrix*, ties broken by higher `rho` then
lower `C`.

## Files

| file | bytes | sha256 (prefix) | role |
|---|---|---|---|
| `detail.csv` | 103,400 | `75ce7965b426` | **one row per run**, 47 columns — the primary record |
| `summary.csv` | 4,844 | `0b4c4504201b` | per scene × method aggregation (mean/sd/worst) |
| `table_three_turn.csv` | 3,307 | `601d4cdd6555` | the numbers printed as **Table 11** |
| `stats_three_turn.json` | 12,269 | `34c722ebc29b` | paired sign-flip / Wilcoxon / McNemar tests and intervals |
| `traces/MANIFEST.json` | 19,262 | `39c5768c4a61` | batch description and per-file hashes of the 16 traces |
| `traces/*.npz` (16) | 154 KB – 2.23 MB | — | raw trajectories for the plan view of Fig. 8(a): 2 scenes × 4 methods × (state trace + `…__turn.npz` turn detail), **all at seed 5000** |
| `reproduce_table11.py` | — | — | tested recomputation entry (see below) |

The trace files are the traced seed of the confirmatory batch (seed 5000), not
one trace per seed: the MANIFEST records `"traced_seed": 5000`.

## Fields of `detail.csv`

Outcome:

| column | meaning |
|---|---|
| `scenario` | `V2S5A` (no obstacle) or `V2S5B` (obstacle) |
| `method` | controller id (see Design) |
| `seed` | 5000–5029 |
| `config_id` | frozen configuration hash (`b35d5401c119`) |
| `C_sampled` | **animal–obstacle contact fraction**: per run, the share of the 36 animals that touch the obstacle (values are multiples of 1/36; `0` in the obstacle-free control); averaged over seeds this is the article's `C` |
| `swept_animal_obstacle`, `swept_robot_animal`, `swept_robot_obstacle` | 0/1 flags for a swept contact of that class anywhere in the run |
| `min_clearance_so`, `…_rs`, `…_ro` | minimum surface clearance for sheep–obstacle, robot–sheep, robot–obstacle (m); `inf` = the pair never came near |
| `rho` | path-keeping / cohesion ratio (1.0 = on the path) |
| `max_dev_m` | worst lateral deviation from the path (m) |
| `arrival` | reached the goal before the horizon (1/0) |
| `joint_completion` | goal **and** cohesion both met (1/0) — the GC criterion Table 11 counts |
| `goal_arrival_s` | time to the success criterion (s) |
| `censored` | 1 if the run was right-censored (here: 0 everywhere) |

Turn geometry — the `D_turn` family:

| column | meaning |
|---|---|
| `D_turn` | **largest per-animal lateral excursion over the three bends**, normalised by the 8 m corridor half-width (dimensionless); one scalar per run |
| `D_turn1`, `D_turn2`, `D_turn3` | the same measure restricted to bend 1 / 2 / 3 |
| `E_turn` | `max(0, D_turn − 1)`, the excess over the corridor boundary; `E_turn = 0` means the whole flock stayed inside |
| `D_turn_incl_lost` | `D_turn` with animals that left the corridor included |
| `D_turn_centroid_window`, `D_turn1/2/3_centroid_window` | centroid-window variant of the same measure — **present for seed 5000 only** (8 rows: 2 scenes × 4 methods) |

Encounter diagnostics:

| column | meaning |
|---|---|
| `effective_encounter` | 1 if the obstacle actually met the flock (within 5 m of a live animal on the second bend) |
| `enc_timing_ok`, `exposure_ok`, `enc_proximity_ok` | the three conditions behind `effective_encounter` |
| `flock_cross_s120_s` | arc length at which the flock crosses `s = 120 m` |
| `obs_at_s120_s`, `obs_at_s120_flock_arc`, `obs_closest_center_s`, `obs_closest_center_lat_m` | obstacle position/time relative to the flock at the encounter |
| `encounter_lag_s` | delay between the designed and the realised encounter |
| `flock_arc_at_min_dist`, `min_dist_obs_centroid_m`, `min_dist_obs_animal_m`, `min_dist_time_s` | closest-approach geometry |
| `trace_steps`, `elapsed_s`, `error` | bookkeeping; `error` is empty in all 240 rows |

## What it backs

* **Table 11** (`tab:curved`) — the V2S5B rows. Reproduced from `detail.csv`:
  `Ours` C = 0.0028 ± 0.0112, contact in 2/30 runs, D_turn = 0.988 ± 0.187,
  E_turn = 0.045 / 0.904, GC 27/30; `B1` 0.0250 ± 0.0616, 7/30, 1.082 ± 0.520,
  0.161 / 1.947, 25/30; `B2` 0.0231 ± 0.0632, 5/30, 1.016 ± 0.316,
  0.087 / 1.340, 26/30; `VH-Path` 0.1639 ± 0.1935, 17/30, 2.052 ± 0.792,
  1.052 / 3.306, 17/30.
* **Fig. 8** (`fig:curved`) — (a) the plan view is drawn from `traces/*.npz`
  (seed 5000); (b) `C` per seed with the 30-seed mean uses `C_sampled` and
  `joint_completion` from `detail.csv`.
* The obstacle-free control V2S5A: `C = 0` in all 4 × 30 runs, 120/120
  arrivals, the three main methods bit-identical, GC 26/30 for each.

## Note on the centroid-window `D_turn`

The centroid-window variant is stored for **seed 5000 only**, so it cannot
produce a panel mean. The four V2S5B values are 0.979 (`Ours`), 1.205 (`B1`),
0.973 (`B2`) and 1.089 (`VH-Path`); they are one illustrative seed and no
ordering is inferred from them. The literal `D_turn` is the only turn metric
defined for all 30 seeds and is the one Table 11 tabulates.

## Recompute

```bash
python 05_Curved_Path_240/reproduce_table11.py
```

Reads `detail.csv` only, prints the Table 11 / Fig. 8 key numbers and fails if
any of them disagrees with the article or with `table_three_turn.csv`.
Measured runtime ≈ 0.1 s; the exact output is quoted in
[`00_Documentation/README_EN.md`](../00_Documentation/README_EN.md) §3.
