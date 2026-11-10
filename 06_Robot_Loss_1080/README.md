# 06 — Robot-loss sensitivity panel (1,080 runs)

This folder carries the experiment behind **Fig. 9** and **Table 12** of the
submitted article: the robot-loss sensitivity panel at the 36:12 configuration.
The five CSV files were copied byte-for-byte from the submission package
(`04_数据/机器人损毁_1080/`); no value was recomputed or rounded on the way in.

## Design

| item | value |
|---|---|
| runs | **1,080** = 3 methods × 3 scenes × 4 conditions × 30 paired seeds |
| seeds | **2000–2029** (the frozen main-evaluation seed block, paired across conditions) |
| methods | `Ours_rollout` (proposed), `B1` (path-only), `B2` (distance-reactive) |
| scenes | `V2S1` — normal path, no obstacle; `V2S2` — single obstacle; `V2S3` — sequential risk, three threats |
| conditions | **F0** no failure; **F1** one robot disabled at 0.35 of the horizon; **F2** two robots disabled at 0.35 and 0.45; **F3** one robot disabled 2 s before the first risk onset of the paired F0 run |
| fleet | 36:12 (36 animals, 12 robots) |
| censoring | 0 — 1,080/1,080 arrived, no timeout, no right-censored run |

A disabled robot is zeroed and its body stays in the contact checks and in the
animals' obstacle-avoidance term, so the plant still sees twelve robots. `V2S1`
has no obstacle and therefore no risk onset: its F3 cells run the F1 schedule
and are flagged `f3_applicable = 0` (they reproduce the V2S1-F1 rows on every
deterministic column). The F0 arm reproduces the frozen main matrix bit for bit.

## Files

| file | bytes | sha256 (prefix) | role |
|---|---|---|---|
| `detail.csv` | 2,774,904 | `5e7d6dbcaf83` | **one row per run**, 347 columns — the primary record |
| `summary.csv` | 48,643 | `1bd95b64c071` | per method × scene × condition aggregation (36 rows), with interval columns |
| `table_failure.csv` | 7,433 | `df35ea7bf6fc` | the numbers behind **Table 12** (36 rows) |
| `attribution.csv` | 49,339 | `76e822aa9359` | F0 → condition decomposition of the contact episodes (36 rows, `_F0` / `_C` / `d_` column triples) |
| `stats_failure.csv` | 15,174 | `4a1b09f68b6c` | paired tests: family, endpoint, n01/n10/n00/n11, median difference, p, Holm |
| `reproduce_table12.py` | — | — | tested recomputation entry (see below) |

## Fields of `detail.csv` (the ones that back Table 12 / Fig. 9)

| column | meaning |
|---|---|
| `method`, `scenario`, `scenario_name`, `seed`, `condition` | cell identifiers (`scenario_name` spells out the scene geometry) |
| `config_id` | frozen configuration hash |
| `collision_rate` | **animal–obstacle contact fraction `C`**: per run, the share of the 36 animals that touch an obstacle (multiples of 1/36); averaged over 30 seeds it is the "Obstacle contact [%]" column of Table 12 and the series in Fig. 9(a) |
| `arrival` | reached the goal region (1/0); 1 in all 1,080 runs |
| `joint_completion` | goal **and** cohesion both met (1/0) — the GC criterion Table 12 reports |
| `max_dev`, `max_dev_norm` | worst lateral deviation in metres and divided by the 8 m corridor half-width (`= max_dev_w` in `summary.csv`, the `devW` column of Table 12) |
| `timeout`, `censored` | 0 everywhere |
| `f3_applicable` | 0 for V2S1-F3 (no risk onset to time the failure against), 1 otherwise |
| `any_contact_rs`, `any_contact_ro`, `any_contact_rr`, `any_contact_so`, `any_contact_ss` | 0/1 run-level contact flags by class: robot–sheep, robot–obstacle, robot–robot, sheep–obstacle, sheep–sheep |
| `n_contact_events_*` | contact episodes of each class (the source of the `epi_*` means in `summary.csv`) |
| `n_preexisting_contact_*` | episodes already present at t = 0, excluded from the new-contact counts |
| `recovery`, `recovery_success`, `switch_count`, `role_switches`, `ctrl_ms` | recovery time, recovery success, role switches, control cycle time |
| `t_pre_on`, `t_divert_on`, `t_dist_trigger`, `t_risk_on`, `ttc_lead` | event-timing columns (PRE / DIVERT / distance trigger / risk onset, and the lead time) |

`summary.csv` adds the 95 % interval columns (`arrival_lo/hi`, `joint_lo/hi`,
`contact_lo/hi`, `epi_*_lo/hi`) and the per-cell means (`epi_robot`,
`epi_rs`, `rho`, `max_dev_w`).

> **Column-name warning.** In `table_failure.csv` the Table 12 "Obstacle
> contact [%]" column is called **`animals_lost_pct`** (it equals
> `100 × mean(collision_rate)`), while that file's `contact_rate` column is
> **not** the animal–obstacle `C` — it reads 1.0 even in the obstacle-free
> V2S1. Use `animals_lost_pct` or `detail.csv:collision_rate`; the
> recomputation script does the latter.

## What it backs

* **Table 12** (`tab:loss`) — the proposed controller's 12 scene × condition
  rows (GC %, obstacle contact %, episodes/run, devW). Reproduced from
  `detail.csv`: GC 86.7 / 83.3 / 76.7 / 83.3 % on V2S1, 90.0 / 86.7 / 80.0 /
  86.7 % on V2S2 and 76.7 / 73.3 / 73.3 / 70.0 % on V2S3 for F0 → F3;
  obstacle contact 0.00 % on V2S1, 0.56 / 0.56 / 0.56 / 0.46 % on V2S2 and
  1.67 / 3.15 / 3.24 / 6.94 % on V2S3.
* **Fig. 9** (`fig:loss`) — (a) animals with obstacle contact by condition
  (`collision_rate`), (b) worst lateral deviation `devW` (`max_dev_norm`),
  (c) robot-involving contact episodes split surviving / disabled body
  (`n_contact_events_*` and `attribution.csv`), (d) GC completion by method
  (`joint_completion`).
* The headline count: **1,080 of 1,080 arrivals**, no timeout and no censored
  run.

## Recompute

```bash
python 06_Robot_Loss_1080/reproduce_table12.py
```

Reads `detail.csv` only, prints the V2S3 F0 → F3 contact series, the GC series,
the arrival count and a cross-check against `table_failure.csv`, and fails if
any of them disagrees with the article. Measured runtime ≈ 0.2 s; the exact
output is quoted in
[`00_Documentation/README_EN.md`](../00_Documentation/README_EN.md) §3.
