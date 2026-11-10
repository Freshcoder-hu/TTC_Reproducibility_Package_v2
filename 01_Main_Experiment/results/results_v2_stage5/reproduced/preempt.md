<!-- REPRO:preempt -->
| method | runs with RECOVER | windows | closed | preempted | open at end | preempt_ctrl = True | preempt_truth = True | legacy = True |
|---|---|---|---|---|---|---|---|---|
| Ours_rollout | 240 | 297 | 271 | 25 | 0 | 25 | 0 | 0 |
| B1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| B2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| VH-Path | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| VH-Danger | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| VH-Danger-path | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| B-Outmost | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| B-PCBF | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Preemption is only *defined* for a method that has a RECOVER state; the baselines have none, so their zeros are structural.  `preempt_ctrl` is the paper's reading (controller information set), `preempt_truth` the truth-based sensitivity variant, `legacy` the frozen `metrics.preempt_ok` column (kept for provenance only — it is **not** a preemption statistic).
