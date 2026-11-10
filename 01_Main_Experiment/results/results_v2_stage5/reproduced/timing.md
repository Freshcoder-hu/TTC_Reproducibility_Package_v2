<!-- REPRO:timing -->
| batch (method) | cells | wall [min] | s/cell | censored | errors |
|---|---|---|---|---|---|
| Ours_rollout | 300 | 19.7 | 3.94 | 0 | 0 |
| B1 | 300 | 15.9 | 3.19 | 0 | 0 |
| B2 | 300 | 15.4 | 3.09 | 0 | 0 |
| VH-Path | 300 | 22.4 | 4.49 | 39 | 0 |
| VH-Danger | 300 | 18.8 | 3.75 | 25 | 0 |
| VH-Danger-path | 300 | 18.4 | 3.69 | 15 | 0 |
| B-Outmost | 300 | 15.4 | 3.08 | 23 | 0 |
| B-PCBF | 300 | 53.7 | 10.75 | 275 | 0 |

Measured total for the 2400-run matrix at `jobs = 12`: **3.00 h** (10793 s).  Per-cell wall clocks are measured under 12-way concurrency; the clean serial per-cycle cost is the `cycle [ms]` column of the timing sample (`checks/speedup.csv`).

| probe | runs | jobs | wall [s] | s/run | speed-up | efficiency |
|---|---|---|---|---|---|---|
| V2S2/Ours_rollout/2000 | 8 | 1 | 155.4 | 19.43 | 1.00× | 1.00 |
| V2S2/Ours_rollout/2000 | 8 | 4 | 43.4 | 5.42 | 3.58× | 0.90 |
| V2S2/Ours_rollout/2000 | 8 | 12 | 33.1 | 4.13 | 4.70× | 0.39 |
