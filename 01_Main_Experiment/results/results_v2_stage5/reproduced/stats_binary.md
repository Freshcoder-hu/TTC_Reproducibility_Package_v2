<!-- REPRO:stats_binary -->
| endpoint | baseline | seed n01/n10 | p_Holm (seed, all-10) | run n01/n10 | p_Holm (run, 300 pairs) |
|---|---|---|---|---|---|
| arrival | B1 | 0/0 | 1.0000 | 0/0 | 1.0000 |
| arrival | B2 | 0/0 | 1.0000 | 0/0 | 1.0000 |
| arrival | VH-Path | 0/19 | 0.0000 | 0/39 | 0.0000 |
| arrival | VH-Danger | 0/20 | 0.0000 | 0/25 | 0.0000 |
| arrival | VH-Danger-path | 0/13 | 0.0024 | 0/15 | 0.0003 |
| arrival | B-Outmost | 0/23 | 0.0000 | 0/23 | 0.0000 |
| arrival | B-PCBF | 0/30 | 0.0000 | 0/275 | 0.0000 |
| joint | B1 | 2/2 | 1.0000 | 20/15 | 1.0000 |
| joint | B2 | 0/2 | 1.0000 | 10/29 | 0.0135 |
| joint | VH-Path | 0/2 | 1.0000 | 28/137 | 0.0000 |
| joint | VH-Danger | 0/2 | 1.0000 | 29/140 | 0.0000 |
| joint | VH-Danger-path | 0/2 | 1.0000 | 33/151 | 0.0000 |
| joint | B-Outmost | 0/2 | 1.0000 | 3/215 | 0.0000 |
| joint | B-PCBF | 0/2 | 1.0000 | 0/236 | 0.0000 |

n01 = baseline-only successes, n10 = Ours-only successes (exact McNemar, two-sided binomial on the discordant pairs).
