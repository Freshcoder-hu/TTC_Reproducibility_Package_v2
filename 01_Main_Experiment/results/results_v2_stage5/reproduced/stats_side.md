<!-- REPRO:stats_side -->
Paired Wilcoxon signed-rank, two-sided.  *seed* = the primary family (n = 30 seeds, scenes averaged within seed, family size **49**); *run* = the descriptive run-level family (n = 300, pseudoreplication).  Δ median < 0 favours Ours when *better* = lower.

| endpoint | baseline | n | Δ median (Ours − base) | p (seed) | p_Holm (seed) | p (run) | p_Holm (run) | better | seed | run | disagree |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | B-Outmost | 30 | -0.036 | 0.0001 | 0.0018 | 0.0000 | 0.0000 | lower | yes | yes |  |
| C | B-PCBF | 30 | 0.006 | 0.0518 | 0.4659 | 0.0068 | 0.0409 | lower | no | yes | **yes** |
| C | B1 | 30 | -0.096 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| C | B2 | 30 | -0.067 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| C | VH-Danger | 30 | -0.119 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| C | VH-Danger-path | 30 | -0.164 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| C | VH-Path | 30 | -0.200 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| clr_so | B-Outmost | 30 | -3.958 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| clr_so | B-PCBF | 30 | -15.751 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| clr_so | B1 | 30 | 1.576 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| clr_so | B2 | 30 | 1.123 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| clr_so | VH-Danger | 30 | 0.132 | 0.9099 | 1.0000 | 0.0000 | 0.0000 | higher | no | yes | **yes** |
| clr_so | VH-Danger-path | 30 | 0.267 | 0.6143 | 1.0000 | 0.0000 | 0.0000 | higher | no | yes | **yes** |
| clr_so | VH-Path | 30 | 1.362 | 0.0978 | 0.7822 | 0.0000 | 0.0000 | higher | no | yes | **yes** |
| ctrl_ms | B-Outmost | 30 | 3.225 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | B-PCBF | 30 | -31.450 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | B1 | 30 | 1.429 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | B2 | 30 | 1.519 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | VH-Danger | 30 | -0.444 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | VH-Danger-path | 30 | -0.560 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| ctrl_ms | VH-Path | 30 | -3.501 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | B-Outmost | 30 | -2.760 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | B-PCBF | 30 | -3.090 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | B1 | 30 | 0.071 | 0.0000 | 0.0005 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | B2 | 30 | 0.046 | 0.0000 | 0.0003 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | VH-Danger | 30 | -0.850 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | VH-Danger-path | 30 | -0.604 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| devW | VH-Path | 30 | -0.990 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| exec_time | B-Outmost | 13 | -1.750 | 0.7532 | 1.0000 | 0.0129 | 0.0647 | lower | no | no |  |
| exec_time | B-PCBF | 0 | — | — | — | — | — | lower | no | no |  |
| exec_time | B1 | 30 | -0.015 | 0.9590 | 1.0000 | 0.0916 | 0.2912 | lower | no | no |  |
| exec_time | B2 | 30 | -0.267 | 0.5857 | 1.0000 | 0.0728 | 0.2912 | lower | no | no |  |
| exec_time | VH-Danger | 30 | -24.470 | 0.0005 | 0.0064 | 0.0000 | 0.0000 | lower | yes | yes |  |
| exec_time | VH-Danger-path | 30 | -12.752 | 0.0000 | 0.0005 | 0.0000 | 0.0000 | lower | yes | yes |  |
| exec_time | VH-Path | 29 | -30.288 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | lower | yes | yes |  |
| rho | B-Outmost | 30 | 0.444 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| rho | B-PCBF | 30 | 0.551 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| rho | B1 | 30 | -0.000 | 0.1306 | 0.9141 | 0.0038 | 0.0263 | higher | no | yes | **yes** |
| rho | B2 | 30 | -0.000 | 0.0082 | 0.0822 | 0.0000 | 0.0003 | higher | no | yes | **yes** |
| rho | VH-Danger | 30 | 0.111 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| rho | VH-Danger-path | 30 | 0.059 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| rho | VH-Path | 30 | 0.102 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| ttc_min | B-Outmost | 30 | -7.079 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| ttc_min | B-PCBF | 30 | -10.684 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| ttc_min | B1 | 30 | 0.802 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| ttc_min | B2 | 30 | 0.690 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | higher | yes | yes |  |
| ttc_min | VH-Danger | 30 | -0.716 | 0.0036 | 0.0433 | 0.1456 | 0.2912 | higher | yes | no | **yes** |
| ttc_min | VH-Danger-path | 30 | -0.796 | 0.0036 | 0.0433 | 0.2576 | 0.2912 | higher | yes | no | **yes** |
| ttc_min | VH-Path | 30 | 0.384 | 0.9263 | 1.0000 | 0.0006 | 0.0050 | higher | no | yes | **yes** |

<!-- REPRO:stats_risk -->
Sensitivity: the same seed-clustered tests with the five risk scenarios equally weighted (uncorrected p, Holm not applied).

| endpoint | baseline | n | Δ median | p (seed, risk) |
|---|---|---|---|---|
| C | B1 | 30 | -0.125 | 0.0000 |
| C | B2 | 30 | -0.081 | 0.0000 |
| C | VH-Path | 30 | -0.267 | 0.0000 |
| C | VH-Danger | 30 | -0.136 | 0.0000 |
| C | VH-Danger-path | 30 | -0.219 | 0.0000 |
| C | B-Outmost | 30 | -0.028 | 0.0007 |
| C | B-PCBF | 30 | 0.006 | 0.1119 |
| clr_so | B1 | 30 | 1.668 | 0.0000 |
| clr_so | B2 | 30 | 1.213 | 0.0000 |
| clr_so | VH-Path | 30 | 1.859 | 0.0148 |
| clr_so | VH-Danger | 30 | 1.425 | 0.0020 |
| clr_so | VH-Danger-path | 30 | 1.711 | 0.0004 |
| clr_so | B-Outmost | 30 | -2.462 | 0.0002 |
| clr_so | B-PCBF | 30 | -12.092 | 0.0000 |
| ttc_min | B1 | 30 | 0.933 | 0.0000 |
| ttc_min | B2 | 30 | 0.730 | 0.0000 |
| ttc_min | VH-Path | 30 | 0.798 | 0.0368 |
| ttc_min | VH-Danger | 30 | 0.585 | 0.2059 |
| ttc_min | VH-Danger-path | 30 | 0.584 | 0.2989 |
| ttc_min | B-Outmost | 30 | -5.476 | 0.0000 |
| ttc_min | B-PCBF | 30 | -10.450 | 0.0000 |
| rho | B1 | 30 | -0.000 | 0.2802 |
| rho | B2 | 30 | -0.000 | 0.1414 |
| rho | VH-Path | 30 | 0.095 | 0.0000 |
| rho | VH-Danger | 30 | 0.096 | 0.0000 |
| rho | VH-Danger-path | 30 | 0.031 | 0.0000 |
| rho | B-Outmost | 30 | 0.449 | 0.0000 |
| rho | B-PCBF | 30 | 0.585 | 0.0000 |
| devW | B1 | 30 | 0.082 | 0.0001 |
| devW | B2 | 30 | 0.061 | 0.0000 |
| devW | VH-Path | 30 | -0.986 | 0.0000 |
| devW | VH-Danger | 30 | -0.867 | 0.0000 |
| devW | VH-Danger-path | 30 | -0.494 | 0.0000 |
| devW | B-Outmost | 30 | -2.766 | 0.0000 |
| devW | B-PCBF | 30 | -3.377 | 0.0000 |
| ctrl_ms | B1 | 30 | 1.644 | 0.0000 |
| ctrl_ms | B2 | 30 | 1.725 | 0.0000 |
| ctrl_ms | VH-Path | 30 | -3.226 | 0.0000 |
| ctrl_ms | VH-Danger | 30 | -0.237 | 0.0000 |
| ctrl_ms | VH-Danger-path | 30 | -0.391 | 0.0000 |
| ctrl_ms | B-Outmost | 30 | 3.272 | 0.0000 |
| ctrl_ms | B-PCBF | 30 | -28.312 | 0.0000 |
| exec_time | B1 | 28 | 0.010 | 0.8376 |
| exec_time | B2 | 28 | -0.355 | 0.3995 |
| exec_time | VH-Path | 26 | -34.312 | 0.0000 |
| exec_time | VH-Danger | 24 | -28.317 | 0.0001 |
| exec_time | VH-Danger-path | 28 | -14.475 | 0.0041 |
| exec_time | B-Outmost | 7 | 10.350 | 0.2367 |
| exec_time | B-PCBF | 0 | — | — |
