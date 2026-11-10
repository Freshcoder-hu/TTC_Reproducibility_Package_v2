<!-- REPRO:census -->
| method | runs | goal | time | censored | errors | infeasible events | S | P | C | clr_rs [m] | clr_rr [m] | contacts ro/rs/rr | C1lo bind | C1hi bind |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Ours_rollout | 300 | 300 | 0 | 0 | 0 | 52.8 | 30.1 | 21.9 | 8.88 | -0.48 | 0.31 | 2.4/7.9/0.3 | 0.0 | 0.0 |
| B1 | 300 | 300 | 0 | 0 | 0 | 48.3 | 27.2 | 20.3 | 8.20 | -0.48 | 0.38 | 2.6/7.2/0.3 | 0.0 | 0.0 |
| B2 | 300 | 300 | 0 | 0 | 0 | 62.9 | 35.4 | 26.5 | 10.04 | -0.50 | 0.30 | 3.6/8.5/0.3 | 0.0 | 0.0 |
| VH-Path | 300 | 261 | 39 | 39 | 0 | 12.0 | 6.2 | 5.7 | 1.20 | 1.23 | 0.48 | 1.6/0.0/0.1 | 0.0 | 0.0 |
| VH-Danger | 300 | 275 | 25 | 25 | 0 | 10.5 | 5.3 | 5.0 | 1.15 | 1.41 | 1.02 | 1.5/0.0/0.0 | 0.0 | 0.0 |
| VH-Danger-path | 300 | 285 | 15 | 15 | 0 | 11.8 | 6.0 | 5.7 | 1.16 | 1.40 | 1.12 | 1.7/0.0/0.0 | 0.0 | 0.0 |
| B-Outmost | 300 | 277 | 23 | 23 | 0 | 107.9 | 75.5 | 30.4 | 15.82 | -0.62 | -0.12 | 0.0/16.8/1.0 | 0.0 | 0.0 |
| B-PCBF | 300 | 25 | 275 | 275 | 0 | 104.0 | 57.9 | 41.9 | 12.96 | -0.52 | 0.36 | 0.0/16.0/0.1 | 0.0 | 0.0 |

All counts are **per run**, averaged over the 300 runs of the method; `goal`/`time` are termination classes and every run is in exactly one of them.  `infeasible events` is the number of hard-breach interventions by the execution layer; those events split into **resolved**, **S** (solver-infeasible: no candidate in the finite admissible set was safe and the unavoidability certificate did not apply — *not* a claim of physical unavoidability) and **P** (physically unavoidable under the stated assumptions, a sufficient certificate and therefore a lower bound).  **C** is a post-hoc control failure diagnosed once per contact episode by a myopic oracle, so it is a lower bound too; it is counted on animal-animal, robot-obstacle and robot-animal contacts alike, which is why it is larger than the `C` safety endpoint (animal-obstacle only).  `clr_rs` is the worst robot-animal surface clearance: **negative for every method that actually herds** (Ours, B1, B2, B-Outmost, B-PCBF) and positive only for the VH family, which stays away from the flock and therefore does not do the task — see §5.  `C1lo bind`/`C1hi bind` are counter columns: 0 means the bound never clipped, and for C1-low that is structural (§1.3), not a pass.
