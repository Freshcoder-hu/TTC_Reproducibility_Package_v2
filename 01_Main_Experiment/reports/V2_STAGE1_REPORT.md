# V2 Stage 1 — Physical execution layer

**Status:** implemented, all acceptance checks pass (A–E plus the five reviewer additions F–I).
**Scope:** new files only (`v2/`). The V1 tree (`simulation.py`, `proposed*.py`,
`robots.py`, `sheep.py`, `corrections.py`, `config.py`) is byte-identical to
before and the archived V1 matrix remains reproducible bit-for-bit (test A).

---

## 0. 执行摘要（给作者）

1. **惰性（A）通过，且强于要求**：V2-compat 与归档 V1 单元逐位相同（38 个非计时列全等），
   并且与同进程直接跑 V1 的**整条轨迹**（1055 个控制拍的每个日志字段）逐位相同。
   新增的连续接触检测、可行性层、限幅统计都是只读的，不反向影响动力学。
2. **联合限幅（B）**：C3 / R4 / 横向加速度在 2 个种子、149 364 个"羊·步"与 49 788 个"机器人·步"上
   **零违例**，且三个约束**确实在起作用**（C3 咬合 25.2%、R4 94.4%、a_lat 46.4%）。
3. **`a_ang = 5.0 rad/s²`**：`config.py` 本来就声明 5.0，故沿用并**首次真正施加**（R4 咬合率 94%，说明它此前从未生效）。
4. **两个必须报告的坏消息**（未做任何"调零"）：
   - 作者提议的 `d_rr_safe = 1.2 m` **守不住 0.90 m 硬限**（S3、3 种子：每次运行 36.7 次机器人—机器人接触，
     最小表面间距 −0.46 m）。默认已改为 **2.0 m**（= V1 旧值，硬限之上 1.10 m 余量），
     改后 2 次运行仅 2 次接触、最差 −0.40 m。1.2 m 仍可一键复现（`--set d_rr_safe=1.2`）。
   - 在 V1 控制器不变的前提下，V2 **并没有修好**机器人—障碍物的实体间距（最差 −1.12 m）。但必须按同一判据比较：
     把 **V1 的同一条轨迹**按实体半径重检，其最差间距本来就是 **−1.25 m**、每次运行 9.7 次接触；
     V2 为 −1.12 m、10.0 次接触，即**基本持平/略好**，不是 V2 引入的新问题。
     原因已定位并有量化依据：`a_lat = 2.0` 使 4.5 m/s 时的最小转弯半径达 **10.1 m**，
     而 V1 的 DIVERT 横向让道几何（4 m 前视、3 m 横移）只需 **4.2 m**，即几何上要求 **v ≤ 2.89 m/s**；
     4.5 m/s 的速度帽在避让时根本用不上。这是**控制器未针对新被控对象设计**，不是执行层的 bug（§9.3）。
     机器人—羊（rs）与机器人—机器人（rr）在 V2 下变差（robots 更快、转弯受限），同样如实报告。
5. **不可行三分类（I）**：S=229、P=172、C=33（2 个种子的 S3）。明确写明：
   **S 类 ≠ 物理不可避免**；P 类只是"给定假设下的充分条件"。
6. **V1 口径对照（G）**：1269/1920（66.1%，机器人中心进入障碍圆盘 `dmin < 1.2`）**完全复现**；
   同一条轨迹按实体半径 `r_robot + r_o = 1.65` 重检为 **1356/1920（70.6%）**，
   即 **+87 次运行（+4.5 pp）纯粹由判据变化产生**。1509/2160 属于 `results_v9_noB3` 谱系，本工作区不存在该目录 → **待确认**。
   在**运行级事件**上同样量化了该效应：同一 V1 轨迹加实体半径后，S3 每次运行多出
   **+1.67 次机器人—障碍物接触、+2.0 次机器人—羊接触**，而 V1 的全部旧指标逐位不变。
   这是本阶段最有价值的一张表（§8、§9.1）。
7. **接触检测（D）**：线性扫掠对仿真模型是**精确的**（首次接触时间误差 1.6e-15 s，最近距 0 误差）；
   圆弧模式（8 子段）相对真圆弧误差 1.3e-5 m，优于声明的 2.2e-5 m 分辨率；弦线模型误差 = 矢高 1.4e-3 m。
8. **确定性（E）**：重复运行 0 差异；`jobs=1` 与 `jobs=4` **逐位相同**（每个格子是独立进程、行序固定，归约顺序确定）。
9. **联合投影 vs 分别截断（F）**：同一短跑，联合投影 C3 违例 0，分别截断 **67 194** 次；反例 v=1.5/ω=3 已复现（4.50 vs 限 4.0）。
10. 未验证/无法验证项见 §11（含 1509/2160、真机参数、Stage 2 行为模型）。

---

## 1. Scope, invariants and what is deliberately untouched

| | |
|---|---|
| new code | `v2/` package (17 files, 3 918 lines incl. comments) + `results_v2_stage1/` |
| untouched | `simulation.py`, `proposed*.py`, `robots.py`, `sheep.py`, `corrections.py`, `config.py`, all `results_*` |
| V1 reproducible | yes — `python -m v2.probe_v1_repro` reproduces `S3/Ours_rollout/seed 200` bit-exactly (38/38 non-timing columns) |
| interpreter | CPython 3.13.7, numpy 2.5.2, `_rollout.dll` (MSVC) loaded from the code root |
| float settings | default IEEE-754 doubles, single-threaded BLAS (`OMP/MKL/OPENBLAS/NUMEXPR_NUM_THREADS=1`), no fast-math, no `float32` anywhere |

Untouched-tree evidence (SHA-256, first 16 hex, re-checkable at any time):

```
simulation.py      11D44C0C45B246DF     proposed_rollout.py  1BA4672082395E72
proposed.py        230586E288172FEB     robots.py            F50D549D0CC17E92
proposed_event.py  6C839DA2B602EF37     sheep.py             2DF906756DEBE211
proposed_improved.py F771AEB3A4918635   corrections.py       47EA6A57CDE49C5E
                                        config.py            C50F79FE714ABAC5
```

and every `results_*` directory except the new `results_v2_stage1/` still carries
its original mtime (`results_cea_matrix` 2026-10-21 21:38, `results_gate_checks`
2026-10-22 08:33, …).

Why a separate package rather than edits: the manuscript's V1 numbers must stay
reproducible while V2 is developed, and a reviewer must be able to diff "what
changed" by diffing directories, not by reading a patch.

---

## 2. Parameter record (V1 vs V2)

Generated from `v2/config_v2.py::PARAM_TABLE`, i.e. from the same object that is
hashed into the configuration identifier — the table cannot drift from the code.
**AUTHOR-FIXED** marks the values fixed by the author.

| parameter | V1 | V2 (Stage 1) | note |
|---|---|---|---|
| r_robot [m] | not modelled | 0.45 | AUTHOR-FIXED; disc footprint |
| a_lat [m/s^2] | not modelled | 2.0 | AUTHOR-FIXED; \|v·ω\| cap |
| v_max [m/s] | 2.6 | 4.5 | AUTHOR-FIXED |
| a_lin [m/s^2] | 3.0 | 4.0 | AUTHOR-FIXED |
| omega_max [rad/s] | 3.5 | 3.5 | AUTHOR-FIXED; unchanged |
| a_ang [rad/s^2] | 5.0 declared, not imposed | 5.0 imposed | AUTHOR-FIXED (5.0); `config.py` already declared 5.0 |
| d_rr_contact [m] | 0.0 | 0.90 | derived 2·r_robot; V1 had no body |
| d_ro_contact [m] | 1.20 (= r_o) | 1.65 | derived r_robot + r_o |
| d_rs_contact [m] | 0.35 (= r_s) | 0.80 | derived r_robot + r_s |
| d_ss_contact [m] | 0.70 | 0.70 | derived 2·r_s |
| d_so_contact [m] | 1.55 | 1.55 | derived r_s + r_o |
| d_rr_safe (soft) [m] | 2.00 | **2.00** | margin 1.10 m over the 0.90 hard limit; author proposed 1.2 (=0.30 m); rejected with data, see D2 |
| d_ro_safe (soft) [m] | 1.20 (= r_o) | **2.00** | margin 0.35 m over the 1.65 hard limit; author value kept |
| slot_gap [m] | 0.30 | 0.30 | unchanged; slot pitch follows d_rr_safe |
| r_sheep [m] | 0.35 | 0.35 | unchanged |
| r_obs [m] | 1.2 | 1.2 | unchanged |
| n_sheep : n_robots | 36 : 12 | 36 : 12 | unchanged (36:12) |
| C1 sheep speed [m/s] | 1.0–1.5 imposed | 1.0–1.5 imposed | upper bound preserved by construction |
| C2 sheep turn [rad/s] | 3.0 imposed | 3.0 imposed | unchanged |
| C3 sheep accel [m/s²] | 4.0 **declared, NOT imposed** | 4.0 imposed on the velocity **vector** | V1 jumped 1.0→1.5 m/s in one step = 10 m/s² |
| dt [s] | 0.05 | 0.05 | unchanged; common integration step |
| ctrl_period [ticks] | 2 | 2 | unchanged (0.1 s) |

Additional V2-only numerics (not physics): `contact_mode="chord"`,
`contact_substeps=8`, `a_dec_plan=3.0`, `t_delay=0.1`, `brake_margin=0.1`,
`c_diag_window=12` ticks, `eps_v=1e-6`.

Derived hard distances are *computed* from the radii (`V2Cfg.hard_contacts()`),
so changing `r_robot` cannot leave a stale contact table behind.

---

## 3. Configuration identity

Every run writes `config_id` into its row and into `run_meta.json`.

| run mode | config_id | SHA-256 (first 16) |
|---|---|---|
| V2 Stage 1 (shipped default) | `cb2fc33d88a6` | `cb2fc33d88a6807a` |
| V1-compatibility (test A) | `a7e94a2ee683` | `a7e94a2ee68388f3` |

The hash covers **all** V2 fields, the derived contact distances, the V1
reference values, and the `SimCfg` mirror mapping. Any override (e.g.
`--set d_rr_safe=1.2`) produces a different id, so a result cannot be detached
from the parameters that produced it. The `SimCfg` a run actually uses is
re-derived from `V2Cfg` through `SIM_MIRROR` and checked by
`V2Cfg.assert_consistent()`; a mismatch raises instead of silently running.

---

## 4. What the layer does

Execution order per integration step (`v2/fleet_v2.py::RobotFleetV2.step`):

```
1. command filter        soft spacing target + hard-geometry guard   (robot_safety_v2)
2. feasibility layer     braking supervisor, then prediction + hard check + escape
                         (only on control ticks: step % ctrl_period == 0)      (feasibility_v2)
3. joint-limit projection R1..R4 and |v*omega| <= a_lat, solved jointly    (limits_v2)
4. position update        p <- p + u(theta_new) * v_new * dt
```

Sheep, per step (`v2/animal_m0.py::advance_sheep_m0`):

```
1. desired (dtheta, s)   verbatim V1 social model, with C1/C2 applied as in V1  (animal_m0)
2. (V2) box projection   intersection of speed band, turn rate, vector accel    (limits_v2)
3. position update        p <- p + u(theta_new) * s_new * dt
```

### 4.1 Joint limit enforcement — the (V2) box, not sequential clipping

The manuscript's box is transcribed literally:

```
0 <= s+ <= s_max ,   |wrap(theta+ - theta)| <= omega_s * dt ,   ||v+ - v|| <= a_s * dt
```

The projection parameterises the next velocity as `v+ = s+ · u(theta + dtheta)`
and solves the intersection in closed form: for a candidate `dtheta`, the
reachable speeds are

```
c = s_old cos(dtheta),  D = (a_s dt)^2 - (s_old sin(dtheta))^2
s+ in [c - sqrt(D), c + sqrt(D)]      (non-empty  <=>  |dtheta| <= asin(a_s dt / s_old))
```

If the C1 lower bound is not reachable at the requested turn, the turn is
**shrunk** to the largest value compatible with reaching `s_min`
(law of cosines: `cos dtheta = (s_min^2 + s_old^2 - A^2)/(2 s_min s_old)`).
A sequential clip would instead have produced a spurious C1 violation — measured
0 of them (`bind_C1_low_sheep = 0`).

Near-zero speed is handled explicitly: the reachable set is the disc of radius
`a_s dt` around the origin, the heading state is retained, a requested direction
is reachable, and an unreachable C1 band is **flagged**
(`plant_infeasible`) rather than silently violated.

For the robot, R4 needs the turn rate as a state; `RobotFleetV2` carries
`omega` and applies `|omega+ - omega| <= a_ang dt`. The lateral cap
`|v·omega| <= a_lat` is resolved jointly with R4 by a short fixed-point that
only ever reduces the pair, and the existence of a feasible pair is *proved* by
the invariant `v_prev·|omega_prev| <= a_lat`:

```
omega_min = max(0, |omega_prev| - a_ang dt),   v_lo = max(0, v_prev - a_lin dt)
v_lo <= a_lat/omega_min        (always, under the invariant)
=> (v_lo, omega_min) is admissible => the feasible set is never empty
```

The defensive counter `LAT_UNRESOLVED` proves this in practice: **0** hits in
every run of test B.

### 4.2 Command filter (soft target + hard guard)

`apply_robot_safety_v2` is a parameterised transcription of
`robots.apply_robot_safety` (verified bit-identical with the guard off, 200/200
random trials) plus one new term: once a pair is inside the **hard** band the
closing component of the command is cancelled (not merely penalised). The soft
target keeps V1's predictive projection semantics, so the only change is the
number it targets.

### 4.3 Feasibility layer — flags and semantics

| flag / field | meaning |
|---|---|
| `feas_enabled` | master switch; `False` = layer bypassed entirely (test A, and any V1-compat run) |
| `brake_enabled`, `brake_classes` | stopping-distance supervisor; default only `("rr",)` (see D5) |
| `enforce_hard_separation` | enables the hard-geometry guard in the command filter |
| `enforce_c3`, `enforce_r4`, `enforce_lat` | the three new caps |
| `n_infeasible` | count of control ticks at which a hard breach was **projected** inside the execution interval |
| `n_infeasible_resolved` | …of which the layer found and executed a safe admissible option |
| `n_infeasible_S` | …no safe candidate **and** no unavoidability certificate → *solver-infeasible* |
| `n_infeasible_P` | …unavoidability certificate holds → *physically unavoidable under stated assumptions* |
| `n_contact_C` | contact episodes for which the post-hoc oracle found a safe action that was not taken → *control failure* |
| `n_brake_events` | control ticks where the approach component was reduced by the braking supervisor |

Event record (kept in memory, `record_events=True`): `t, i, cls, partner,
clr_pred, clr_after, action, resolved, P, tau, cert_R, cert_gap, cert_rhard`.

Cause codes are `rr | ro | rs` (no animal–animal infeasibility is claimed: the
layer only commands robots).

### 4.4 Continuous contact detection

Per integration interval and per pair class (`rr, ro, rs, ss, so`) the minimum
**surface** clearance is computed by the linear-swept relative-centre formula,
with the exact entry time from the same quadratic. Because both integrators
translate along a straight chord for the whole `dt`, the sweep is **exact for
the simulated model** (no time discretisation error); this is verified in test D
to 0 and 1.6e-15 s.

For curved motion the layer offers `contact_mode="arc"`: the differential-drive
arc is sub-sampled into `contact_substeps = 8` sub-chords. Stated resolution:

```
eps_arc = R (1 - cos(omega dt / (2 N))) <= 2.2e-5 m
```

measured 1.3e-5 m in test D3. The chord model itself errs by the full sagitta

```
eps_chord = R (1 - cos(omega dt / 2)) = 1.4e-3 m  (v = 1.5, omega = 3, dt = 0.05)
```

with a sign that depends on which side the partner is on — reported, and the
reason `arc` mode exists as a sensitivity switch.

Recorded per run: `min_surface_clearance_*`, `any_contact_*`,
`n_contact_events_*` (contiguous episodes; pairs already overlapping at t=0 are
counted separately as `n_preexisting_contact_*` because `init_flock()` starts
with overlaps — V1 has the same initial condition), `first_contact_time`,
`first_contact_class`, `first_contact_pair`, and per-class first-contact.

"First contact before any avoidance response" is implemented as: the sweep runs
on the **executed** trajectory at every sub-step of the interval, so a contact
cannot be hidden by the discrete state being sampled only at interval ends; and
the layer separately records the *counterfactual* breach it projected from the
pre-response command (the `n_infeasible` stream), so "would have hit" and "did
hit" are both available.

### 4.5 Metrics

`v2/metrics_v2.summarize_v2` calls the frozen `metrics.summarize` and merges the
V2 fields, so every V1 column keeps its exact value. New columns: the five
clearance minima, five any-contact flags, five event counts, five pre-existing
counts, first-contact fields, `n_infeasible{,_S,_P,_resolved,_rr,_ro,_rs}`,
`n_contact_C`, `n_brake_events`, `violation_{C1_low,C1_high,C2,C3,R1,R2,R3,R4,lat}`,
`bind_*` (binding frequency), `opp_{sheep,robot}_steps`, `plant_infeasible`,
`LAT_UNRESOLVED`, `LAT_JOINT_FALLBACK`, `n_guard_rr/ro`, `config_id`.

Violations are measured **from the realised state change**, not from the
projection's own bookkeeping, so a bug in the projection cannot satisfy the
check.

---

## 5. Design decisions (with justification)

**D1 — `a_ang = 5.0 rad/s²`.** The author's instruction was "5.0 unless the
existing config suggests otherwise"; `config.py` already declares `a_ang = 5.0`
(R4) but never imposes it. Keeping 5.0 therefore both follows the instruction
and makes the change minimal: R4 moves from *declared* to *enforced* with the
already-published value. 5.0 rad/s² reaches the 3.5 rad/s rate cap in 0.70 s —
slow enough that R4 binds visibly (94.4 % of robot-steps, test B), fast enough
that heading control is not crippled.

**D2 — soft robot–robot target: 2.0 m, not the proposed 1.2 m.** The proposal is
kept and is one flag away (`--set d_rr_safe=1.2`, different `config_id`), but it
is not the default, for three reasons:

1. **Margin criterion.** A command-level target can only be held if the margin
   exceeds what the plant moves in one control period: `v_max · t_ctrl` =
   4.5 × 0.1 = 0.45 m. The proposed margin is `1.2 − 0.90 = 0.30 m < 0.45 m`,
   so the target is unholdable by construction at the speed cap.
   (The 1.2 m variant has `config_id = 29640def0ddb`; the shipped one is
   `cb2fc33d88a6`, so the two can never be confused.)
2. **Measurement** (S3, seeds 200–202, full Ours, `v2/diag_v2.py`):
   | soft `d_rr_safe` | margin | rr contact episodes / run | worst rr clearance | hard 0.90 held? |
   |---|---|---|---|---|
   | 1.2 m (author) | 0.30 m | 36.7 | −0.457 m | **no** |
   | 1.5 m | 0.60 m | 8 (1 seed only) | −0.362 m | no |
   | 2.0 m (shipped) | 1.10 m | 0.7 | +0.024 m | yes (mean) |
3. **Comparability.** 2.0 m is exactly V1's soft target, so the V1 → V2 delta
   isolates the physical layer (body radius, caps, detector) instead of also
   changing the formation pitch, which follows `d_rr_safe` (`slot_positions`
   enforces `d_rr_safe + slot_gap`: 2.3 m at 2.0, 1.5 m at 1.2). A formation
   change would confound the comparison the manuscript wants to make.

`d_ro_safe` stays at the author's 2.0 m. Note that its 0.35 m margin **also**
fails criterion 1 at the speed cap, and the stopping distance at 4.5 m/s is
2.53 m (a_lin) / 3.38 m (de-rated) — i.e. **no static target can guarantee
high-speed obstacle avoidance**; that is what the braking supervisor and the
prediction/escape layer are for, and why the report states the residual
robot–obstacle contacts instead of claiming they are impossible.

**D3 — joint projection instead of sequential clipping.** Required by the
manuscript ("must respect the intersection in (V2)") and demonstrated in test F:
separate clipping violates the vector bound 67 194 times on the same two runs
where the joint projection violates it 0 times. The counterexample from the
review (v = 1.5, ω = 3 ⇒ |vω| = 4.5 > 4.0) is reproduced: separate clipping
gives 4.50 m/s² in one step, the joint projection 4.00.

**D4 — turn rate as robot state.** R4 cannot be imposed without it (V1 recomputes
a turn per step with no memory). `RobotFleetV2.omega` is the only new state.

**D5 — the braking supervisor acts on robot–robot only.**
`d_req(v) = v·t_delay + v²/(2·a_dec) + d_body + margin`, with `a_dec = 3.0` (75 %
of `a_lin`, so the plan never relies on the limit being reached) and
`t_delay = 0.1 s` = one control period (the command computed now takes effect
for the next interval; this is the "control delay" the review asked to state).
It limits the **approach component** only, leaving the tangential component free
— otherwise it would forbid the divert manoeuvre, which is a *lateral* escape.

It is applied to `rr` only, because a stopping-distance rule presumes both
parties *can* stop. Against a non-cooperative crossing obstacle, stopping in its
path is the worst action, and the diagnostic confirms it: enabling the rule for
`ro` made the robot–obstacle surface clearance **worse** (−1.49 m vs −0.94 m on
S3/seed 200) because the robot stopped instead of leaving the swept corridor.
Robot–animal separation is not supervised either: herding *requires* deliberate
approach (animals flee at `r_avo = 4 m`), so a stopping rule against animals
would forbid the task.

**D6 — hard constraint is enforced as far as the dynamics allow, and counted
otherwise.** No positional teleport and no post-hoc projection of positions is
used; when no admissible action avoids a projected breach the event is logged
with a cause code. The manuscript's own P4 says the same ("record infeasible or
unavoidable encounters as failures rather than interpreting requested constraints
as guarantees").

**D7 — the unavoidability certificate `P` is conditional and sufficient only.**
`|p − q(τ)| + R_max(τ) < r_hard`, with `R_max(τ) = min(v_max τ, v τ + ½ a_lin τ²)`
over-approximates the reachable set, so a `P` verdict is a **lower bound** on
true unavoidability. Assumptions: the partner follows its predicted motion
(constant velocity for obstacles/animals, projected for robots) and the robot's
admissible set is the one the projection enforces. The certificate is evaluated
at the tick where the breach is projected, so it certifies "unavoidable from this
state", not "unavoidable had an earlier decision been different" — the latter is
what the post-hoc `C` oracle tests.

**D8 — `S` is not `P`.** `S` means the finite candidate set failed; it is
explicitly *not* a claim of physical unavoidability. They are counted separately
everywhere (`n_infeasible_S` vs `n_infeasible_P`) and the report never quotes the
sum as "unavoidable".

**D9 — `C` is a lower bound.** The post-hoc oracle is myopic (greedy over 13
candidate commands with a 2-step prediction, scanning back up to 12 control
ticks), so it can miss a feasible sequence that a longer-horizon planner would
find. `C` therefore under-counts control failures.

**D10 — pre-existing overlaps are not contact events.** `init_flock()` places 36
animals in a 2.8 m disc, so pairs are already inside 0.70 m at t = 0. Those are
counted in `n_preexisting_contact_ss` and excluded from
`n_contact_events_ss`, otherwise the event count would be dominated by the
initial condition (67 pairs) rather than by dynamics. This is V1 behaviour
inherited unchanged, not a V2 regression.

**D11 — the archive's `dmin_robot_obs` is a lagged statistic.** V1 measures the
robot at `t+dt` against the obstacle at `t` (the distance loop runs before
`o.update(t+dt)`), which compares positions that never coexisted. The swept
detector uses synchronised geometry and reports *larger* clearances (G4:
−0.913 m vs −1.356 m on S3). Both are reported; the archive column cannot be
silently reinterpreted.

**D12 — `dt` and the control period are unchanged.** The (V2) box requires a
"common integration step"; changing it would make the V1 comparison meaningless
and is anyway impossible without touching the frozen tree.

---

## 6. Acceptance tests A–E

Transcripts are copies of `results_v2_stage1/ACCEPTANCE.txt` (regenerate with
`python -m v2.acceptance_v2`). Every check below passes.

### A. Inertness (critical) — PASS

```
A. INERTNESS  (r_robot->0, caps off, feasibility bypassed)
A1  cell S3/Ours_rollout/seed 200: 38 non-timing columns
  [PASS] A1 V2-compat == archived V1 cell (bit-exact, non-timing)  -- 0 mismatch(es)
A2  V1 run_scene vs V2-compat: 1055 logged ticks, 0 difference(s)
  [PASS] A2 V2-compat == V1 run_scene (whole trajectory, bit-exact)  -- 0 difference(s)
```

Conditions pinned (as the review required):

| requirement | how it is met |
|---|---|
| new safety layer fully bypassed | `compat_v1cfg()`: `enforce_c3/r4/lat/hard_separation`, `feas_enabled` all `False`; `hard_guard` off; `RobotFleetV2` never constructs a `FeasibilityLayer` |
| RNG call order unchanged | V2 draws **no** random numbers; the only RNG is `run_scene_v2`'s `np.random.default_rng(seed)` in `init_flock`, at the same point as V1 |
| integration order unchanged | step = `[view.refresh] → controller (every 2 ticks) → robots.step → advance_sheep → V1 stats → obstacle.update → [view.refresh] → collision check → log` — identical, and the added detector sits between the obstacle update and the collision check as a read-only observer |
| control timing unchanged | `ctrl_period = 2`, same tick gate |
| old metrics unchanged | `metrics.summarize` is called unmodified; the V1 numbers are computed by the same statements |
| compiler / float settings | same interpreter, same `_rollout.dll`, double precision, single-threaded BLAS |
| detector must not feed back | `ContactMonitor` reads copies (`pos.copy()`) and writes only to its own state; A2 proves it over 1055 ticks |

A1 compares against the **archive**; A2 compares against a **fresh in-process V1
run**, field by field through the whole trajectory (`t, rho, lat, ttc,
ttc_each[], dmin, cflock, cov_direct, cov_shared, lost, tag, d_goal, d_range,
head_err, nguyen_ok`) plus 15 result scalars. Not one bit differs.

### B. Joint limits — PASS (zero violations, and demonstrably binding)

```
B  seed 200: ticks=72072 wall=17.1s  viol C3/R4/lat = 0/0/0
B  seed 201: ticks=77292 wall=17.7s  viol C3/R4/lat = 0/0/0
B  totals over 2 seeds: violation_C1_low=0, violation_C2=0, violation_C3=0,
   violation_R1=0, violation_R2=0, violation_R3=0, violation_R4=0, violation_lat=0
  [PASS] B1 zero C3 / R4 / lateral violations
  [PASS] B2 the caps are actually binding (test is not vacuous)
```

**Binding-frequency table** (2 runs, S3, full Ours; opportunities = agent-steps):

| bound | clips | opportunities | rate |
|---|---|---|---|
| C2 sheep turn rate | 104 848 | 149 364 | 70.20 % |
| **C3 sheep vector accel** | 37 612 | 149 364 | 25.18 % |
| … of which the turn was reduced | 13 787 | 149 364 | 9.23 % |
| … of which the speed was reduced | 37 612 | 149 364 | 25.18 % |
| C1 sheep lower bound (relaxed) | 0 | 149 364 | 0 % |
| R1 robot speed | 0 | 49 788 | 0 % |
| R2 robot turn rate | 40 957 | 49 788 | 82.26 % |
| R3 robot linear accel | 24 384 | 49 788 | 48.98 % |
| **R4 robot angular accel** | 47 004 | 49 788 | 94.41 % |
| **LAT \|v·ω\|** | 23 092 | 49 788 | 46.38 % |

The test is not vacuous: every new cap clips on a large fraction of steps. R4
binds on 94 % of robot-steps and the lateral cap on 46 %, which is itself a
finding: **the two limits V1 declared but never imposed are the two that are
almost always active.** C1-low never needed relaxing (0 clips, 0 violations),
confirming that the law-of-cosines turn shrink preserves the speed band.

### C. Non-penetration — reported (never tuned)

```
class             hard d    min clr            ~seed  events  preexist   any
robot-robot         0.90    -0.4014   [0.26, -0.401]       2         0  True
robot-obstacle      1.65    -1.3016 [-1.302, -1.091]      21         0  True
robot-sheep         0.80    -0.6368  [-0.637, -0.47]      23         0  True
sheep-sheep         0.70    -0.6986 [-0.682, -0.699]     375        67  True
sheep-obstacle      1.55    -1.2305 [-0.611, -1.231]       7         0  True
```

Interpretation (per class):

> the harness prints "see report §6.4/§9"; the intended pointers are this
> section (test C) and §9.2–§9.4.

* **robot–robot** — essentially held: 2 episodes over 2 runs, worst −0.40 m in
  one seed. Not a guarantee; at `d_rr_safe = 1.2` it is 69 episodes and
  −0.457 m (D2). With only two episodes no per-class cause claim is made here;
  the aggregate S/P/C census is in §9.2.
* **robot–obstacle** — 21 episodes, worst −1.30 m (the robot centre is inside the
  obstacle disc). This is the largest residual failure and it is a **control**
  problem, not a plant problem: see §9.3 for the turn-radius proof.
* **robot–sheep** — 23 episodes, worst −0.64 m. Robots are commanded *through*
  the flock by the slot geometry; the missing `rs` term in the soft filter (V1
  had none either) is the cause. Fixing it belongs to the controller.
* **sheep–sheep** — dominated by the 67 initial-condition overlaps (D10); after
  those resolve, 375 episodes remain, worst −0.70 m, i.e. animals squeezing past
  each other at 1.0–1.5 m/s with a 0.70 m body. V1 has the same behaviour (its
  social model has no hard body constraint), so this is a pre-existing model
  property that V2 now *measures*.
* **sheep–obstacle** — 7 episodes, worst −1.23 m; this is the V1 `collision_rate`
  mechanism (animals have no autonomous obstacle avoidance by design, 方案 A).

Which of these are unavoidable vs controller failure is answered in §9.2 with
the S/P/C census; the short version is that **none of them is claimed
unavoidable by default**, and `S` is explicitly not `P`.

### D. Contact-detection accuracy — PASS

```
D1a closest approach of a swept segment to a point:
    d=0.40000000000000002 (exact 0.4), tau=0.8333333333 (exact 5/6)
D1b moving pair, exact entry tau = 0.50029201607433349 vs analytic
    0.50029201607433349  (|diff| = 0.00e+00, start clearance = +0.0696 m)
D2  monitor, 2 robots closing at 3 m/s, lateral offset 0.5 m, r_rr = 0.90:
    reported min surface clearance = -0.40000000000000002  (analytic -0.4)
    reported first contact time    = 1.4172228408817358  (analytic 1.4172228408817373)
    |error| = 1.55e-15 s   events=1
D3  one integration step, v=1.5, omega=3.0, dt=0.05, N_sub=8:
    exact sagitta R(1-cos(omega dt/2))        = 1.406e-03 m
    sub-chord resolution R(1-cos(omega dt/2N)) = 2.197e-05 m
    probe           dense arc    chord err   arc-N8 err
    +y         0.019941437217   -1.403e-03   -1.259e-05
    -y         0.019945936040   +1.403e-03   +1.101e-05
```

* Closest approach and entry time are exact to machine precision (the swept
  formula is analytic; the model is piecewise linear).
* The chord model's error equals the sagitta with a sign that depends on the
  side of the partner (D11-style honesty: the chord is not conservative in
  general).
* Arc mode with 8 sub-chords is within 1.3e-5 m of a 4001-point reference, i.e.
  better than the stated 2.2e-5 m resolution.
* Test D2 also verifies the **monitor-level** accumulation across 60 intervals,
  not just the primitive.

### E. Determinism — PASS (bit-exact, including parallel)

```
E1  repeat run: 0 differing columns
  [PASS] E1 two identical runs give identical results  -- 0 diff
E2  jobs=1 vs jobs=4: 2 rows, 0 differing cells
  [PASS] E2 jobs=1 == jobs=4 (bit-exact, deterministic reduction order)
```

Bit-exactness across `--jobs` is *achieved*, not approximated: each CSV cell is
one independent run in its own process (no shared accumulators), and
`ProcessPoolExecutor.map` preserves task order, so the row order and every value
are identical. There is no cross-process reduction to reorder. (If a future
stage adds a cross-run reduction, this claim must be re-checked.)

---

## 7. Reviewer additions F–I

### F. Separate clipping vs joint projection — PASS

```
F1  counterexample: v=1.5 m/s, omega=3.0 rad/s -> |v*omega| = 4.50 m/s^2
    separate clipping : turn kept 0.15 rad -> |dv| = 0.2248 m/s over one step
                        = 4.50 m/s^2 (LIMIT 4.0)
    joint projection  : turn 0.133732 rad, speed 1.486607 -> |dv| = 0.2000
                        -> 4.00 m/s^2

F2  same short runs, projection=joint vs projection=sequential
    arm           seed   viol_C3   bind_C3  collision_rate  rho_avg
    joint          200         0     18888          0.0278   0.9872
    joint          201         0     18724          0.1667   0.9772
    sequential     200     33752     16876          0.1389   0.9747
```

Total C3 violations: **joint 0, sequential 67 194** (33 752 + 33 442). The
sequential arm is a counterfactual only (`projection="sequential"`), never a
default; it is what "clip speed and turn separately" would give, and it is
exactly the V1 defect the manuscript describes (10 m/s² implied by a 1.0→1.5 m/s
step).

### G. V1 criterion / V1-trajectory solid re-check / V2 solid contact — PASS

```
G1  archive: 2400 runs = 8 methods x 10 scenarios x 30 seeds;
    risk subset = 1920 runs (8 x 8 x 30)
    V1 criterion   dmin_robot_obs < r_o        = 1.20 m : 1269/1920 = 66.1%
    same trajectories, solid re-check < r_robot + r_o = 1.65 m :
                                                        1356/1920 = 70.6%
    contacts added purely by the criterion change : 87 runs (+4.5 pp)
  [PASS] G1 1269/1920 reproduced exactly
```

So the two archived numbers reconcile as: **the 66.1 % figure is exactly
`#{runs: dmin_robot_obs < 1.2}` over the 8-method × 8-risk-scenario × 30-seed
matrix**, and its criterion is *robot centre inside the obstacle disc*
(`r_o = 1.2 m`), not body contact. The 9-method figure 1509/2160 belongs to the
`results_v9_noB3` lineage; that directory does not exist in this workspace, and
`paper_cea_v4/V4_REVISION_REPORT.md` itself attributes it to the previous
lineage and does not use it → **待确认** (see §11).

**Per scenario** (V1 runs, n = 240 each) and **per method** (n = 240 each):

| scenario | `<1.20` | `<1.65` | added | | method | `<1.20` | `<1.65` | added |
|---|---|---|---|---|---|---|---|---|
| S2 | 158 | 170 | 12 | | B-Outmost | 0 | 0 | 0 |
| S2R3 | 179 | 183 | 4 | | B-PCBF | 64 | 70 | 6 |
| S2R2 | 158 | 172 | 14 | | B1 | 230 | 231 | 1 |
| S2R1 | 129 | 151 | 22 | | B2 | 228 | 235 | 7 |
| S3 | 174 | 178 | 4 | | Ours_rollout | 198 | 207 | 9 |
| S4S | 134 | 152 | 18 | | VH-Danger | 141 | 171 | 30 |
| S4M | 165 | 172 | 7 | | VH-Danger-path | 201 | 223 | 22 |
| S4L | 172 | 178 | 6 | | VH-Path | 207 | 219 | 12 |

The criterion artefact is **not uniform**: it is 30 runs (+12.5 pp) for
VH-Danger and 1 run for B1, because it depends on how often robots graze
obstacles at 1.2–1.65 m. Any claim of the form "V2 improved physical safety"
must therefore use the solid criterion on *both* sides, which is why the
three-way table is mandatory.

**Continuous re-check of the V1 trajectory** (8 risk scenarios, seed 200,
Ours_rollout, replaying the archived cell through the V2 detector with
`r_robot = 0.45, r_o = 1.2`, i.e. the V1 trajectory under the V2 criterion):

| scenario | archive `dmin_robot_obs` | swept min surface clr (`ro`) | per-step min clr | `ro` events |
|---|---|---|---|---|
| S2 | 0.6529 | −0.5410 | −0.9971 | 2 |
| S2R3 | 0.1478 | −1.1036 | −1.5022 | 3 |
| S2R2 | 0.9262 | −0.2382 | −0.7238 | 2 |
| S2R1 | 2.5091 | +1.3875 | +0.8591 | 0 |
| S3 | 0.2942 | −0.9134 | −1.3558 | 8 |
| S4S | 0.3109 | −0.8992 | −1.3391 | 2 |
| S4M | 0.1482 | −1.0828 | −1.5018 | 3 |
| S4L | 0.0553 | −1.1684 | −1.5947 | 3 |

Two conclusions: (i) at the *run* level the lagged and the swept criteria agree
on this subset (the same 7 of 8 scenarios are contacts); (ii) the *depth* differs
systematically — the archive's lagged statistic is 0.3–0.45 m more pessimistic,
because it compares the robot at `t+dt` with the obstacle at `t` (D11). The V2
column is the physically meaningful one.

**V2 solid contact** is the third column of the comparison matrix in §9.4 (same
scenarios/seeds, new layer): e.g. S3 `ro` surface clearance −1.39 (V2, default)
vs −0.80 (V1). So V2 does **not** improve robot–obstacle clearance — reported
plainly.

### H. Nominal spacing vs braking distance — PASS

```
d_ro_safe (nominal, author) = 2.00 m; hard body limit r_robot + r_o = 1.65 m
static surface margin = 0.35 m;  t_delay = 0.10 s

    v  d_stop a=3.0  d_stop a=4.0  d_delay  d_req(3.0)  d_req(4.0)  vs 2.0 m
  1.0         0.167         0.125    0.100       2.017       1.975    +0.017
  2.0         0.667         0.500    0.200       2.617       2.450    +0.617
  2.6         1.127         0.845    0.260       3.137       2.855    +1.137
  3.5         2.042         1.531    0.350       4.142       3.631    +2.142
  4.5         3.375         2.531    0.450       5.575       4.731    +3.575
```

`d_req = d_stop + v·t_delay + d_body + margin`. At the 4.5 m/s cap the stopping
distance alone (2.53 m at `a_lin = 4.0`; 3.38 m de-rated to 3.0) exceeds the
entire 2.0 m nominal target, so **the nominal target is not a high-speed
guarantee** — it is a spacing preference. Hence the layered response: soft target
(routine), braking supervisor (approach-rate limiting, `rr`), prediction +
escape (hard breaches), and honest accounting of what still fails.

`t_delay = 0.1 s` is one control period, i.e. the command computed at a tick can
only influence the *next* interval. Computation time is not added on top because
the controller runs inside the same tick; if a real stack adds actuation or
communication delay, `V2Cfg.t_delay` must be increased and the table recomputed.

### I. Infeasibility cause census — PASS

```
     seed   infeasible  infeasible_S  infeasible_P  resolved  contact_C  brake
      200          178           104            74         0         12   4595
      201          223           125            98         0         21   5253
```

Totals over the two runs: **S = 229, P = 172, C = 33**; `resolved = 0`.
Read as: ~25 % of control ticks project a hard breach; of those, roughly half
carry an unavoidability certificate *from the state at which they are evaluated*
(P) and half do not (S) — and **S is not a claim of physical unavoidability**.
`resolved = 0` is itself informative: with 0.1 s of lookahead and 0.4 m/s of
per-tick velocity authority, by the time a breach is *projected* there is
nothing left to choose, so the layer's real lever is the soft target and the
braking supervisor, not the last-moment escape. `C = 33` contact episodes had a
safe action available 1.2 s earlier that was not taken (myopic oracle, so a
lower bound).

---

## 8. Where the numbers land: V1 vs V2 (paired, 3 seeds)

`python -m v2.compare_v1_v2 --jobs 4` → `results_v2_stage1/compare.csv`.
Ours_rollout, seeds 200–202; all values are means over the 3 seeds (clearance
columns are therefore means of per-run minima, not global minima).

**`V1solid` is the reviewer's "V1 trajectory, solid re-check" arm**: the V1 plant
verbatim (`compat=True`, feasibility off, guard off) with `r_robot = 0.45` fed to
the *detector only*. Its V1 metrics are bit-identical to the `V1` row — the
radius provably does not touch the dynamics — while its clearance columns use the
V2 body criterion. This makes the three columns of the requested table
comparable for the first time.

| scen | arm | collision_rate | min_dist | dmin_robot_obs | dmin_robot_robot | rho_avg | max_dev |
|---|---|---|---|---|---|---|---|
| S2 | V1 | 0.009 | 1.675 | 0.469 | 1.677 | 0.980 | 3.12 |
| S2 | **V1solid** | 0.009 | 1.675 | 0.469 | 1.677 | 0.980 | 3.12 |
| S2 | V2 (2.0) | 0.019 | 1.268 | 0.450 | 1.117 | 0.993 | 1.84 |
| S2 | V2 (1.2) | 0.000 | 1.928 | 0.542 | 0.623 | 0.966 | 4.54 |
| S3 | V1 | 0.102 | 0.791 | 0.412 | 1.482 | 0.977 | 3.44 |
| S3 | **V1solid** | 0.102 | 0.791 | 0.412 | 1.482 | 0.977 | 3.44 |
| S3 | V2 (2.0) | 0.083 | 0.778 | 0.542 | 0.923 | 0.985 | 2.85 |
| S3 | V2 (1.2) | 0.204 | 0.815 | 0.273 | 0.482 | 0.970 | 4.62 |

| scen | arm | rr clr | ro clr | rs clr | so clr | n_ev rr | n_ev ro | n_ev rs | n_ev so |
|---|---|---|---|---|---|---|---|---|---|
| S2 | V1 | +1.677 | −0.734 | −0.051 | +0.124 | 0 | 3.00 | 0.67 | 0.33 |
| S2 | **V1solid** | **+0.777** | **−1.184** | **−0.501** | +0.124 | 0 | 3.33 | 3.33 | 0.33 |
| S2 | V2 (2.0) | +0.217 | −1.224 | −0.604 | −0.286 | 0 | 3.33 | 5.67 | 0.67 |
| S2 | V2 (1.2) | −0.278 | −1.106 | −0.554 | +0.387 | 12.0 | 4.00 | 4.33 | 0.00 |
| S3 | V1 | +1.482 | −0.799 | −0.051 | −0.766 | 0 | 8.00 | 0.67 | 3.67 |
| S3 | **V1solid** | **+0.582** | **−1.249** | **−0.501** | −0.766 | 0 | 9.67 | 2.67 | 3.67 |
| S3 | V2 (2.0) | +0.023 | −1.115 | −0.614 | −0.775 | 0.67 | 10.0 | 10.7 | 3.00 |
| S3 | V2 (1.2) | −0.418 | −1.388 | −0.554 | −0.744 | 36.7 | 6.67 | 4.33 | 7.33 |

| scen | arm | n_infeas | S | P | C | n_brake |
|---|---|---|---|---|---|---|
| S2 | V2 (2.0) | 60 | 34.3 | 25.0 | 8.7 | 1854 |
| S2 | V2 (1.2) | 170 | 96.7 | 68.3 | 12.0 | 3935 |
| S3 | V2 (2.0) | 177 | 101 | 75 | 15.7 | 4758 |
| S3 | V2 (1.2) | 528 | 303 | 213 | 23.3 | 7222 |

(`V1` / `V1solid` have no feasibility layer, hence no counts.)

**The criterion effect, measured exactly.** The `V1 → V1solid` clearance shifts
are precisely the radius increments — rr `1.482 − 0.90 = 0.582`, ro
`−0.799 − 0.45 = −1.249`, rs `−0.051 − 0.45 = −0.501` — which doubles as an
end-to-end consistency check of the detector. At the *event* level the same
trajectories gain **+1.67 robot–obstacle and +2.0 robot–animal episodes per run
(S3)** from nothing but the body radius.

**V2 vs V1 under the same criterion** (the only valid comparison):

* robot–obstacle: 10.0 vs 9.67 episodes and −1.115 vs −1.249 m → **neutral to
  slightly better**; V2 does not fix it either (see §9.3);
* robot–robot: +0.023 vs +0.582 m, 0.67 vs 0 episodes → **worse** (faster
  robots, curvature-limited);
* robot–animal: 10.7 vs 2.67 episodes, −0.614 vs −0.501 m → **worse**;
* animal–obstacle (`collision_rate`): 0.083 vs 0.102 → slightly better;
* path keeping: `rho_avg` 0.985 vs 0.977 and `max_dev` 2.85 vs 3.44 → better
  (slower, curvature-limited robots wander less).

So the honest summary is **"V2 is not a safety improvement; it is a physically
admissible model"**: with the V1 controller unchanged it is roughly neutral on
obstacle avoidance, better on path keeping, and worse wherever robots now move
quickly near each other or near animals. The 1.2 m arm is dominated on every
axis — the D2 decision.

---

## 9. The three-way accounting and the infeasibility census

### 9.1 Criterion reconciliation (reviewer item 1)

| quantity | value | how verified |
|---|---|---|
| 8-method matrix, total runs | 2400 = 8 × 10 × 30 | `results_cea_matrix/_parts/*/detail.csv` |
| … risk subset | 1920 = 8 × 8 × 30 (S2, S2R3, S2R2, S2R1, S3, S4S, S4M, S4L) | scenarios with obstacles |
| **V1 criterion** `dmin_robot_obs < 1.2` | **1269 / 1920 = 66.1 %** | recomputed from the archive → **exact match** |
| **V1 trajectory, solid re-check** `< 1.65` | **1356 / 1920 = 70.6 %** | same rows, same column, new radius |
| **added purely by the criterion** | **+87 runs (+4.5 pp)** | difference |
| 9-method `results_v9_noB3` | 1509 / 2160 | **not present in this workspace → 待确认** |

Both archived denominators are consistent with the stated designs
(8×8×30 = 1920 and 9×8×30 = 2160), so the *difference between the two published
percentages is a method-count difference, not a criterion change* — but I can
only verify the 8-method number directly. The 9-method number is left 待确认.

The important result is the second row: **on identical V1 trajectories, adding
the 0.45 m body converts 87 "safe" runs into contacts.** Any V1→V2 safety
comparison that does not apply the same solid criterion to both sides would
report a ~4.5 pp artefact as an improvement.

### 9.2 Cause classification (reviewer item 2)

| code | definition | count (2 runs) | can be quoted as "unavoidable"? |
|---|---|---|---|
| **S** | no admissible candidate in the evaluated set is safe, and the certificate does not apply | 229 | **no** — it is a statement about the search, not about physics |
| **P** | certificate `|p−q(τ)| + R_max(τ) < r_hard` holds | 172 | yes, **conditional** on the stated assumptions |
| **C** | a safe admissible action existed (post-hoc oracle) but was not taken | 33 | no — it is a control failure |

The report never quotes `S + P` as "unavoidable", and §6 of the acceptance log
states the distinction explicitly.

### 9.3 Why robot–obstacle contacts persist: a control-layer limit, quantified

Lateral cap ⇒ minimum turn radius `R_min(v) = max(v/ω_max, v²/a_lat)`:

| v [m/s] | ω_max(V1, no cap) | R_min, V1 | R_min, V2 (a_lat = 2) |
|---|---|---|---|
| 1.5 | 3.5 | 0.43 m | 1.13 m |
| 2.6 | 3.5 | 0.74 m | 3.38 m |
| 3.87 | 3.5 | 1.11 m | 7.49 m |
| 4.5 | 3.5 | 1.29 m | **10.13 m** |

The V1 DIVERT manoeuvre (`robots.divert_target`, `lookahead=4.0`,
`max_lateral_ratio=0.55` ⇒ shift ≤ 3.3 m on a 6 m half-width path) steers the
flock toward a point `L = 4 m` ahead and `h = 3 m` to the side. The circular arc
through that displacement needs

```
R_need = (L^2 + h^2) / (2h) = (16 + 9) / 6 = 4.17 m      (L = 4, h = 3)
        = 7.50 m                                          (L = 6, h = 3.3, the wide case)
```

Therefore:

* at the 4.5 m/s cap the available radius is 10.1 m, so the manoeuvre is
  **geometrically impossible** — the robot cannot bend the path enough and
  overshoots the target;
* the largest speed at which it fits is `v = sqrt(a_lat · R_need)`:
  **2.89 m/s** for the actual 4 m lookahead, 3.87 m/s for the wide case;
* V1 could do it at 2.6 m/s with a 0.74 m radius (no lateral cap at all).

This is why the residual robot–obstacle contacts are **controller-side**: the V1
law was tuned for a plant that can turn on the spot, and the 4.5 m/s cap is
unusable during a divert. Stage 2 (or a controller pass) must either reduce the
commanded speed in DIVERT (the plant can report the admissible curvature:
`v ≤ sqrt(a_lat · R_need)`) or re-plan the divert geometry. It must **not** be
"fixed" by relaxing `a_lat`, which is an author-fixed physical value.

### 9.4 What the hard limit costs

All clearances below use the V2 body criterion (`r_robot = 0.45`), including the
V1 row (`V1solid`), so the columns are comparable. 3 seeds, S3.

| configuration | rr episodes / run | worst rr clr | ro worst clr | animal collision rate |
|---|---|---|---|---|
| V1 trajectory, solid re-check | 0 | +0.582 | −1.249 | 0.102 |
| `d_rr_safe` 1.2 | 36.7 | −0.457 | −1.388 | 0.204 |
| `d_rr_safe` 1.5 | 8 (1 seed) | −0.362 | — | — |
| `d_rr_safe` 2.0 (shipped) | 0.7 | +0.023 | −1.115 | 0.083 |

(3 seeds, S3; the 1.5 m row is from the single-seed diagnostic and is marked as
such in §5-D2.)

---

## 10. What Stage 2 (M0 / M1 / M2) must hook into

1. **Animal model entry point.** `v2/animal_m0.py::desired_sheep_update(st, cfg,
   robots_pos, path)` returns `(dtheta, s_des, dtheta_raw, flee)` — the desired
   *increment*, not an integration. M1 (finite obstacle avoidance) and M2
   (heterogeneous gains, reaction delay, stimulus-dependent speed target)
   replace **this function only**; the (V2) box projection below it, the
   execution loop, the detector and the metrics do not change. That is the whole
   point of separating "what the animal wants" from "what the body can do".
2. **Admissible lower speed = 0.** The (V2) box in the manuscript allows
   `s_min = 0` for M1/M2. It is already a field (`V2Cfg.s_min`, mirrored to
   `SimCfg.v_base`); Stage 2 sets it to 0. Note `advance_sheep_m0` handles
   `s_old ≈ 0` explicitly (disc of radius `a_s dt`, heading retained), so
   stopping and restarting is already supported and tested (unit test
   "near-zero speed").
3. **Per-animal parameters.** M2 needs gains/delay per individual. The projection
   takes scalars per animal today (`project_sheep_increment` is called per
   animal in a loop), so vectorising `a_sheep` / `omega_sheep` is a signature
   change there, not a rewrite.
4. **Animal sensing sweep (3/6/9 m).** `desired_sheep_update` receives the *full*
   robot array, exactly as V1 does (`cfg.r_avo` gates the influence). M1/M2 must
   apply their own finite obstacle range inside the desired-velocity function;
   the execution layer must not know about sensing, or the "same physical plant,
   different behaviour model" comparison breaks.
5. **Metrics already in place.** `min_surface_clearance_*` and
   `n_contact_events_*` are computed identically for any animal model, so the
   M0/M1/M2 contrast needs no new instrumentation. `violation_C3` etc. are
   computed from realised state changes, so a misbehaving M1/M2 shows up
   immediately.
6. **Feasibility layer is model-agnostic.** Its predictions treat animals as
   constant-velocity; that is an approximation which becomes weaker as M1/M2 add
   avoidance. If Stage 2 wants better predictions it should supply an animal
   rollout, not change the classifier.
7. **Configuration.** Extend `V2Cfg` (new frozen fields ⇒ new `config_id`), not
   `SimCfg`, for anything model-specific. `SIM_MIRROR` only carries values the
   V1 helpers genuinely need.
8. **Controller re-tuning is *not* Stage 2.** §9.3 shows the divert geometry
   needs `v ≤ 2.89 m/s` at `a_lat = 2.0` (4 m lookahead, 3 m shift). Either the
   controller learns the admissible curvature (it can: `V2Cfg` is available to
   it) or a later stage re-plans the manoeuvre. Do not "fix" it by relaxing
   `a_lat`.

---

## 11. Limitations and 待确认

| item | status |
|---|---|
| **1509/2160 (69.9 %)** | **待确认**. `results_v9_noB3` is not part of this package (the author's local workspace was searched and it is absent there too). The denominator 2160 = 9×8×30 matches the stated design, and `paper_cea_v4/V4_REVISION_REPORT.md` attributes the number to the previous lineage and says it is not used, but it could not be recomputed here. |
| V1 `dmin_robot_obs` semantics | Confirmed by reading `simulation.py` (measured before `o.update`) and by the G4 replay; the archive column is **lagged**, so it is not the synchronised closest approach. Neither the archive nor its interpretation was modified. |
| "V1 criterion = robot centre in the obstacle disc" | Confirmed against `paper_cea_v4/sections/s6_discussion.tex` and `V4_REVISION_REPORT.md` (criterion `dmin_robot_obs < 1.2`), and reproduced to the run count. |
| P-classification generality | The certificate is a **sufficient** condition under the stated motion assumptions. A run-specific proof (e.g. an exhaustive reachable-set computation over the whole pre-contact window) was **not** attempted → P is a lower bound. |
| C-classification | Myopic oracle, 1.2 s lookback, 13 candidate commands → C is a **lower bound** on control failures. |
| `contact_mode="arc"` in full runs | Implemented and unit-tested for resolution, but the reported runs use `chord` (the exact model). Arc-mode run-level sensitivity was **not** run (would change the plant, not just the detector) → 待确认 if the reviewer wants it. |
| Sheep–sheep hard constraint | Not enforced (no C-level constraint for animals in V1/V2); only measured. Enforcing it would be a model change, not a physical-layer change → out of scope for Stage 1. |
| Real-platform parameters | `a_lat = 2.0`, `a_lin = 4.0`, `a_ang = 5.0`, `r_robot = 0.45` are the author's engineering values, not measured on hardware → 待确认. |
| Runtime | ~16–18 s per 1070-tick run (single core), vs 4.5 ms/tick controller time in V1. Not optimised; the detector + feasibility layer dominate. A full 2400-run V2 matrix at this cost is ~11 core-hours → not run in Stage 1. |
| `n_preexisting_contact_ss = 67` | An initial-condition property of `init_flock()` (V1 identical). Left unchanged; flagged as a separate column. |

---

## 12. Files, commands, reproduction

### New files (V1 tree untouched)

```
v2/__init__.py
v2/config_v2.py          V2Cfg, V1 reference values, config hash, parameter table
v2/limits_v2.py          (V2) box + R1-R4/lateral joint projections, counters
v2/animal_m0.py          M0 desired-velocity (V1 transcription) + integration  <- Stage-2 hook
v2/robot_safety_v2.py    soft target + hard-geometry guard (V1-transcribed)
v2/fleet_v2.py           RobotFleetV2 (omega state, 4-phase step)
v2/feasibility_v2.py     braking supervisor, prediction, escape, S/P/C
v2/contacts_v2.py        swept detector + ContactMonitor (chord / arc)
v2/sim_v2.py             run_scene_v2 (V1 loop + V2 hooks)
v2/metrics_v2.py         summarize_v2 (V1 untouched + V2 columns)
v2/runner_v2.py          single cell / batch / --jobs / --compat / --set
v2/acceptance_v2.py      tests A-I
v2/unit_tests_v2.py      transcription + projection unit tests
v2/compare_v1_v2.py      V1 / V1-solid / V2(2.0) / V2(1.2) matrix
v2/diag_v2.py            design-variant diagnostics (D2 evidence)
v2/probe_v1_repro.py     V1 archive reproduction probe
v2/extract_docx.py       manuscript text/OMML extraction (read-only)
V2_STAGE1_REPORT.md      this document
results_v2_stage1/       ACCEPTANCE.txt, compare.csv, detail*.csv, run_meta.json
```

### Commands

```powershell
# environment (as used for every number in this report)
$env:OMP_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
$env:OPENBLAS_NUM_THREADS=1; $env:NUMEXPR_NUM_THREADS=1
python -X utf8 -m v2.probe_v1_repro                  # V1 still reproducible
python -X utf8 -m v2.unit_tests_v2                   # 14 structural checks
python -X utf8 -m v2.acceptance_v2                   # A-I, writes ACCEPTANCE.txt
python -X utf8 -m v2.compare_v1_v2 --jobs 4          # V1 / V1solid / V2(2.0) / V2(1.2)
python -X utf8 -m v2.runner_v2 --scenarios S3 --seeds 2 --jobs 4 \
       --algs Ours_rollout --out results_v2_stage1
python -X utf8 -m v2.runner_v2 --scenarios S3 --seeds 2 --compat   # test A arm
python -X utf8 -m v2.runner_v2 --scenarios S3 --seeds 3 --set d_rr_safe=1.2
python -X utf8 -m v2.diag_v2 --scenarios S3 --seeds 200 201 202
```

`--set` accepts any `V2Cfg` field (`bool`, `int`, `float`, or a comma-separated
tuple, e.g. `--set brake_classes=rr,ro`) and re-hashes the configuration, so a
sensitivity run cannot be mistaken for the shipped one.
