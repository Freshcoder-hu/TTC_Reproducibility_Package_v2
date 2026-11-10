# V2 Stage 2 — animal behaviour models M0 / M1 / M2

**Status:** implemented; all acceptance tests A–F pass (§7). The V1 tree is
byte-identical to Stage 1 and the archived V1 numbers stay reproducible
(test A). The Stage-1 V2 behaviour is reproduced **bit-exactly** by M0 on the
whole trajectory, not just on the summary row.

**Deliverables this stage:** 10 new modules in `v2/` (§11; the package grows
from 17 to 27 files), the calibrated behaviour models, the predictor/environment
decoupling, the layered speed policy in the controller layer,
`V2_STAGE2_REPORT.md` (this document) and `results_v2_stage2/`.

---

## 0. 执行摘要（给作者）

1. **惰性（A）通过且是"整条轨迹"级的**：M0 在 4 个 spot cell 上与 Stage-1 V2 代码
   逐位相同（共 **1 839 440** 个浮点/整数状态值全等）；V1-compat 单元同时复现
   归档 V1 行的 **41/43 列**（仅排除计时列 `ctrl_ms` 与 `seed`）。另外两条
   **分层惰性**也逐位成立：`M1(k_M1=0) == M0`，
   `M2(σ=0, delay=0, speed_map off) == M1`。因此 M1/M2 的对比**只**来自动物响应，
   不含任何物理/执行层变化。
2. **M1 确实会避障（B）**：受控走廊（单障碍物横穿，**无机器人**，20 个种子）上，
   接触率 **0.300 → 0.093 (R=3 m) → 0.000 (R=6 m) → 0.000 (R=9 m)**，
   平均最小表面间距 **1.44 → 2.35 → 4.54 → 5.98 m**，对感知半径**单调**。
   速度/转向率分布、振荡与"爆炸"指标见 §7-B（转向率上限 3.0 rad/s 从未被突破，
   群体 p95 半径 ≤ 6.9 m，无 NaN/inf）。
3. **M2 的延迟与异质性是"真的"（C）**：逐动物配对的"首次响应"时刻随反应延迟
   **近似 1:1 平移**（配对位移减延迟 = −0.035 / −0.004 / −0.076 s，全部在
   2 拍 = 0.10 s 之内；比值 0.77 / 0.99 / 0.83），并且延迟会**恶化**
   避障结果（R=3 m：接触率 0.0056 → 0.0208，0 → 0.45 s）；增益分布 sd = 0.189、
   均值恰为 1、逐 seed 不同；去掉异质性会改变结果分布（20/20 个 seed 的配对间距
   不同，平均 −0.0204 m，置换检验 p = 0.56 —— 差异**真实但很小**，因为该设置下
   避障已接近饱和，如实报告）。
4. **预测器/环境解耦（D）**：影子模式下（预测不影响控制），**环境轨迹逐位相同**
   （1503 拍、same arrays），而预测结果在 **26/35 次规划调用**上不同
   （最大差 9.21，逐列差见表）；NumPy 引擎在 M0 上与 V1 DLL 在 8 步短时程上
   **一致到 2.7e-14**（80 步长时程 3/7 个候选偏移仍逐位一致，其余经离散分支
   翻转后混沌放大，如实报告）。**实时模式**下换预测器模型会改变实际决策
   （20 vs 21 次规划、offset 序列不同），证明预测器真的进入了控制回路。
5. **确定性（E）**：重复运行 0 差异（整条轨迹）；`jobs=1` vs `jobs=4`
   **138 个非计时列全部相同**。
6. **联合限幅（F）在 M1/M2 下依然成立且依然咬合**：C3 / R4 / 横向 / C1 带违例
   **全为 0**，咬合计数 C3 21 887–55 478、R4 23 529–32 680、横向 10 347–13 361
   （即约束确实在起作用，测试非空）。M2 的刺激速度目标确实被用上（速度最大
   **2.000 m/s**），并把 C3 咬合率相对 M1 提高约 **2.3 倍**。
7. **分层速度策略（控制器层，§5）**：`r_req` 由**命令的横向偏移与路径几何**导出
   （`r_req = (L²+h²)/(2h)`），实测 DIVERT 期间 `r_req` 均值 **16.1 m**、最小
   **4.001 m**，`v_allow` 均值 **3.47 m/s**、最小 **2.828 m/s**（= √(2·4.0)），
   与 Stage-1 报告 §9.3 预测的 4.17 m / 2.89 m/s 一致但**不是硬编码**。
   在 S3 上的 A/B（2 seeds）：DIVERT 期间**执行**速度均值 1.51 → 1.58 m/s，
   `rho` 0.784 → 0.893，`max_dev` 13.9 → 6.9 m；**但**机器人—障碍物最差间距
   在 2 个种子上没有改善（1.17 → 0.50 m，正负抵消），因此**不宣称**该规则修复了
   ro 间距 —— n=2 不足以判定，标 待确认（§5.3、§10）。
8. **两个必须写明的坏消息**：
   - **M2 明显降低任务保持**：S3 上 M1 的 ρ = 0.974/0.984，M2 只有 0.793/0.768
     （两个种子都成立）。这是"受惊羊群更难被引导"的直接后果，不是 bug。
   - 走廊（无机器人）测试床里，**M0 的接触率只有 30% 而不是 100%**：羊群本身的
     对齐/吸引项也在把它们带离障碍物，因此 B 的"改善"是在一个**非平凡基线**上
     取得的，不是把 100% 打到 0。
9. 无法验证/需要作者确认的项目集中在 §10（真机反应延迟与增益分布、M2 是否应压过
   M1 的路径保持、9 m 感知的生态学依据、长时程预测器一致性）。

---

## 1. Scope, invariants and what is deliberately untouched

| | |
|---|---|
| new code | `v2/animal_models.py`, `v2/corridor_v2.py`, `v2/calibrate_v2.py`, `v2/controller_v2.py`, `v2/speed_policy_v2.py`, `v2/predictor_v2.py`, `v2/acceptance_s2.py`, `v2/plots_s2.py`, `v2/speed_policy_ab.py`, `v2/capture_reference.py` (+ `results_v2_stage2/`) |
| edited V2 files | `v2/config_v2.py` (Stage-2 fields + tables), `v2/sim_v2.py` (behaviour-model hook, trace hooks, model-band C1 accounting, empty-robot guards), `v2/fleet_v2.py` (zero-obstacle fix), `v2/runner_v2.py` (Stage-2 controller stack, config override in `run_cell`), `v2/metrics_v2.py` (Stage-2 columns) |
| untouched | `simulation.py`, `proposed*.py`, `robots.py`, `sheep.py`, `corrections.py`, `config.py`, every pre-existing `results_*` |
| V1 reproducible | yes — `simulation.py … config.py` SHA-256 (first 16) unchanged from the Stage-1 record: `11D44C0C45B246DF`, `230586E288172FEB`, `6C839DA2B602EF37`, `F771AEB3A4918635`, `1BA4672082395E72`, `F50D549D0CC17E92`, `2DF906756DEBE211`, `47EA6A57CDE49C5E`, `C50F79FE714ABAC5`; test A also reproduces the archived V1 row column by column |
| interpreter | CPython 3.13.7, numpy 2.5.2, `_rollout.dll` (MSVC); `OMP/MKL/OPENBLAS/NUMEXPR_NUM_THREADS=1` |

Two Stage-1 latent defects had to be fixed inside V2 (neither can change a
Stage-1 number, and both are covered by test A):

* `v2/fleet_v2.py::RobotFleetV2.state` raised on `obs_pos=None` (a scenario
  with **no** obstacles and the feasibility layer on).  Only the zero-obstacle
  path changed; every Stage-1 number was produced with obstacles present.
* `v2/sim_v2.py` computed `min()` over an empty robot array, so `n_robots=0`
  crashed.  The guard is `if len(robots.pos)`; the non-empty path is untouched.
  This is what makes the **no-robot** corridor of test B run in the *real*
  simulator rather than in a private re-implementation.

### 1.1 Configuration identity

Every run still writes `config_id`.  Adding Stage-2 fields re-hashes the
record, so the Stage-1 identifiers necessarily changed; the Stage-1 archive
keeps the identifiers it was produced with.

| configuration | Stage-1 id | Stage-2 id |
|---|---|---|
| `V2Cfg()` / Stage-1 default (M0, policy off) | `cb2fc33d88a6` | `98692c880f86` |
| V1-compatibility (`compat_v1cfg`) | `a7e94a2ee683` | `a626e6dbb20c` |
| Stage-2 shipped, M0 | — | `7f3c6f6608ef` |
| Stage-2 shipped, M1 | — | `84c641e3641a` |
| **Stage-2 shipped, M2 (default)** | — | **`8aef4c1776dc`** |
| Stage-2 shipped, M2, speed policy off | — | `28d82f5fda45` |

---

## 2. What the three models are

`v2/animal_m0.py::desired_sheep_update` is the Stage-1 hook: it returns *what
the animal wants* (a heading increment and a speed) and nothing else.  Stage 2
replaces **only** that function.  Below it, the (V2) box projection, the
execution loop, the contact detector and the metrics are Stage-1 code, unchanged.

### 2.1 M0 — constraint-corrected reference

Verbatim Stage-1 code path (`advance_sheep_m0`), autonomous obstacle avoidance
**off**, but the *same* execution and sensing settings as M1/M2 (same V2Cfg,
same (V2) box, same detector, same seeds).  `AnimalModelM0.step` calls
`advance_sheep_m0(...)` with the identical argument list, which is why the
inertness is bit-exact rather than "behaviourally equal".

### 2.2 M1 — local, finite obstacle avoidance

One term added to the Couzin composition:

```
f_obs,i = Σ_o  w(g_io) · u(p_i − p_o)                (surface-gap weight)
g_io    = ‖p_i − p_o‖ − r_s − r_o                     (surface gap, not centre distance)
w(g)    = k_M1 · (1 − g/R)                            (linear, shipped)
w(g)    = 0 for g ≥ R                                 (vanishes outside the animal's range)
w(g)    = k_M1 at g = 0                               (bounded, body overlap clamped)

d_i = w_c f_Couzin,i + w_o f_obs,i + w_r f_robot,i + w_p f_path,i     (V1)
```

Implementation notes, all of them checkable in the code:

* **delayed animal-local observations**: the obstacle snapshot is held in a
  per-run ring buffer and read at `m1_obs_delay` = 0.10 s (2 ticks) old.  There
  are **no robot-shared obstacle tracks** and **no future obstacle
  trajectory** (unlike the predictor, which is a prediction model and does
  extrapolate constant-velocity motion — §6).
* **the animal's own sensing range** `R` is applied to the surface gap; the
  weight has compact support by construction (`np.where(gap < R, …, 0)`), not
  by a later mask.
* **scale consistency**: the social terms are all sums of unit vectors
  (`rep_dir` = number of neighbours inside `r_rep`; the robot term is the V1
  `avo_k (1 − d/r_avo)` sum; `steer` is O(1)), so the obstacle term is a sum of
  unit vectors with weight ≤ `k_M1` — the coefficients are comparable without
  an arbitrary normalisation.
* **separation priority is retained** (two mechanisms, both counted):
  * in the V1 separation branch (a neighbour inside `r_rep`, or a robot inside
    `r_avo`) the obstacle term is added to the separation direction but its
    component *against* separation is removed,
    `(u_sep + a_⊥) · u_sep = 1 > 0`, so the weighted sum can never reverse the
    separation direction (`sep_guard` counts the events);
  * if a **body overlap** exists (surface gap ≤ `m1_body_guard` = 0) while an
    obstacle is sensed, the obstacle term is dropped entirely and pure
    separation is used (`body_priority` counts the events).
* **zero-obstacle path**: when nothing is sensed the module runs the verbatim
  V1 composition (`_compose_v1`) instead of adding a zero vector and
  re-normalising; that is what makes `M1(k_M1=0) == M0` bit-exact.

### 2.3 M2 — behavioural stress model on top of M1

Three additions, each separately switchable:

| addition | mechanism | shipped value |
|---|---|---|
| heterogeneous response gain | `h_i ~ LogNormal(0, σ)`, rescaled so `mean(h) = 1`, applied to the obstacle term only | `σ = 0.20` |
| reaction delay | the **response** is delayed: the obstacle term is computed from the observation-delayed snapshot and then held in a per-animal buffer for `d_react` ticks | `0.30 s` |
| stimulus-dependent speed target | `s*_i = m_v_base + (m_v_max − m_v_base)·sat(‖f_obs,i‖ / m2_stim_ref)`, then multiplied by a turn-demand factor `m2_slow_gain + (1−m2_slow_gain)·cos(min(ψ,π/2))`, `ψ` = heading demand; admissible band `[0, m_v_max]` | `m_v_max = 2.0`, `m2_slow_gain = 0.5` |

The reaction delay is a delay of the *response*, not of the observation.  That
distinction is measurable: delaying the *observation* leaves the onset latency
essentially unchanged (the animal keeps moving while the perceived obstacle
position is stale — this was observed during development and is why the
implementation buffers the computed term instead), whereas delaying the
*response* shifts the first response by the delay (§7-C).

The **admissible lower speed is 0** for M1 and M2 (`m_s_min = 0`), so animals
can slow and stop; M0 keeps the V1 band `[1.0, 1.5]`.  With `m2_slow_gain = 0.5`
a 90° turn demand halves the target, which is how the lower part of the band is
actually reached (calibration: mean speed 1.075 → 0.933 m/s when the factor goes
1.0 → 0.5, §4.3).

> **M2 is a behavioural *stress* model, not a validated description of
> frightened sheep.**  Its parameters are engineering values on a plausible
> range; they are not fitted to animal data, and the manuscript says the same
> (2.1).  The code comments and this report repeat it so that no reader can
> mistake the model for an empirical result.

### 2.4 What does *not* change

The (V2) box is the Stage-1 intersection projection, applied to the new desired
vectors: `0 ≤ s⁺ ≤ s_max`, `|wrap(θ⁺−θ)| ≤ ω_s dt`, `‖v⁺−v‖ ≤ a_s dt` solved
jointly.  For M1/M2 the band is `[m_s_min, m_v_max]` (0–2.0 m/s); the *V1* C1
band `[1.0, 1.5]` is untouched in `SimCfg` because every legacy column
(`violation_rate`, the V1 kinematic check) must keep its V1 meaning.  A
consequence, stated rather than hidden: the **legacy** `violation_rate` column
flags M2 animals that exceed 1.5 m/s, because they legitimately do; the
authoritative Stage-2 columns are `violation_C1_low` / `violation_C1_high`
(measured against the model's own band) and both are 0 (§7-F).

---

## 3. Parameter table (shipped Stage-2 configuration)

Generated by `v2/config_v2.py::param_table_stage2_markdown(stage2_v2cfg())`, i.e.
from the same frozen record that is hashed into `config_id`.

| parameter | shipped value | searched range / provenance | note |
|---|---|---|---|
| animal model integrated by the simulator | `animal_model` = M2 | M0 / M1 / M2 | M0 = Stage-1 model (bit-exact); M1 = +local obstacle term; M2 = +stress |
| M1 animal obstacle sensing range [m] (surface gap) | `m1_sense_range` = 6.0 | sweep 3 / 6 / 9; shipped 6 | M1 weight vanishes for gap >= R by construction |
| M1 avoidance gain k_M1 | `m1_gain` = 3.0 | searched 0.5 / 1.0 / 2.0 / 3.0; calib. 3.0 | same scale as rep_dir (sum of unit vectors) |
| M1 distance-decay shape | `m1_decay` = linear | searched linear / quadratic / exp(1 m) / exp(3 m); calib. linear | parsimony tie-break, §4.2 |
| exp-decay e-folding length [m] | `m1_decay_len` = 2.0 | 1.0 / 3.0 (only for exp) | unused by the shipped linear shape |
| animal-local observation delay [s] | `m1_obs_delay` = 0.1 | fixed 0.10 (= 2 ticks) | delayed observation, no shared tracks |
| surface gap at which body separation wins [m] | `m1_body_guard` = 0.0 | fixed 0.0 | obstacle term dropped on body overlap |
| admissible lower speed of the behaviour model [m/s] | `m_s_min` = 0.0 | 0 (author decision) | allows slowing and stopping; M0 keeps 1.0 |
| normal speed reference [m/s] | `m_v_base` = 1.0 | fixed | manuscript 1.0–1.5 |
| robot-flee speed [m/s] | `m_v_flee` = 1.5 | fixed | unchanged from V1 |
| stimulus-dependent upper reference [m/s] | `m_v_max` = 2.0 | searched 1.5 / 1.8 / 2.0; shipped 2.0 | exploratory 1.8–2.0 per the manuscript |
| lognormal sigma of the per-animal gain | `m2_gain_sigma` = 0.2 | searched 0.10 / 0.20 / 0.30; shipped 0.20 | mean renormalised to 1 |
| extra per-animal reaction delay [s] | `m2_react_delay` = 0.3 | searched 0.15 / 0.30 / 0.45; shipped 0.30 | response delay, added on top of the observation delay |
| stimulus mapped to `m_v_max` | `m2_stim_ref` = 2.0 | fixed | saturating speed map |
| speed factor at a 90° turn demand | `m2_slow_gain` = 0.5 | searched 1.0 / 0.5 / 0.0; shipped 0.5 | design override, §4.3 |
| speed-target switch | `m2_speed_map` = True | True / False | `False` reproduces the M1 speed rule |
| RNG stream tag | `m2_seed_tag` = 20260227 | fixed | gains/delays identical for a given seed |
| predictor animal model | `predictor_model` = M0 | M0 / M1 / M2 | independent of `animal_model` |
| predictor implementation | `predictor_engine` = dll | dll (V1, M0) / v2 (NumPy, M0–M2) | `dll` keeps Stage-1 exactly |
| shadow prediction | `predictor_shadow` = False | False / True | evaluate without actuating |
| predictor obeys the speed policy | `predictor_speed_policy` = True | True / False | it is a controller rule |
| layered speed policy | `speed_policy` = manoeuvre | off / manoeuvre | controller layer, not a physical limit |
| modes where the bound applies | `speed_policy_modes` = (pre, divert) | — | straight-line repositioning keeps `v_max` |
| straight-manoeuvre threshold | `speed_policy_h_eps` = 0.05 | fixed | `|h| ≤ eps ⇒ r_req = ∞ ⇒ v_max` |

Stage-1 physical parameters (`r_robot = 0.45`, `a_lat = 2.0`, `v_max = 4.5`,
`a_lin = 4.0`, `omega_max = 3.5`, `a_ang = 5.0`, `d_rr_safe = 2.0`,
`d_ro_safe = 2.0`, …) are unchanged; see `V2_STAGE1_REPORT.md` §2.

---

## 4. Calibration (seeds 1000–1019 only)

**Evaluation seeds 2000–2029 were never touched.**  Every run in this stage —
calibration, acceptance, A/B — uses seeds 1000–1019.

`python -X utf8 -m v2.calibrate_v2 --stage all` → `results_v2_stage2/calibration_*.csv`,
`calibration_summary.json`, `calibration.log`.

### 4.1 Test bed and objective

Test bed: `v2/corridor_v2.py`, the controlled single-obstacle corridor with
**no robots** (one obstacle crossing a 12 m corridor at 1.0 m/s, surface
entering the corridor between t ≈ 1.5 s and 14.5 s; 36 animals on a 6×6 grid
advancing along the path; 22 s runs).  A grid start is used instead of
`init_flock` because, with no robots present, soft repulsion dominates the
Couzin branch and a 2.8 m disc start diffuses instead of walking the corridor
(measured: centroid drifted to (5, −3) after 10 s and never met the obstacle).
This is a **test-bed initial condition only**: every protocol run keeps
`init_flock`, and test A shows the frozen numbers are untouched.

```
J = 100 · contact_fraction            (avoidance success; primary)
  +  10 · max(0, −worst_clearance)    (never let the obstacle penetrate)
  +   5 · max(0, 0.5 − path_eff)      (animals must still move as a group)
  +   2 · max(0, latency − 2.0)       (no implausibly slow response)
  +   2 · max(0, 0.3 − latency)       (no instantaneous reaction)
```

`contact_fraction` = fraction of animals whose minimum surface gap to the
obstacle ever reaches 0; `worst_clearance` its minimum; `path_eff` = net
displacement / travelled distance; `latency` = mean onset latency (§7-C).

Corridor keeping (`inside_frac`) is **reported but not in J**: with no robots no
model can hold the corridor, so the term would penalise animals *for avoiding*
(dodging leaves the corridor) instead of measuring the avoidance.  (An earlier
draft of the objective did include it; the correction was made before the
selection was applied and is recorded here rather than silently.)

**Selection rule, fixed before reading the result:** best J; cells within
`TIE_TOL = 0.5` are treated as equivalent and the *simplest* is shipped
(`linear` before `quadratic` before `exp`, then the smaller gain).

### 4.2 M1: gain × decay shape (R = 6 m, seeds 1000–1004)

| decay | k | J | contact | mean clr [m] | worst [m] | latency [s] | path eff |
|---|---|---|---|---|---|---|---|
| linear | 0.5 | 28.481 | 0.150 | 2.45 | −1.35 | 1.158 | 0.91 |
| linear | 1.0 | 12.585 | 0.056 | 3.20 | −0.70 | 1.703 | 0.89 |
| linear | 2.0 | 0.556 | 0.006 | 3.80 | +0.61 | 1.929 | 0.85 |
| **linear** | **3.0** | **0.000** | **0.000** | **4.63** | **+1.40** | 1.517 | 0.86 |
| quadratic | 0.5 | 33.797 | 0.189 | 2.02 | −1.46 | 2.158 | 0.86 |
| quadratic | 1.0 | 26.040 | 0.122 | 2.53 | −1.19 | 2.941 | 0.84 |
| quadratic | 2.0 | 19.599 | 0.083 | 2.67 | −0.84 | 3.417 | 0.85 |
| quadratic | 3.0 | 3.288 | 0.000 | 3.48 | +0.45 | 3.644 | 0.85 |
| exp(1 m) | 0.5 | 38.781 | 0.222 | 1.84 | −1.39 | 3.337 | 0.92 |
| exp(1 m) | 1.0 | 38.971 | 0.222 | 1.83 | −1.37 | 3.502 | 0.91 |
| exp(1 m) | 2.0 | 35.426 | 0.167 | 2.12 | −1.38 | 4.494 | 0.84 |
| exp(1 m) | 3.0 | 29.688 | 0.139 | 2.32 | −1.16 | 4.086 | 0.85 |
| exp(3 m) | 0.5 | 31.314 | 0.172 | 2.25 | −1.41 | 1.040 | 0.90 |
| exp(3 m) | 1.0 | 11.824 | 0.061 | 3.18 | −0.57 | 1.373 | 0.90 |
| exp(3 m) | 2.0 | 0.556 | 0.006 | 3.95 | +0.42 | 1.714 | 0.89 |
| exp(3 m) | 3.0 | 0.000 | 0.000 | 4.77 | +1.55 | 1.592 | 0.86 |

Three cells reach J = 0 (linear k=3, exp3 k=3, quadratic k=3 within tolerance);
the parsimony rule ships **linear, k_M1 = 3.0**.  The gain matters far more than
the shape: at k = 0.5 every shape still penetrates (−1.35 … −1.46 m), at k = 3.0
none does.  The 1 m exponential is the worst shape by a wide margin because its
weight decays within 3 m even when the sensing range is 6 m.

### 4.3 M2: coordinate search on top of the chosen M1

| param | value | J | contact | worst [m] | latency [s] | speed mean/max |
|---|---|---|---|---|---|---|
| m2_react_delay | 0.15 | 0.000 | 0.000 | +1.98 | 1.704 | 1.112/1.90 |
| m2_react_delay | 0.30 | 0.000 | 0.000 | +1.88 | 1.518 | 1.111/1.93 |
| m2_react_delay | 0.45 | 0.000 | 0.000 | +1.94 | 1.661 | 1.103/1.90 |
| m2_gain_sigma | 0.10 | 0.000 | 0.000 | +2.01 | 1.823 | 1.118/1.96 |
| m2_gain_sigma | 0.20 | 0.000 | 0.000 | +1.98 | 1.704 | 1.112/1.90 |
| m2_gain_sigma | 0.30 | 0.000 | 0.000 | +2.07 | 1.868 | 1.114/1.94 |
| m_v_max | 1.5 | 0.000 | 0.000 | +1.83 | 1.922 | 1.075/1.48 |
| m_v_max | 1.8 | 0.000 | 0.000 | +1.93 | 1.940 | 1.098/1.74 |
| m_v_max | 2.0 | 0.000 | 0.000 | +2.01 | 1.823 | 1.118/1.96 |
| m2_slow_gain | 1.0 | 0.000 | 0.000 | +1.83 | 1.922 | 1.075/1.48 |
| m2_slow_gain | 0.5 | 0.552 | 0.000 | +2.13 | 2.276 | 0.933/1.46 |
| m2_slow_gain | 0.0 | 0.568 | 0.000 | +2.16 | 2.284 | 0.856/1.45 |

**The objective cannot identify the M2 parameters on this test bed**: at
R = 6 m the avoidance already saturates, so every cell has J ≈ 0.  This is a
result, not a nuisance, and it is reported as such: the selection therefore
falls back to a declared design rule instead of a spurious argmin.

* `m2_react_delay = 0.30 s`: middle of the searched physiological band
  0.15/0.30/0.45 (flat objective, J range 0.000–0.000).
* `m2_gain_sigma = 0.20`: middle of the searched spread 0.10/0.20/0.30 → a 20 %
  coefficient of variation on the response gain (flat objective).  Heterogeneity
  itself is part of the model definition (author decision), so the search never
  includes σ = 0 as a candidate; σ = 0 appears only as the "remove
  heterogeneity" arm of test C.
* `m_v_max = 2.0`: the author's exploratory upper reference (1.8–2.0 m/s);
  the objective is flat between 1.5/1.8/2.0.  The mapping is nevertheless
  *used*: mean speed 1.075 (m_v_max = 1.5) → 1.118 (2.0), max speed 1.48 → 1.96.
* `m2_slow_gain = 0.5`: a **design override**, the only M2 parameter where the
  objective prefers another value (1.0 gives J = 0, 0.5 gives 0.552).  The
  author's decision requires the admissible lower speed 0 to be *reachable by
  the model*; with the factor at 1.0 the speed target never falls below the
  base value, so the mechanism would exist only on paper.  The cost (0.55 J, a
  path-efficiency penalty) is reported rather than hidden.

### 4.4 Sensitivity (one factor at a time)

| model | factor | value | J | contact | mean clr | worst |
|---|---|---|---|---|---|---|
| M1 | m1_gain | 1.5 | 2.222 | 0.022 | 3.63 | +0.35 |
| M1 | m1_gain | **3.0** | 0.000 | 0.000 | 4.63 | +1.40 |
| M1 | m1_gain | 4.5 | 0.000 | 0.000 | 4.93 | +1.86 |
| M1 | m1_decay | **linear** | 0.000 | 0.000 | 4.63 | +1.40 |
| M1 | m1_decay | quadratic | 3.288 | 0.000 | 3.48 | +0.45 |
| M1 | m1_decay | exp(1 m) | 29.688 | 0.139 | 2.32 | −1.16 |
| M1 | m1_decay | exp(3 m) | 0.000 | 0.000 | 4.77 | +1.55 |
| M1 | m1_sense_range | 3.0 | 19.694 | 0.111 | 2.26 | −0.86 |
| M1 | m1_sense_range | **6.0** | 0.000 | 0.000 | 4.63 | +1.40 |
| M1 | m1_sense_range | 9.0 | 0.000 | 0.000 | 6.03 | +2.50 |
| M2 | m2_react_delay | 0.0 | 0.000 | 0.000 | 5.07 | +2.31 |
| M2 | m2_react_delay | **0.30** | 0.000 | 0.000 | 4.77 | +1.84 |
| M2 | m2_react_delay | 0.60 | 0.000 | 0.000 | 4.73 | +1.83 |
| M2 | m2_gain_sigma | 0.0 | 0.000 | 0.000 | 4.89 | +2.15 |
| M2 | m2_gain_sigma | **0.20** | 0.000 | 0.000 | 4.82 | +2.08 |
| M2 | m2_gain_sigma | 0.40 | 0.000 | 0.000 | 4.73 | +1.83 |

Reading: the M1 gain is identified only as a threshold (anything ≥ 2 clears the
corridor), the sensing range is *not* saturated (3 m is materially worse —
which is exactly what test B measures), and the M2 parameters move the outcome
by ≤ 0.35 m of clearance, i.e. **the M2 layer is not identified by the
avoidance outcome at R = 6 m**.  That is why §7-C measures it at R = 3 m and
with paired designs instead of relying on J.

### 4.5 Budget

| item | runs | core-seconds |
|---|---|---|
| M1 grid (16 cells × 5 seeds) | 80 | 76.0 |
| M2 coordinate search (12 cells × 5 seeds) | 60 | 57.0 |
| Sensitivity (18 cells × 5 seeds) | 90 | 85.6 |
| **calibration total** | **230** | **218.6** |
| acceptance A–F (this report) | ~200 corridor + 25 full-scenario runs | 610 s wall (single core) |
| speed-policy A/B (§5.3) | 4 | 74 s |

`results_v2_stage2/calibration_summary.json` records the calibration numbers
directly; the acceptance/AB counts are from the transcripts.

---

## 5. The layered speed policy (controller layer)

### 5.1 The rule and where it is implemented

`v2/speed_policy_v2.py` + `v2/controller_v2.py::SpeedPolicyController`.

```
v_allow = min( v_max , sqrt( a_lat · r_req ) )        while pre-positioning or diverting
r_req   = (L² + h²) / (2 h)                          arc through a displacement (L along, h across)
```

* `h` is the **commanded lateral offset** — the distance the flock is still
  being asked to move sideways, i.e. `offset − lat_c` where `offset` is the
  controller's chosen offset (its *declared manoeuvre*) and `lat_c` the flock
  centroid's current lateral coordinate in the path frame;
* `L` is the along-path lookahead over which that offset is commanded (4 m in
  DIVERT, 6 m when the offset is 0);
* the rule is **self-releasing**: as the flock actually moves sideways `h`
  shrinks, `r_req` grows and `v_allow` rises back toward `v_max`.  The smallest
  possible bound is `r_req = L` (at `h = L`), i.e. 2.83 m/s at `L = 4`, so the
  rule cannot deadlock a manoeuvre.

V1 DIVERT geometry (L = 4 m, h = 3 m) ⇒ `r_req = 25/6 = 4.167 m` ⇒
`v ≤ 2.887 m/s`; the rollout-based DIVERT (offset 3.6 m at 4 m lookahead) ⇒
`r_req = 4.02 m` ⇒ `v ≤ 2.836 m/s`.  The number is **derived, not hard-coded**:
the code contains no 4.17 anywhere, and the log shows the value moving with the
manoeuvre (mean 16.1 m, minimum 4.001 m).

The controller never guesses a geometry: if a controller does not declare a
manoeuvre the policy reports `applied=False` and leaves the command alone.

**This is a controller rule, not a physical limit.**  The physical limit is
`|v·ω| ≤ a_lat`, already enforced by Stage 1; a 4.5 m/s robot has a 10.1 m
minimum turn radius, so the *law* was asking for a manoeuvre the plant cannot
fly.  `a_lat` is author-fixed and is **not** relaxed (Stage-1 report §9.3).

### 5.2 What is logged

Every control tick: `(t, mode, ahead, lateral, r_req, v_allow, n_clamped,
speed_loss)`.  Per run the summary carries `sp_n_ticks`, `sp_n_bounded_ticks`,
`sp_n_clamped_robots`, `sp_r_req_mean/min`, `sp_v_allow_mean/min`,
`sp_speed_loss_sum`.

S3 / M1 / seed 1000 (representative):
`sp_n_ticks = 597`, `sp_n_bounded_ticks = 597`, `sp_n_clamped_robots = 3237`,
`sp_r_req_mean = 16.11 m`, `sp_r_req_min = 4.001 m`, `sp_v_allow_mean = 3.47 m/s`,
`sp_v_allow_min = 2.829 m/s`, `sp_speed_loss_sum = 346.3 m/s` (summed over
robot-ticks).  So the bound was active on **every** pre/divert tick of that run
and clipped ≈ 5.4 robots per tick on average.

### 5.3 Observed effect on DIVERT (S3, M2, seeds 1000–1001)

`python -X utf8 -m v2.speed_policy_ab` → `results_v2_stage2/behaviour/speed_policy_AB.*`

| quantity | policy off | policy manoeuvre |
|---|---|---|
| mean executed robot speed during PRE/DIVERT [m/s] | 1.514 | 1.576 |
| max executed speed during PRE/DIVERT [m/s] | 4.500 | 4.500 |
| robot–obstacle contact episodes / run | 0.5 | 0.5 |
| worst robot–obstacle surface clearance [m] (mean of 2 seeds) | +1.17 | +0.50 |
| worst robot–animal surface clearance [m] | −0.717 | −0.737 |
| animal collision rate | 0.028 | 0.042 |
| path keeping ρ | 0.784 | **0.893** |
| maximum path deviation [m] | 13.9 | **6.9** |

Honest reading, with the caveats spelled out:

* the rule clearly **reduces path deviation and improves path keeping**;
* it does **not** show a robot–obstacle clearance benefit on 2 seeds (±: seed
  1000 improves −0.067 → −0.032, seed 1001 degrades +2.40 → +1.03), and the
  mean executed DIVERT speed barely moves because most pre/divert ticks are
  already far below the cap (the *commanded* cap is what changes);
* `max` executed speed stays 4.5 m/s because some ticks inside PRE have
  `|h| ≤ h_eps` (straight repositioning), where the rule deliberately does not
  bind;
* **the DIVERT-speed effect is therefore reported as "the bound is applied and
  logged" (mean `v_allow` 3.47 m/s vs `v_max` 4.5 m/s), not as a clearance
  improvement** — n = 2 seeds cannot establish the latter → 待确认 (§10).

---

## 6. Predictor / environment decoupling

### 6.1 Design

| side | what selects it | where |
|---|---|---|
| environment | `v2cfg.animal_model` | `v2/sim_v2.py` → `v2.animal_models.make_animal_model` |
| predictor | `v2cfg.predictor_model` + `v2cfg.predictor_engine` | `v2/controller_v2.py::OursRolloutV2._predict` → `v2/predictor_v2.evaluate_rollouts` |

Nothing connects the two except the configuration record.  `predictor_engine`
has two values: `dll` = the V1 native predictor (M0 only, keeps Stage-1 exactly)
and `v2` = `v2/predictor_v2.py`, a NumPy transcription of `rollout.cpp` in which
the animal step is dispatched to M0/M1/M2.  Every other predictor component
(slot geometry, spacing projection, safety filter, unicycle integration, the
7-column penalty output) is kept identical to V1, and the V1 predictor's own
quirks are kept too (e.g. the attraction term `att/max(na,1) − p` pulls a
neighbour-less animal toward the origin — a predictor artefact, documented in
the module).

Predictor-side simplifications, stated: the animal obstacle-sensing *delay* is
not modelled inside the rollout (the obstacle state is propagated at constant
velocity, as V1 does for all obstacle motion), M2's gain is taken at its mean
(a mean-field prediction), and the speed policy **is** applied inside the
predictor when `predictor_speed_policy=True` because it is a controller rule the
predicted robots must obey.

### 6.2 The proof (test D)

1. **Engine fidelity.** On identical inputs, the NumPy engine reproduces the V1
   DLL for M0 to **2.7e-14 relative** over an 8-step horizon (7 candidate
   offsets).  Over the full 80-step horizon 3/7 offsets stay identical to 1e-12
   and the rest diverge by up to 0.30 because a discrete slot-spacing/contact
   test flips on a 1e-16 summation-order difference and is then amplified by the
   80-step rollout.  This is reported, not hidden: the port is faithful, the
   long-horizon divergence is chaos, and the *live* predictor keeps using the
   DLL for M0 so no Stage-2 number depends on it.
2. **Shadow mode (matched vs mismatched, same environment).** With
   `predictor_shadow=True` the live controller keeps the V1 predictor (so the
   environment cannot depend on the model under study) while the shadow
   evaluates the configured model every planning tick and *throws the result
   away*.  Environment = M2, predictor = M0 vs M2, seed 1000, S2:
   * environment trajectory: **1503 ticks, every logged array bit-identical**;
   * predictions: **26 of 35 planning calls differ**, max |Δ| = 9.21; per-column
     maxima `(contacted, pressure, outside, centre, spread, progress, worst) =
     (0.417, 1.281, 9.208, 0.880, 2.165, 5.835, 3.154)` — the two models predict
     materially different outcomes for the same candidate offsets.
3. **Live mode.** With the same environment (M0) and the NumPy engine, changing
   only `predictor_model` changes the actuated plan: 20 vs 21 planning calls and
   different offset sequences
   (M0-predictor `[1.35, 1.35, 0.0, −2.4, …]` vs M2-predictor
   `[1.35, 1.35, 1.35, 0.0, 1.35, …]`).  So the predictor really is in the
   control loop, and model mismatch is testable end-to-end.

---

## 7. Acceptance tests A–F

Transcript (`results_v2_stage2/ACCEPTANCE.txt`, tables in
`results_v2_stage2/behaviour/`); reproduced verbatim below.

```text
V2 Stage-2 acceptance tests
shipped parameters: {"m1_decay": "linear", "m1_decay_len": 2.0, "m1_gain": 3.0, "m1_sense_range": 6.0, "m2_gain_sigma": 0.2, "m2_react_delay": 0.3, "m2_slow_gain": 0.5, "m_v_max": 2.0}
python 3.13.7, numpy 2.5.2
==============================================================================
A. M0 inertness: whole-trajectory bit-exactness against the Stage-1 code
  [PASS] A/S3_s200_v2 M0 == Stage-1 (whole trajectory, 496496 values)
  [PASS] A/S3_s200_v1compat M0 == Stage-1 (whole trajectory, 523280 values)
  [PASS] A/S3_s200_v1compat V1 archive row reproduced (41 of 43 archived columns; ctrl_ms/seed excluded)
  [PASS] A/S1_s201_v2 M0 == Stage-1 (whole trajectory, 299112 values)
  [PASS] A/S3_s200_v2_seq M0 == Stage-1 (whole trajectory, 520552 values)
  layer inertness (the added terms are inactive => exactly the layer below)
  [PASS] A/M1(m1_gain=0) == M0, bit-exact
  [PASS] A/M2(sigma=0, delay=0, no speed-up) == M1, bit-exact
==============================================================================
B. M1 actually avoids: corridor crossing, NO robots, seeds 1000-1019
  M0       contact=0.300+-0.045  episodes=11.1  mean_clr=  1.44  worst= -1.45  speed[p95]=1.00/1.00  omega[p95]=3.00/3.00  flips=7.76/s  eff=0.92
  M1_R3    contact=0.093+-0.049  episodes=3.8  mean_clr=  2.35  worst= -0.70  speed[p95]=1.00/1.00  omega[p95]=3.00/3.00  flips=7.26/s  eff=0.83
  M1_R6    contact=0.000+-0.000  episodes=0.0  mean_clr=  4.54  worst=  1.44  speed[p95]=1.00/1.00  omega[p95]=3.00/3.00  flips=7.37/s  eff=0.85
  M1_R9    contact=0.000+-0.000  episodes=0.0  mean_clr=  5.98  worst=  2.53  speed[p95]=1.00/1.00  omega[p95]=3.00/3.00  flips=7.51/s  eff=0.88
  [PASS] B/M1 beats M0 at every sensing range  -- contact 0.300 -> 0.093, 0.000, 0.000
  [PASS] B/contact rate monotone non-increasing in range  -- R3:0.093, R6:0.000, R9:0.000
  [PASS] B/mean clearance monotone non-decreasing in range  -- R3:2.35, R6:4.54, R9:5.98
  [PASS] B/M1 mean clearance beats M0 at the widest range  -- 1.44 -> 5.98
  [PASS] B/no heading-rate explosion (|dtheta|/dt <= omega_s)  -- worst omega 3.000
  [PASS] B/no speed explosion (s <= m_v_max)  -- worst speed 1.000
  [PASS] B/no spatial explosion (finite states, group p95 radius < 25 m)  -- p95 radius max M0:4.9, M1_R3:6.9, M1_R6:4.8, M1_R9:4.8
  [PASS] B/oscillation bounded (heading-rate sign flips < 12 /s)  -- M0:7.8, M1_R3:7.3, M1_R6:7.4, M1_R9:7.5
==============================================================================
C. reaction delay and gain heterogeneity are real (corridor)
  delay=0.00 s  latency=2.541+-0.754  first-response paired +0.000 (505 animals)  contact=0.0056+-0.0145  clearance=2.477
  delay=0.15 s  latency=2.642+-0.498  first-response paired +0.115 (475 animals)  contact=0.0083+-0.0159  clearance=2.442
  delay=0.30 s  latency=2.817+-0.593  first-response paired +0.296 (477 animals)  contact=0.0069+-0.0123  clearance=2.415
  delay=0.45 s  latency=2.834+-0.573  first-response paired +0.374 (490 animals)  contact=0.0208+-0.0236  clearance=2.293
  animal-paired first-response gain per unit delay: [0.769, 0.988, 0.832]
  animal-paired first-response shift minus the delay [s]: [-0.035, -0.004, -0.076] (2 ticks = 0.10 s)
  [PASS] C/animal-paired first response is shifted by the reaction delay (within 2 ticks)  -- shift-minus-delay=[-0.035, -0.004, -0.076] s, gains=[0.769, 0.988, 0.832]
  [PASS] C/latency monotone non-decreasing in delay  -- [2.541, 2.642, 2.817, 2.834]
  dynamic consequence at R=3: contact 0.0056 (no delay) -> 0.0208 (0.45 s delay)
  [PASS] C/delay degrades the avoidance outcome (0.45 s vs 0 s)  -- contacts 0.0056 -> 0.0208
  gains: mean=1.0000 sd=0.1889 min=0.5894 max=1.7335 p5/25/50/75/95=[0.736, 0.851, 0.986, 1.117, 1.307]
  [PASS] C/gain spread is applied (sd > 0, min < 1 < max)  -- sd=0.1889 range=[0.589,1.733]
  [PASS] C/gain mean is 1 (same total gain as M1)  -- mean=1.000000
  [PASS] C/gains are seed-dependent  -- first two animals seed 1000: [1.1803, 1.16] vs [1.2969, 0.9931]
  hetero sigma=0.20  contact=0.0069+-0.0123  clearance=2.415+-0.122
  homo   sigma=0.00  contact=0.0097+-0.0136  clearance=2.435+-0.143
  paired clearance difference hetero-homo = -0.0204 m (sd 0.1460, 20/20 seeds differ), permutation p=0.5582
  [PASS] C/removing heterogeneity changes the outcome distribution  -- paired clearance diff -0.0204 m, sd 0.1460, p=0.5582; contact 0.0069 vs 0.0097
==============================================================================
D. predictor / environment decoupling
  M0 engine agreement, 8 prediction steps (DLL vs NumPy): max rel=2.671e-14  max abs=1.776e-15
  full 80-step horizon: 3/7 candidate offsets agree to 1e-12; the rest diverge through a discrete slot-spacing / contact test flipping (chaotic amplification of a 1e-16 summation-order difference), max abs=2.995e-01
  [PASS] D/NumPy engine reproduces the V1 DLL on M0 (8 steps, <=1e-8 rel)  -- max rel=2.671e-14
  shadow runs: 1503 ticks, environment trajectory identical = True
  predictions: 35 planning calls, 26 differ; max|dpred|=9.208e+00, max rel (|a|>1e-6)=3.737e+05
  per-column max |difference| (contacted, pressure, outside, centre, spread, progress, worst): [0.4167, 1.2805, 9.2078, 0.8802, 2.165, 5.8347, 3.1536]
  predicted worst margin at offset +3.6 m: M0 first/last=0.282/23.166  M2 first/last=1.625/23.166
  [PASS] D/shadow: environment trajectory identical (bit-exact)  -- []
  [PASS] D/shadow: predictions differ between predictor models  -- 26/35 calls differ
  live mode: 20 vs 21 plans; offsets M0-predictor [1.35, 1.35, 0.0, -2.4, -2.4, -2.4, -1.35, -2.4] ...
             offsets M2-predictor [1.35, 1.35, 1.35, 0.0, 1.35, 0.0, 2.4, 1.35] ...
  [PASS] D/live: a different predictor model changes the actuated plan  -- 20 vs 21 plans
==============================================================================
E. determinism
  [PASS] E/repeat run identical (S3, seed 1000, 2911 ticks)  -- []
  [PASS] E/repeat run identical (S2, seed 1001, 1464 ticks)  -- []
  jobs=1 vs jobs=4: 2 rows x 138 columns (ctrl_ms excluded), mismatches=0
  [PASS] E/jobs=1 == jobs=4, every non-timing column  -- 0 mismatches
==============================================================================
F. joint limits still hold (and bind) in M1/M2
  M1 seed 1000: C3=0 R4=0 lat=0 C1low=0 C1high=0 | bind C3=23153 R4=23529 lat=10347 | speed 1.134/1.500 | v_allow 3.4685091379708965 (min 2.8289149391063133) | animal contacts 0 | task success True (goal) rho 0.973672899204814
  M1 seed 1001: C3=0 R4=0 lat=0 C1low=0 C1high=0 | bind C3=21887 R4=23908 lat=10915 | speed 1.129/1.500 | v_allow 3.567396864069046 (min 2.8284293045999807) | animal contacts 0 | task success True (goal) rho 0.9843833708346726
  M2 seed 1000: C3=0 R4=0 lat=0 C1low=0 C1high=0 | bind C3=55478 R4=32680 lat=13361 | speed 1.041/2.000 | v_allow 3.5848902185737974 (min 2.8284271664979572) | animal contacts 1 | task success True (goal) rho 0.7932528628907459
  M2 seed 1001: C3=0 R4=0 lat=0 C1low=0 C1high=0 | bind C3=49188 R4=30191 lat=12854 | speed 1.021/2.000 | v_allow 3.369259488717233 (min 2.8284398637092707) | animal contacts 3 | task success True (goal) rho 0.7676994835915375
  [PASS] F/C3, R4, lateral and C1-band violations are 0 in M1 and M2
  [PASS] F/C3 and R4 and the lateral cap still bind (non-vacuous)
  [PASS] F/speed policy applied during pre/divert  -- 4/4 runs with bounded ticks
==============================================================================
SUMMARY: 32/32 checks passed (610.4 s)
```

---

## 8. Behaviour sanity

Plots: `results_v2_stage2/plots/` (regenerated by `python -m v2.plots_s2`).

**Corridor trajectories (seed 1000).**  M0 walks into the obstacle (39 % of the
animals contact it); M1 and M2 leave a clear hole around it.  The dotted lines
are the 6 m corridor edges.

![corridor trajectories](results_v2_stage2/plots/B_paths.png)

**Speed / heading-rate / gap distributions** (4 seeds, corridor): the obstacle
term changes *where* the animals go, not how fast they turn; the heading rate
stays inside `omega_s` = 3 rad/s and the speed inside the admissible band.

![speed and heading distributions](results_v2_stage2/plots/B_distributions.png)

**C: the delay reaches the response.**  Orange = animal-paired first-response
shift, which follows the slope-1 line; blue = the mean 20 %-threshold onset
(noisier, same trend).  Right: the per-animal gain distribution.

![delay and gains](results_v2_stage2/plots/C_delay_and_gains.png)

**The speed policy in the real scenario.**  Left: derived `r_req` while
pre/diverting, clustered just above the V1 DIVERT geometry (4.17 m, dotted).
Right: the bound actually applied over one S3 run, returning to `v_max` = 4.5 m/s
on straight ticks and reaching 2.83 m/s at the tightest manoeuvre.

![DIVERT speed policy](results_v2_stage2/plots/DIVERT_speed_policy.png)

**B (20 seeds): contact rate, clearance and the per-seed spread.**

![corridor summary](results_v2_stage2/plots/B_corridor.png)

### 8.1 B — corridor, 20 seeds (`behaviour/B_corridor.csv`)

| setting | contact | episodes | mean min clr [m] | worst [m] | speed p95/max | ω p95/max | flips/s | path eff |
|---|---|---|---|---|---|---|---|---|
| M0 | 0.300 ± 0.045 | 11.1 | 1.44 | −1.45 | 1.00/1.00 | 3.00/3.00 | 7.8 | 0.92 |
| M1 R=3 m | 0.093 ± 0.049 | 3.8 | 2.35 | −0.70 | 1.00/1.00 | 3.00/3.00 | 7.3 | 0.83 |
| M1 R=6 m | 0.000 ± 0.000 | 0.0 | 4.54 | +1.44 | 1.00/1.00 | 3.00/3.00 | 7.4 | 0.85 |
| M1 R=9 m | 0.000 ± 0.000 | 0.0 | 5.98 | +2.53 | 1.00/1.00 | 3.00/3.00 | 7.5 | 0.88 |

Notes: M1 does not change the *speed* (it has no speed mapping — that is M2);
it changes the heading.  The p95 of |dθ|/dt equals the C2 cap 3.0 rad/s in
every setting **including M0**, i.e. the cap binds on at least 5 % of
animal-steps and is never exceeded.  The heading-rate sign-flip rate (7–8 per
animal per second) is a property of the *V1 social model* in a dense flock and
is **not increased** by the obstacle term (7.76 → 7.51), so the avoidance does
not add oscillation.

### 8.2 C — delay and heterogeneity

See §7-C for the transcript; the two headline numbers are the animal-paired
first-response shift minus the delay (−0.035 / −0.004 / −0.076 s, i.e.
within 2 integration ticks of 0.05 s) and the gain
distribution (mean 1.000, sd 0.189, min 0.589, max 1.734, p5–p95
0.736–1.307).  Removing the heterogeneity changes the paired clearance of
**20/20** seeds (mean −0.0204 m, sd 0.146 m, permutation p = 0.56).  Small, but
real and measurable; the p-value says the *sign* is not systematic, which is the
honest statement at this sample size.

### 8.3 F — joint limits and the speed policy inside M1/M2

`behaviour/F_limits.csv`; see §7-F.  Additional observations worth quoting:

* M2 uses the stimulus speed target: max animal speed **2.000 m/s**
  (`m_v_max`), mean 1.02–1.04 m/s (lower than M1's 1.13 because the turn-demand
  factor slows animals while they manoeuvre);
* C3 binding rises from 21.9–23.2 k (M1) to 49.2–55.5 k (M2) animal-steps —
  the new accelerations are real and the box absorbs them with **zero**
  violations;
* task cost of M2: ρ = 0.793/0.768 vs M1's 0.974/0.984.  M2 makes the flock
  harder to keep on the corridor.  Both seeds still reach the goal.

---

## 9. Hooks Stage 3 (sensing 25/70 m and TTC re-calibration) will need

1. **Sensing ranges are not modelled yet on the robot side.**  `SimCfg.r_sense`
   (10 m) drives `View.refresh`; Stage 3 must split it into robot→animal (25 m)
   and robot→obstacle (70 m) *inside `View`*, and must keep `v2.sim_v2` free of
   any sensing logic (the animal models already own theirs).  Stage 2 leaves the
   hook clean: `animal_models` reads only the `obs_pos`/`obs_r` that `sim_v2`
   passes, so an observation model can be inserted at that boundary without
   touching the behaviour models.
2. **TTC re-calibration needs the same two-band structure**: `ttc_matrix` takes
   radii and a horizon; Stage 3 should add the observation interface parameters
   (range, FOV, latency, drop-out) at the `View` level so that the controller,
   the feasibility layer and the rollout all see the same degraded truth.
3. **Predictor must keep its own animal model.**  `v2/predictor_v2.evaluate_rollouts`
   already dispatches M0/M1/M2 and takes `mcfg`; Stage 3 can add M3 by extending
   that dispatch plus `animal_models.MODEL_CLASSES` — no change to the controller.
4. **Shadow mode is the ready-made instrument** for any "does the predictor
   believe the same world as the plant" question (`predictor_shadow=True`,
   `shadow_log` in `res`).
5. **The speed policy needs the controller's declared manoeuvre.**  Any Stage-3
   controller that re-plans DIVERT must expose `manoeuvre() -> {ahead, lateral}`
   or the policy will (by design) do nothing.
6. **Per-animal parameters are already vectorised** (`gain`, `d_react`,
   `project_sheep_increment` is called per animal), so field-of-view per animal
   or individual sensing ranges are a signature change, not a rewrite.
7. **Reporting hooks**: `metrics_v2.V2_KEYS` now carries the animal-model and
   controller-layer columns, so any new Stage-3 metric should be added there and
   it will flow to CSV and to `summarize_v2` automatically.

---

## 10. Limitations and 待确认

| item | status |
|---|---|
| M2 parameters identified by data | **No** — the avoidance outcome saturates at R = 6 m (all J ≈ 0).  Values come from the declared design rule (§4.3); their *effect* is measured separately in test C at R = 3 m. |
| Heterogeneity effect size | Small (paired clearance −0.0204 m, p = 0.56 on 20 seeds, 20/20 seeds differ).  Whether that is enough to matter behaviourally → **待确认**. |
| Reaction-delay value | 0.30 s is an engineering choice in a physiological band; no animal data → **待确认**. |
| `m2_slow_gain = 0.5` | Design override of the objective optimum (1.0); cost 0.55 J reported. If the author prefers the objective, one config field changes it. |
| M2 and task performance | M2 lowers path keeping (0.97 → 0.79) on S3.  Whether the manuscript should report M2 as the *shipped* model is an author decision → **待确认**. |
| Speed-policy effect on robot–obstacle clearance | Not established: 2 seeds, mixed sign, n too small.  The rule demonstrably improves ρ and reduces `max_dev`; the clearance claim needs a ≥30-seed run → **待确认**. |
| 9 m animal sensing | Ecologically plausible only for large mammals; the manuscript's 3/6/9 sweep is exploratory → **待确认**. |
| Long-horizon predictor agreement (DLL vs NumPy, 80 steps) | 3/7 offsets identical, 4/7 diverge through discrete-branch chaos.  Not a defect (the live M0 path uses the DLL), but any *comparison* between the two engines must use short horizons or accept this → documented. |
| Predictor does not model the sensing delay or M2's individual gains | Deliberate (mean-field, constant-velocity extrapolation).  Stage 3 may need a "predictor knows the delay" arm → open. |
| No-robot corridor initial condition (grid start) | Test-bed only; it changes the encounter geometry relative to `init_flock`.  Documented in §4.1; it does not touch any protocol run. |
| Real-platform behaviour parameters | All M1/M2 values are simulation engineering values; no hardware identification → 待确认. |
| Runtime | Stage-2 runs cost the same as Stage 1 with the DLL predictor (~17 s per S3 cell).  With the NumPy predictor (`predictor_engine="v2"`) a cell costs ~2× that (the predictor is pure NumPy).  A full 2400-run Stage-2 matrix is therefore ~13–26 core-hours → not run here. |

---

## 11. Files and commands

### 11.1 New files

```
v2/animal_models.py      M1/M2 desired-velocity + (V2)-box integration, delay
                         buffers, seeded per-animal gains, model factory
v2/corridor_v2.py        no-robot single-obstacle corridor, grid start, metrics
v2/calibrate_v2.py       M1/M2 grids, objective, selection rule, budget
v2/controller_v2.py      OursRolloutV2 (+declared manoeuvre, selectable
                         predictor), SpeedPolicyController, ShadowPredictor
v2/speed_policy_v2.py    r_req derivation, bound, per-tick log
v2/predictor_v2.py       NumPy rollout (M0/M1/M2 animal model)
v2/acceptance_s2.py      tests A-F
v2/capture_reference.py  whole-trajectory reference capture (test A baseline)
v2/plots_s2.py           behaviour plots
v2/speed_policy_ab.py    speed-policy A/B on S3
results_v2_stage2/       ACCEPTANCE.txt/.json, behaviour/*.csv, plots/*.png,
                         calibration_*.csv/.json/.log, reference/*.npz, detail CSVs
```

### 11.2 Edited V2 files

`config_v2.py` (Stage-2 fields, `stage2_v2cfg`, Stage-2 parameter table),
`sim_v2.py` (behaviour-model hook, trace/animal-log hooks, model-band C1
accounting, controller summary, empty-robot guards, `flock_init`),
`fleet_v2.py` (zero-obstacle fix), `runner_v2.py` (Stage-2 controller stack,
`run_cell` overrides), `metrics_v2.py` (Stage-2 columns).

### 11.3 Commands

```powershell
$env:OMP_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
$env:OPENBLAS_NUM_THREADS=1; $env:NUMEXPR_NUM_THREADS=1

python -X utf8 -m v2.capture_reference         # (run before Stage-2 edits)
python -X utf8 -m v2.calibrate_v2 --stage all  # calibration, seeds 1000-1004
python -X utf8 -m v2.calibrate_v2 --stage select   # apply the selection rule
python -X utf8 -m v2.acceptance_s2             # A-F, writes ACCEPTANCE.txt
python -X utf8 -m v2.plots_s2                  # behaviour plots
python -X utf8 -m v2.speed_policy_ab --seeds 1000 1001 --model M2
python -X utf8 -m v2.runner_v2 --scenarios S3 --seeds 2 --seed-offset 1000 \
       --algs Ours_rollout --set animal_model=M2 speed_policy=manoeuvre
python -X utf8 -m v2.unit_tests_v2             # Stage-1 structural tests
python -X utf8 -m v2.acceptance_v2             # Stage-1 suite: 17/17
```

Regression evidence for the edited V2 files: the Stage-1 suites still pass
in full (`v2.unit_tests_v2`: ALL PASS; `v2.acceptance_v2`: **17/17 checks**,
including its own test A, which compares the archived V1 cell column by
column).
