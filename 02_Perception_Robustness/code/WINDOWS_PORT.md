# WINDOWS_PORT.md — Windows port of the TTC-shepherding CEA experiments

Port root: `01_Main_Experiment/code\` (this directory).
Result lineages produced here are **new** (seeds 200-229); nothing on the Desktop,
no manuscript file and no figure was touched, and `results_win_smoke\` was left
exactly as it was.

---

## 1. Environment (verified in this session)

| item | value |
|---|---|
| OS | Windows 11 (10.0.26200) |
| CPU | 12th Gen Intel Core i7-12650H, 16 logical processors (6P + 4E) |
| Python | 3.13.7 (MSC v.1944 64-bit) |
| numpy | 2.5.2 |
| scipy | 1.17.1 |
| BLAS threads | **1** (`OMP_NUM_THREADS=MKL_NUM_THREADS=OPENBLAS_NUM_THREADS=1`) in every Python process |
| run-level parallelism | `--jobs N` (ProcessPoolExecutor, Windows *spawn*), nothing else |

The BLAS pinning does not touch the C++ kernel (it is plain serial C++); it keeps
the Python/numpy side bit-reproducible.

```powershell
$env:OMP_NUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"
$env:PYTHONUTF8="1"     # so redirected Chinese console text does not raise
```

---

## 2. Native rollout kernel — MSVC build

`rollout.cpp` is a **prediction model only** (it does not replace the evaluator):
a coupled finite-horizon rollout of the same Couzin interaction, slot geometry,
safety filter and unicycle equations, with a coarser prediction step.

Build command used (Developer Command Prompt for VS / `cl` on PATH):

```
cl /LD /O2 /EHsc /D_USE_MATH_DEFINES rollout.cpp /link /EXPORT:evaluate_rollouts
```

* `/LD` → DLL, `/O2` → optimise, `/EHsc` → standard C++ exceptions.
* `/D_USE_MATH_DEFINES` is **required on MSVC**: `rollout.cpp` uses `M_PI`, which
  `<cmath>` only defines when that macro is set. (On g++ the Linux build did not
  need it — this is the only build-flag difference.)
* `/EXPORT:evaluate_rollouts` exports the single `extern "C"` entry point
  `evaluate_rollouts(N,M,K,P,A, sheep_pos, sheep_vel, robot_pos, robot_head,
  robot_speed, obs_pos, obs_vel, obs_radius, path_pos, path_width, cfg, offsets,
  dt, steps, out)` with `out` shaped `(A,7)`.

Artifacts now in this directory:

| file | bytes | note |
|---|---|---|
| `_rollout.dll` | 173 568 | SHA-256 `865D87E69D820FC321173B200D27EDF48D88779990FD63F6B8E445897C180E19` |
| `rollout.obj` | 150 868 | MSVC object |
| `rollout.lib` / `rollout.exp` | 1 760 / 692 | import library + exports (side products of `/LD`) |
| `rollout.cpp` | 7 557 | unchanged source |

`_rollout.dll` must exist before any run; `proposed_rollout.py` and
`run_improvement.py` both abort with a clear message if it is missing.

---

## 3. The two Windows patches (kept exactly as found)

Both are minimal and touch **only** library loading / build plumbing. No control
law, no baseline, no scenario, no metric and no `simulation.py` line was changed.

1. **`proposed_rollout.py`** — platform-dependent native library and no compiler
   at import time:
   * `lib = root/('_rollout.dll' if sys.platform.startswith('win') else '_rollout.so')`
   * if the library is absent → `raise SystemExit('build _rollout.dll first')`
     (the Linux version shelled out to `g++` here; that step is gone)
   * `ctypes.CDLL(str(lib))` + `restype=None` on `evaluate_rollouts`.
2. **`run_improvement.py`** — when `rollout` is among the variants it only checks
   that the native library exists (`raise SystemExit('build _rollout.dll first
   (see BUILD_ROLLOUT_WINDOWS.md)')`) instead of invoking `g++`.

`run_revised_experiments.py` additionally instantiates `RolloutProposed()` once in
`__main__` before `main()`; under Windows *spawn* this guarantees the DLL is
loaded in the parent and each worker loads a complete copy of its own.

`test_corrections.py` + `test_rollout.py`: **8/8 tests pass** on this port
(`python -m unittest test_corrections test_rollout` → `Ran 8 tests ... OK`).

---

## 4. Determinism protocol

* one seed → one `np.random.default_rng(seed)` inside `run_scene`; no global RNG;
* `run_scene` deep-copies the scenario obstacles, so a scenario object reused by
  several methods is never polluted;
* a controller object is constructed per seed, so no state crosses runs;
* `--jobs` is **run-level only** (each task is a whole `(scenario, method)` cell),
  so parallelism cannot change a metric — verified: the same cell run through
  `run_improvement.py --jobs 1` (smoke) and `run_revised_experiments.py --jobs 4`
  gives bit-identical `C` and `d_min` (see §5);
* `ctrl_ms` **is** load-sensitive, so the paper's per-cycle timing comes only from
  the strict serial Stage C run (one process, `--jobs 1`), never from the matrix.

---

## 5. Verification

**(a) Smoke reproduction.** The recorded smoke values (S3, seed 100,
single-threaded BLAS) are reproduced exactly through the Stage A entry point
`run_revised_experiments.py`:

| method | recorded C | re-run C | recorded d_min | re-run d_min |
|---|---|---|---|---|
| `B2` | 0.1389 | 0.1389 | 0.0688 | 0.0688 |
| old `Ours` (`Proposed`) | 0.3889 | 0.3889 | 0.0216 | 0.0216 |
| `Ours_rollout` | 0.0833 | 0.0833 | 0.7139 | 0.7139 |

**(b) Cross-runner determinism.** S3 seed 200, `Ours_rollout`: C = 0.055556,
d_min = 0.884215 from `run_improvement.py --jobs 4` (pilot) and from
`run_revised_experiments.py --jobs 4` (matrix batch 1) — identical to 6 decimals.

**(c) Matrix integrity.** After the merge, `results_cea_matrix\integrity.json`
asserts 2400 rows = 8 methods x 10 scenarios x 30 seeds exactly 200-229, no
`Ours`/`Ours_w/o_PRE`, no duplicated cell.

---

## 6. Exact run commands

All from `01_Main_Experiment/code\` after the BLAS pinning above.

**Stage A — main matrix** (canonical single command; what the paper numbers are):

```bat
python run_revised_experiments.py --all --seeds 30 --seed-offset 200 ^
  --algs Ours_rollout B1 B2 VH-Path VH-Danger VH-Danger-path B-Outmost B-PCBF ^
  --jobs 4 --out results_cea_matrix
```

Actually executed as **8 method batches** (identical results, plus one wall-clock
checkpoint per batch and crash isolation) by

```powershell
.\run_cea_matrix_stage_a.ps1
```

which runs, for each method in the list above,

```bat
python run_revised_experiments.py --all --seeds 30 --seed-offset 200 ^
  --algs <one method> --jobs 4 --out results_cea_matrix\_parts\<method>
```

and then

```bat
python merge_cea_matrix.py --root results_cea_matrix
```

**Stage B — rollout ablation** (S2R2 + S3, seeds 200-229):

```bat
python run_cea_ablation.py --out results_cea_ablation --jobs 4 ^
  --scenarios S2R2 S3 --seed-start 200 --seed-count 30
python analyze_cea_ablation.py --root results_cea_ablation
```

**Stage C — strict serial timing** (one process, one method at a time, no
parallelism; S3, seeds 100-102 = the smoke scenario/seed block, and S3 seeds
200-202 = the matrix lineage):

```bat
python run_revised_experiments.py --scenarios S3 --seeds 3 --seed-offset 100 ^
  --algs Ours_rollout B1 B2 VH-Path VH-Danger VH-Danger-path B-Outmost B-PCBF ^
  --jobs 1 --out results_cea_timing_seeds100
python run_revised_experiments.py --scenarios S3 --seeds 3 --seed-offset 200 ^
  --algs Ours_rollout B1 B2 VH-Path VH-Danger VH-Danger-path B-Outmost B-PCBF ^
  --jobs 1 --out results_cea_timing_seeds200
```

**Stage D — statistics for the new family** (1 Ours + 7 baselines, Holm within
this family only):

```bat
python stats_cea_family.py --root results_cea_matrix
python cea_report_tables.py --root results_cea_matrix ^
  --ablation results_cea_ablation ^
  --timing results_cea_timing_seeds100 results_cea_timing_seeds200
```

**Ablation guard test:**

```bat
python -m unittest test_rollout_ablation
```

---

## 7. Wall clock

<!-- FILL:WALL -->

---

## 8. Deviation from the Linux-validated quoted numbers

The author quoted (30 seeds, Linux/WSL, same seed block): S3 `C_B2 = 21.76 %` →
`C_Ours = 12.31 %`, S2R2 `46.67 %` → `17.22 %`.

<!-- FILL:DEVIATION -->

---

## 9. 待确认

<!-- FILL:OPEN -->

---

## 10. Files this port added (all left in place)

| file | role |
|---|---|
| `run_cea_matrix_stage_a.ps1` | Stage A batch driver (8 method batches + wall clock log) |
| `merge_cea_matrix.py` | merges batches → detail/summary/summary_core/main_table/events + integrity |
| `proposed_rollout_ablation.py` | Stage B rung "rollout on the bare four-state FSM" (mechanical composition) |
| `test_rollout_ablation.py` | guard: that rung may differ from `RolloutProposed.step` in exactly one call |
| `run_cea_ablation.py` | Stage B runner (6 rungs x 2 scenarios x 30 seeds) |
| `analyze_cea_ablation.py` | Stage B paired rung-vs-full statistics |
| `stats_cea_family.py` | Stage D Wilcoxon / exact McNemar / Holm for the 8-method family |
| `cea_report_tables.py` | renders the markdown tables in `results_cea_matrix\analysis\` |
| `WINDOWS_PORT.md` | this document |
| `CEA_MATRIX_REPORT.md` | the report (main table, ablation, timing, statistics, deviation) |
