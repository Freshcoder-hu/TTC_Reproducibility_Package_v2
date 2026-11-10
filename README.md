# TTC shepherding — reproducibility package (v2)

Code and archived results for the submitted article **“Animal contact-risk
prediction and action-conditioned rollout for multi-robot shepherding”**
(*Computers and Electronics in Agriculture*). The article has 9 figures,
12 tables, 8 equations, 1 algorithm and 40 references; the manuscript itself
is submitted separately and is **not** redistributed here.

This is a **simulation and software-measurement** artifact. It contains no
real-flock or physical-robot experiment, and nothing here demonstrates zero
animal–robot contact. Residual contacts are reported, not hidden.

The package is ready to push to GitHub as-is: `.gitignore` and
`.gitattributes` are included, no file exceeds 100 MiB, and the largest file
is 13.3 MiB. One slot is still open — the repository URL; see
[§10](#10-repository-url--one-open-slot).

---

## 1. What is in this repository

| Path | What it is |
|---|---|
| `00_Documentation/` | detailed README, paper version, data notes, figure map, provenance, this package's URL slot |
| `01_Main_Experiment/` | V2 main experiment: 2,400-run matrix, 240-run ablation, stage 1–5 reports, figures/tables build scripts |
| `02_Perception_Robustness/` | the authoritative 2,340-cell robustness corpus (`results/formal.jsonl`) and its analysis |
| `03_Online_Warning_and_Timing/` | online warning / latency measurement (132 event replays + 24 timing runs) |
| `04_Quick_Access/` | convenience copies of key tables and reports; `Submitted_Figures/` holds Figures 1–9 as `.pdf` + `.png` |
| `05_Curved_Path_240/` | confirmatory three-turn panel, 240 runs → Fig. 8 / Table 11 |
| `06_Robot_Loss_1080/` | robot-loss panel, 1,080 runs → Fig. 9 / Table 12 |
| `tools/` | spot-check, standardiser, hygiene gate, online-input stager, number-tolerance verifier |
| `LICENSE` / `LICENSE-DATA` / `LICENSING.md` | MIT for code; CC BY 4.0 for data, results, tables, figures and docs |
| `CITATION.cff` | citation metadata (repository URL marked `TODO(URL)`) |
| `verify_package.py` | one-command checksum verification |
| `PACKAGE_SHA256_MANIFEST.json`, `PACKAGE_FILE_INDEX.csv`, `SHA256SUMS.txt` | integrity manifests |

The **detailed** README — every command, expected output, runtime, and the
limits of each evidence layer — is
[`00_Documentation/README_EN.md`](00_Documentation/README_EN.md). A Chinese
guide is at [`00_Documentation/REPRODUCE_GUIDE.md`](00_Documentation/REPRODUCE_GUIDE.md).

## 2. Environment

| Item | Value used for the verification runs |
|---|---|
| OS | Windows 11 x64 (Linux/macOS should work; only the native `.dll` is platform-specific) |
| Python | **3.13.7** (original simulations: 3.12) |
| numpy | **2.5.2** (original: 2.3.5) |
| scipy | **1.17.1** (original: 1.17.0) |
| pandas | 2.3.3 — only some analysis scripts |
| matplotlib | 3.10.8 — only `paper/scripts/build_figures.py` |

**Pin BLAS to one thread before running anything.** Several reported numbers
are sums over runs; multi-threaded BLAS changes float accumulation order.

```bash
# Linux / macOS
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
```

```powershell
# PowerShell
$env:OMP_NUM_THREADS=1; $env:OPENBLAS_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
```

The Windows rolling kernel `_rollout.dll` is shipped. On other platforms
rebuild it from the provided source (C++17):

```bash
g++ -O3 -std=c++17 -shared -fPIC rollout.cpp -o _rollout.so
g++ -O3 -ffp-contract=off -std=c++17 -shared -fPIC rs_safety.cpp -o _rs_safety.so
```

## 3. Quick start

Run from the package root (the folder containing this README):

```bash
python tools/spotcheck.py
python 05_Curved_Path_240/reproduce_table11.py
python 06_Robot_Loss_1080/reproduce_table12.py
python 01_Main_Experiment/code/reproduce_stage5.py --check-report
python verify_package.py
```

Each command prints `PASS` and exits 0. Together they take a few seconds and
recompute the headline numbers, both panel tables, and every checksum.

## 4. Tested entry points

`R` = wall-clock runtime measured on this tree. All commands are run from the
package root unless stated otherwise.

| # | Command | Expected output | R |
|---|---|---|---|
| 1 | `python tools/spotcheck.py` | a chain of `[PASS]` lines, ending `RESULT: PASS - all graphical-abstract numbers and headline counts reproduce from the shipped records` | 0.4 s |
| 2 | `python tools/standardise_formal.py --verify` | `ROUND-TRIP: PASS - every cell re-read matches …` | 0.7 s |
| 3 | `python tools/check_hygiene.py` | per-extension counts, then `HYGIENE: PASS` (repo mode is the default; `--package-mode` restores the strict dotfile check) | 0.4 s |
| 4 | `python 01_Main_Experiment/code/reproduce_stage5.py` | `wrote 12 tables + report_numbers.json to …/results_v2_stage5/reproduced` | 1.6 s |
| 5 | `python 01_Main_Experiment/code/reproduce_stage5.py --check-report` | `CHECK-REPORT: PASS (the document is byte-identical to the regenerated one)` | 1.8 s |
| 6 | `python tools/verify_numbers.py` | after snapshotting `reproduced/`: `RESULT: PASS - 13/13 files … 0 error(s)` (see `00_Documentation/README_EN.md`, *Two kinds of verification*) | 0.1 s |
| 7 | `python 05_Curved_Path_240/reproduce_table11.py` | Table 11 rows, then `RESULT: PASS - Table 11 / Fig. 8 key numbers reproduce …` | 0.1 s |
| 8 | `python 06_Robot_Loss_1080/reproduce_table12.py` | `V2S3: 1.67 % -> 6.94 %`, `1080/1080`, then `RESULT: PASS - …` | 0.2 s |
| 9 | `python 01_Main_Experiment/paper/scripts/build_data.py` | ends `censored total: 377` / `ablation 300 rows; formal 2400 rows` | 59 s |
| 10 | `python 01_Main_Experiment/paper/scripts/build_figures.py` | 9 lines `figure fig01_observability` … `fig09_radar` | 12 s |
| 11 | `python tools/stage_online_inputs.py` | `staged hard link …` plus the corpus SHA-256 | 0.2 s |
| 12 | `python 03_Online_Warning_and_Timing/analysis/analyze_online_evidence.py` | a JSON summary on stdout, exit 0 | 5.9 s |
| 13 | `python tools/stage_online_inputs.py --clean` | `removed …/results/formal.jsonl` | 0.1 s |
| 14 | `python verify_package.py` | `Verified 1032 files; errors = 0` | ~6 s |

Notes:

* Commands 4–5 and 9–10 write into `results_v2_stage5/reproduced/` and
  `paper/figures/`; within one environment they are idempotent (a second run is
  byte-identical). Across environments compare numbers, not bytes, with command
  6 (see *Two kinds of verification* in the detailed README).
* `paper/figures/` is not shipped. The submitted Figures 1–9 are in
  `04_Quick_Access/Submitted_Figures/`; `build_figures.py` regenerates the
  auxiliary V7-era re-plots, whose `fig07`–`fig09` are **not** the submitted
  Figs. 7–9.
* Commands 11–13 temporarily stage the shared robustness corpus as a hard
  link, then remove it; after `--clean` the tree is byte-identical to the
  delivered one.

## 5. Seed blocks

| Block | Seeds | Used by | Where |
|---|---|---|---|
| Main evaluation | **2000–2029** | 8 methods × 10 scenes × 30 = **2,400** runs; ablation R0–R3; robot-loss panel reuses the block as paired seeds | `results_v2_stage5/matrix/*.csv`, `ablation/detail.csv`, `06_Robot_Loss_1080/` |
| Corrected protocol | **3000–3029** | perception robustness: 13 conditions × 3 methods × 2 scenes = 78 → **2,340** cells | `02_Perception_Robustness/results/formal.jsonl` |
| Curved corridor | **5000–5029** | confirmatory three-turn panel, 4 controller rows × 2 scenes × 30 = **240** | `05_Curved_Path_240/detail.csv` |
| Calibration | **1000–1019** | M1/M2 calibration and C1 / T_REC sweeps | partly — the shipped sweeps use 1000–1004 and 1000–1002 |
| Diagnostics | **1005–1009**, **3000** | geometry/spacing diagnostics; serial timing traces | partly — timing traces are `…__3000.npz` |

Seeds are never shared between blocks, and no denominator is carried from one
layer to another. The robot-loss panel deliberately replays seeds 2000–2029
as its frozen control, but it is a **separate experiment**, not 1,080 more
main runs. The six denominators (2,400 / 2,340 / 132 + 24 / 240 / 1,080) must
not be added together; see
[`00_Documentation/README_EN.md` §6](00_Documentation/README_EN.md#6-denominators-and-what-may-not-be-merged).

## 6. Figure and table → data map

### Figures

| Fig. | Topic | Numeric source / producer |
|---|---|---|
| 1 | Observable warning time | `paper/data/numbers.json` ← `build_data.py`; probe `code/v2/observability_v2.py` |
| 2 | The coupling (schematic) | definitions in `code/v2/controller_v2.py` |
| 3 | Scenario geometry | `code/v2/scenarios_v2.py` |
| 4 | Nine-scenario comparison | `results_v2_stage5/matrix/*.csv` (2,400 runs) → `build_data.py` → `build_figures.py` |
| 5 | Contact fraction by scene × method | same matrix → `paper/tables/table_per_scenario.csv` |
| 6 | Ablation ladder R0→R3 | `results_v2_stage5/ablation/detail.csv` (240 runs) |
| 7 | Perception-response profile | `02_Perception_Robustness/results/formal.jsonl` → `analysis/summary.csv` |
| 8 | Three-turn panel | `05_Curved_Path_240/detail.csv` + `traces/*.npz` → `reproduce_table11.py` |
| 9 | Robot-loss panel | `06_Robot_Loss_1080/detail.csv` → `reproduce_table12.py` |

### Tables

| Table | Topic | Where the numbers come from |
|---|---|---|
| 1–3 | Cost terms, frozen configuration, scenes | `code/v2/{controller_v2,config_v2,scenarios_v2}.py` |
| 4 | The eight methods | `results_v2_stage5/matrix/` (one CSV per method) |
| 5 | Evidence layers / denominators | §5 above and README_EN §6 |
| 6 | Corrected protocol (3000–3029) | `formal.jsonl`, `version=safe` ∧ `factor=exact` |
| 7–8 | Archived comparison + paired contrasts | `paper/tables/table_main9.csv`, `table_contrasts*.csv` |
| 9–10 | 11-condition contact fraction, decomposition | `02_Perception_Robustness/analysis/{summary,decomposition}.csv` |
| 11 | Three-turn panel | `05_Curved_Path_240/table_three_turn.csv` ← `detail.csv` |
| 12 | Robot-loss panel | `06_Robot_Loss_1080/table_failure.csv` ← `detail.csv` |

One row per figure/table, with the exact producer script, is in
[`00_Documentation/FIGURES_TO_EVIDENCE_MAP.md`](00_Documentation/FIGURES_TO_EVIDENCE_MAP.md).

## 7. What is not runnable end-to-end here

These files are part of the record, not working entry points in this layout:

| Entry | Why |
|---|---|
| `python -m v2.acceptance_v2` | needs the V1 comparison archive `results_cea_matrix/_parts/`, which is not part of this package |
| `python -m v2.acceptance_s2` | reaches `acceptance_v2.archived_row`, i.e. the same missing archive |
| `python -m v2.acceptance_s3`, `python -m v2.acceptance_s4` | their stage archives are present, but the inertness tests still need the V1 archive |
| `measure_online_evidence.py` | re-runs the 132-event measurement; Linux-oriented and expensive by design |
| `check_timing_overhead.py` | calls `os.sched_setaffinity`, which does not exist on Windows |
| full 2,400-run matrix (`v2/runner_v2`, `v2/stage5.matrix`) | archived CSVs are shipped, so the rerun is optional; it needs 12 jobs and hours |
| re-running the 240-run or 1,080-run panels | per-run records are shipped, so the recomputations suffice; re-simulating needs the author's drivers and a byte-identical native kernel |

**About the Stage-4 indentation fix.** `01_Main_Experiment/code/v2/acceptance_s4.py`
previously had a two-line mis-indentation inside `test_A` that raised
`IndentationError: unexpected indent`; it is now repaired and the file
compiles. That repair removes the static syntax error only. **It does not
make the Stage-4 suite end-to-end runnable**: the suite still needs the V1
archive that is not shipped. All 376 `.py` files in the package compile with
`python -m compileall -q` (0 failures), which is a syntax result, not a
run result.

## 8. Integrity

```bash
python verify_package.py          # reads PACKAGE_SHA256_MANIFEST.json
```

or with standard tools:

```bash
sha256sum -c SHA256SUMS.txt       # Linux / macOS
```

`PACKAGE_FILE_INDEX.csv` gives path, category, byte count and SHA-256 for
every entry. Verify **before** running commands 4–5 or 8–9 if you want to
compare against the shipped bytes, because those commands rewrite generated
outputs (idempotently).

## 9. Licence

* **Code** — all `.py`, `.cpp`, `.cu`, headers, build files and scripts —
  is under the **MIT License**: [`LICENSE`](LICENSE).
* **Data and results** — all `.csv`, `.jsonl`, `.json`, `.npz`, tables,
  figures and documentation — is under **CC BY 4.0**: [`LICENSE-DATA`](LICENSE-DATA).

The path-by-path mapping is in [`LICENSING.md`](LICENSING.md). Copyright
(c) 2026 Shuaihu Wang, Gaowei Zhang, Zehao Xu, Yun Huang.

## 10. Repository URL — one open slot

The manuscript must cite this repository instead of saying the materials are
“provided as supplementary material”. The final URL and archival DOI are
**not yet known**, so they are deliberately left as `<URL>` and
`<DOI-or-URL>` here and in
[`00_Documentation/REPOSITORY_URL.md`](00_Documentation/REPOSITORY_URL.md),
which contains the exact replacement sentence, the submission-system text,
and a `[dataset]` BibTeX skeleton. `CITATION.cff` carries the same marker as
`repository-code: "TODO(URL)"`.

Do not publish the repository until those placeholders are replaced.

## 11. Documentation map

| File | Why open it |
|---|---|
| [`00_Documentation/README_EN.md`](00_Documentation/README_EN.md) | the detailed English README: every command, output, runtime, denominators, limits |
| [`00_Documentation/REPRODUCE_GUIDE.md`](00_Documentation/REPRODUCE_GUIDE.md) | the same in Chinese |
| [`00_Documentation/PAPER_VERSION.md`](00_Documentation/PAPER_VERSION.md) | which paper version this reproduces, and what was removed |
| [`00_Documentation/FIGURES_TO_EVIDENCE_MAP.md`](00_Documentation/FIGURES_TO_EVIDENCE_MAP.md) | figure/table → producing file, one row each |
| [`00_Documentation/DATA_NOTES.md`](00_Documentation/DATA_NOTES.md) | `NaN`/`Infinity` in `formal.jsonl`, and the standardised CSV |
| [`00_Documentation/STATISTICAL_SCOPE_AND_LIMITATIONS.md`](00_Documentation/STATISTICAL_SCOPE_AND_LIMITATIONS.md) | what the statistics do and do not support |
| [`00_Documentation/SOURCE_PROVENANCE.md`](00_Documentation/SOURCE_PROVENANCE.md) | SHA-256 of every source archive |
| [`00_Documentation/REPOSITORY_URL.md`](00_Documentation/REPOSITORY_URL.md) | the two places where the final URL must go |
