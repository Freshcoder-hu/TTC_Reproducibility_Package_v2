# TTC shepherding 閳?reproducibility package (v2)

Code and archived results for the submitted article **閳ユ穾nimal contact-risk
prediction and action-conditioned rollout for multi-robot shepherding閳?*
(*Computers and Electronics in Agriculture*). The article has 9 figures,
12 tables, 8 equations, 1 algorithm and 41 references; the manuscript itself
is submitted separately and is **not** redistributed here. The nine submitted
figures can be **regenerated from this tree** (`07_Figure_Source/`), not only
inspected.

This is a **simulation and software-measurement** artifact. It contains no
real-flock or physical-robot experiment, and nothing here demonstrates zero
animal閳ユ悜obot contact. Residual contacts are reported, not hidden.

The package is ready to push to GitHub as-is: `.gitignore` and
`.gitattributes` are included, no file exceeds 100 MiB, and the largest file
is 13.3 MiB. The canonical repository is
**https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2**; see
[鎼?0](#10-repository-url).

> **The repository must be re-pushed in full.** An older copy of it is still
> live on GitHub and carries the **pre-fix** files: `CITATION.cff` with
> a URL placeholder in `repository-code`, `.gitattributes` with `* text=auto`,
> README/PAPER_VERSION statements of 40 references, and no
> `07_Figure_Source/`. The tree delivered here is the corrected one; push it as
> a whole (not as a few edited files) so that no stale file survives.

---

## 1. What is in this repository

| Path | What it is |
|---|---|
| `00_Documentation/` | detailed README, paper version, data notes, figure map, provenance, this package's URL slot |
| `01_Main_Experiment/` | V2 main experiment: 2,400-run matrix, 240-run ablation, stage 1閳? reports, figures/tables build scripts |
| `02_Perception_Robustness/` | the authoritative 2,340-cell robustness corpus (`results/formal.jsonl`) and its analysis |
| `03_Online_Warning_and_Timing/` | online warning / latency measurement (132 event replays + 24 timing runs) |
| `04_Quick_Access/` | convenience copies of key tables and reports; `Submitted_Figures/` holds Figures 1閳? as `.pdf` + `.png` |
| `05_Curved_Path_240/` | confirmatory three-turn panel, 240 runs 閳?Fig. 8 / Table 11 |
| `06_Robot_Loss_1080/` | robot-loss panel, 1,080 runs 閳?Fig. 9 / Table 12 |
| `07_Figure_Source/` | the plotting pipeline: `fig_body.py`, `fig_loss.py`, `fig_si.py`, `make_graphical_abstract.py`, `v9style.py`, `pkgpaths.py`, `compare_figures.py` 閳?regenerates the submitted Figures 1閳? |
| `tools/` | spot-check, standardiser, hygiene gate, online-input stager, number-tolerance verifier |
| `LICENSE` / `LICENSE-DATA` / `LICENSING.md` | MIT for code; CC BY 4.0 for data, results, tables, figures and docs |
| `CITATION.cff` | citation metadata (canonical repository URL recorded in `repository-code`) |
| `verify_package.py` | one-command checksum verification |
| `PACKAGE_SHA256_MANIFEST.json`, `PACKAGE_FILE_INDEX.csv`, `SHA256SUMS.txt` | integrity manifests |

The **detailed** README 閳?every command, expected output, runtime, and the
limits of each evidence layer 閳?is
[`00_Documentation/README_EN.md`](00_Documentation/README_EN.md). A Chinese
guide is at [`00_Documentation/REPRODUCE_GUIDE.md`](00_Documentation/REPRODUCE_GUIDE.md).

## 2. Environment

| Item | Value used for the verification runs |
|---|---|
| OS | Windows 11 x64 (Linux/macOS should work; only the native `.dll` is platform-specific) |
| Python | **3.13.7** (original simulations: 3.12) |
| numpy | **2.5.2** (original: 2.3.5) |
| scipy | **1.17.1** (original: 1.17.0) |
| pandas | 2.3.3 閳?only some analysis scripts |
| matplotlib | 3.10.8 閳?the `07_Figure_Source/` pipeline and `paper/scripts/build_figures.py` (the submitted Figures 1閳? come back pixel-identical with this version) |

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
| 2 | `python tools/standardise_formal.py --verify` | `ROUND-TRIP: PASS - every cell re-read matches 閳ヮ泦 | 0.7 s |
| 3 | `python tools/check_hygiene.py` | per-extension counts, then `HYGIENE: PASS` (repo mode is the default; `--package-mode` restores the strict dotfile check) | 0.4 s |
| 4 | `python 01_Main_Experiment/code/reproduce_stage5.py` | `wrote 12 tables + report_numbers.json to 閳?results_v2_stage5/reproduced` | 1.6 s |
| 5 | `python 01_Main_Experiment/code/reproduce_stage5.py --check-report` | `CHECK-REPORT: PASS (the document is byte-identical to the regenerated one)` | 1.8 s |
| 6 | `python tools/verify_numbers.py` | after snapshotting `reproduced/`: `RESULT: PASS - 13/13 files 閳?0 error(s)` (see `00_Documentation/README_EN.md`, *Two kinds of verification*) | 0.1 s |
| 7 | `python 05_Curved_Path_240/reproduce_table11.py` | Table 11 rows, then `RESULT: PASS - Table 11 / Fig. 8 key numbers reproduce 閳ヮ泦 | 0.1 s |
| 8 | `python 06_Robot_Loss_1080/reproduce_table12.py` | `V2S3: 1.67 % -> 6.94 %`, `1080/1080`, then `RESULT: PASS - 閳ヮ泦 | 0.2 s |
| 9 | `python 01_Main_Experiment/paper/scripts/build_data.py` | ends `censored total: 377` / `ablation 300 rows; formal 2400 rows` | 59 s |
| 10 | `python 01_Main_Experiment/paper/scripts/build_figures.py` | 9 lines `figure fig01_observability` 閳?`fig09_radar` | 12 s |
| 11 | `python tools/stage_online_inputs.py` | `staged hard link 閳ヮ泦 plus the corpus SHA-256 | 0.2 s |
| 12 | `python 03_Online_Warning_and_Timing/analysis/analyze_online_evidence.py` | a JSON summary on stdout, exit 0 | 5.9 s |
| 13 | `python tools/stage_online_inputs.py --clean` | `removed 閳?results/formal.jsonl` | 0.1 s |
| 14 | `python verify_package.py` | `Verified 1040 files; errors = 0` | ~6 s |
| 15 | `python 07_Figure_Source/fig_body.py` | 8 lines `Fig1 閳?Fig8  183.60 mm  texts=閳? min 7.5 pt`, then `wrote 8 figure(s) to 閳ヮ泦 | 5.2 s |
| 16 | `python 07_Figure_Source/fig_loss.py` | `Fig9  183.60 mm  texts=60  min 7.5 pt` | 1.6 s |
| 17 | `python 07_Figure_Source/fig_si.py` | `FigS1 閳?FigS4`, then `wrote 4 figure(s) to 閳ヮ泦 | 2.6 s |
| 18 | `python 07_Figure_Source/make_graphical_abstract.py` | `canvas 130.00 x 66.00 mm` 閳?`written 閳?graphical_abstract.pdf` | 1.4 s |
| 19 | `python 07_Figure_Source/compare_figures.py` | the graded table: `RESULT: BYTE-IDENTICAL=1, PIXEL-IDENTICAL=9, VISUALLY-EQUIVALENT=4` | 12 s |
| 20 | `python 07_Figure_Source/compare_figures.py --no-run --si-ref DIR --ga-ref FILE` | same table without re-running the scripts (needs the submission package's SI figures) | 3 s |

Notes:

* Commands 4閳?, 9閳?0 and 15閳?9 write into `results_v2_stage5/reproduced/`,
  `paper/figures/` and `07_Figure_Source/build/`; within one environment they
  are idempotent (a second run is byte-identical). Across environments compare
  numbers, not bytes, with command 6 (see *Two kinds of verification* in the
  detailed README). `07_Figure_Source/build/` is covered by `.gitignore`.
* `paper/figures/` is not shipped. The submitted Figures 1閳? are in
  `04_Quick_Access/Submitted_Figures/` and are **regenerated** by
  `07_Figure_Source/fig_body.py` (Figs. 1閳?) and `fig_loss.py` (Fig. 9): the
  600 dpi `.png` comes out byte-identical and the `.pdf` pixel-identical (its
  bytes differ only in `/CreationDate`). `build_figures.py` is the *older*
  V7-era re-plot script, whose `fig07`閳ユ彵fig09` are **not** the submitted
  Figs. 7閳?.
* Commands 11閳?3 temporarily stage the shared robustness corpus as a hard
  link, then remove it; after `--clean` the tree is byte-identical to the
  delivered one.

## 5. Seed blocks

| Block | Seeds | Used by | Where |
|---|---|---|---|
| Main evaluation | **2000閳?029** | 8 methods 鑴?10 scenes 鑴?30 = **2,400** runs; ablation R0閳ユ彍3; robot-loss panel reuses the block as paired seeds | `results_v2_stage5/matrix/*.csv`, `ablation/detail.csv`, `06_Robot_Loss_1080/` |
| Corrected protocol | **3000閳?029** | perception robustness: 13 conditions 鑴?3 methods 鑴?2 scenes = 78 閳?**2,340** cells | `02_Perception_Robustness/results/formal.jsonl` |
| Curved corridor | **5000閳?029** | confirmatory three-turn panel, 4 controller rows 鑴?2 scenes 鑴?30 = **240** | `05_Curved_Path_240/detail.csv` |
| Calibration | **1000閳?019** | M1/M2 calibration and C1 / T_REC sweeps | partly 閳?the shipped sweeps use 1000閳?004 and 1000閳?002 |
| Diagnostics | **1005閳?009**, **3000** | geometry/spacing diagnostics; serial timing traces | partly 閳?timing traces are `閳ヮ泤_3000.npz` |

Seeds are never shared between blocks, and no denominator is carried from one
layer to another. The robot-loss panel deliberately replays seeds 2000閳?029
as its frozen control, but it is a **separate experiment**, not 1,080 more
main runs. The six denominators (2,400 / 2,340 / 132 + 24 / 240 / 1,080) must
not be added together; see
[`00_Documentation/README_EN.md` 鎼?](00_Documentation/README_EN.md#6-denominators-and-what-may-not-be-merged).

## 6. Figure and table 閳?data map

### Figures

| Fig. | Topic | Numeric source / producer |
|---|---|---|
| 1 | Observable warning time | `paper/data/numbers.json` 閳?`build_data.py`; probe `code/v2/observability_v2.py`; drawn by `07_Figure_Source/fig_body.py` (`Fig1`) |
| 2 | The coupling (schematic) | definitions in `code/v2/controller_v2.py`; drawn by `07_Figure_Source/fig_body.py` (`Fig2`) |
| 3 | Scenario geometry | `code/v2/scenarios_v2.py`; drawn by `07_Figure_Source/fig_body.py` (`Fig3`) |
| 4 | Nine-scenario comparison | `results_v2_stage5/matrix/*.csv` (2,400 runs) 閳?`build_data.py`; drawn by `07_Figure_Source/fig_body.py` (`Fig4`) |
| 5 | Contact fraction by scene 鑴?method | same matrix 閳?`paper/tables/table_per_scenario.csv`; drawn by `07_Figure_Source/fig_body.py` (`Fig5`) |
| 6 | Ablation ladder R0閳壊3 | `results_v2_stage5/ablation/detail.csv` (240 runs); drawn by `07_Figure_Source/fig_body.py` (`Fig6`) |
| 7 | Perception-response profile | `02_Perception_Robustness/results/formal.jsonl` 閳?`analysis/summary.csv`; drawn by `07_Figure_Source/fig_body.py` (`Fig7`) |
| 8 | Three-turn panel | `05_Curved_Path_240/detail.csv` + `traces/*.npz` 閳?`reproduce_table11.py`; drawn by `07_Figure_Source/fig_body.py` (`Fig8`) |
| 9 | Robot-loss panel | `06_Robot_Loss_1080/detail.csv` 閳?`reproduce_table12.py`; drawn by `07_Figure_Source/fig_loss.py` (`Fig9`) |

### Tables

| Table | Topic | Where the numbers come from |
|---|---|---|
| 1閳? | Cost terms, frozen configuration, scenes | `code/v2/{controller_v2,config_v2,scenarios_v2}.py` |
| 4 | The eight methods | `results_v2_stage5/matrix/` (one CSV per method) |
| 5 | Evidence layers / denominators | 鎼? above and README_EN 鎼? |
| 6 | Corrected protocol (3000閳?029) | `formal.jsonl`, `version=safe` 閳?`factor=exact` |
| 7閳? | Archived comparison + paired contrasts | `paper/tables/table_main9.csv`, `table_contrasts*.csv` |
| 9閳?0 | 11-condition contact fraction, decomposition | `02_Perception_Robustness/analysis/{summary,decomposition}.csv` |
| 11 | Three-turn panel | `05_Curved_Path_240/table_three_turn.csv` 閳?`detail.csv` |
| 12 | Robot-loss panel | `06_Robot_Loss_1080/table_failure.csv` 閳?`detail.csv` |

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
| `python 07_Figure_Source/fig_body.py --only 9` | needs the 100 s-scale panel (`04_閺佺増宓?閻у墽顫楃憴鍕乢閺堫亪娈㈤崠鍛絺鐢?`), which neither this package nor the submission package ships. It is the **earlier Word manuscript's** Fig. 9, not submitted Fig. 9; the submitted one is `07_Figure_Source/fig_loss.py` |
| `07_Figure_Source/fig_si.py` reproducing S1閳ユ彎4 **byte for byte** | the shipped S1閳ユ彎4 were drawn on the earlier 190 mm canvas, before the design width was frozen at 183.6 mm. The script reproduces their content at 183.6 mm (identical text, identical layout, 3.49 % narrower), not the 190 mm files |

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
every entry. Verify **before** running commands 4閳? or 8閳? if you want to
compare against the shipped bytes, because those commands rewrite generated
outputs (idempotently).

## 9. Licence

* **Code** 閳?all `.py`, `.cpp`, `.cu`, headers, build files and scripts 閳?  is under the **MIT License**: [`LICENSE`](LICENSE).
* **Data and results** 閳?all `.csv`, `.jsonl`, `.json`, `.npz`, tables,
  figures and documentation 閳?is under **CC BY 4.0**: [`LICENSE-DATA`](LICENSE-DATA).

The path-by-path mapping is in [`LICENSING.md`](LICENSING.md). Copyright
(c) 2026 Shuaihu Wang, Gaowei Zhang, Zehao Xu, Yun Huang.

## 10. Repository URL

The canonical public repository is
**https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2**. The
manuscript's data-availability sentence points to it and cites the
`[dataset]` entry added to `refs.bib` (the reference list is now **41
entries**: 40 journal articles plus this dataset). `CITATION.cff` records the
same URL in `repository-code`, and
[`00_Documentation/REPOSITORY_URL.md`](00_Documentation/REPOSITORY_URL.md)
records both filled slots with the date. No archival DOI has been assigned, so
the repository URL is the identifier.

## 11. Documentation map

| File | Why open it |
|---|---|
| [`00_Documentation/README_EN.md`](00_Documentation/README_EN.md) | the detailed English README: every command, output, runtime, denominators, limits |
| [`00_Documentation/REPRODUCE_GUIDE.md`](00_Documentation/REPRODUCE_GUIDE.md) | the same in Chinese |
| [`00_Documentation/PAPER_VERSION.md`](00_Documentation/PAPER_VERSION.md) | which paper version this reproduces, and what was removed |
| [`00_Documentation/FIGURES_TO_EVIDENCE_MAP.md`](00_Documentation/FIGURES_TO_EVIDENCE_MAP.md) | figure/table 閳?producing file, one row each |
| [`00_Documentation/DATA_NOTES.md`](00_Documentation/DATA_NOTES.md) | `NaN`/`Infinity` in `formal.jsonl`, and the standardised CSV |
| [`00_Documentation/STATISTICAL_SCOPE_AND_LIMITATIONS.md`](00_Documentation/STATISTICAL_SCOPE_AND_LIMITATIONS.md) | what the statistics do and do not support |
| [`00_Documentation/SOURCE_PROVENANCE.md`](00_Documentation/SOURCE_PROVENANCE.md) | SHA-256 of every source archive |
| [`00_Documentation/REPOSITORY_URL.md`](00_Documentation/REPOSITORY_URL.md) | the canonical repository URL, both slots filled, and the dataset citation |
| [`07_Figure_Source/README.md`](07_Figure_Source/README.md) | the plotting pipeline: input map, how to run, per-figure verdict, remaining limits |

## 12. Data availability, as stated in the manuscript

The submitted article's data-availability sentence points to the canonical
repository and says that **the repository documentation states which tables and
figures can be regenerated from the archived data and which cannot**. That
documentation is this README, `00_Documentation/PAPER_VERSION.md`,
`00_Documentation/FIGURES_TO_EVIDENCE_MAP.md` and
[`07_Figure_Source/README.md`](07_Figure_Source/README.md). In short:

* **Figures 1閳? 閳?regenerated.** `07_Figure_Source/` rebuilds all nine from the
  shipped data; the `.png` is byte-identical and the `.pdf` pixel-identical.
* **Tables 1閳?2 閳?the numbers behind every one of them are shipped and traced**
  to a specific file in `00_Documentation/FIGURES_TO_EVIDENCE_MAP.md`, and the
  reported values recompute: `reproduce_stage5.py` rewrites its twelve analysis
  tables and `--check-report` finds the report byte-identical,
  `reproduce_table11.py` / `reproduce_table12.py` recompute the two panel
  tables, and `tools/verify_numbers.py` compares the regenerated set number by
  number. The **typeset** LaTeX tables themselves are not regenerated: they
  live in the manuscript source, which is submitted separately and is not
  redistributed here.
* **Not regenerable here:** the S1閳ユ彎4 supplementary figures byte for byte
  (190 mm originals, see 鎼?), and the earlier Word manuscript's Fig. 9
  (its input is not shipped).
