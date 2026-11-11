# TTC shepherding 閳?reproducibility package (v2)

> For a one-page quick start, see the repository-level
> [`README.md`](../README.md). This file is the detailed reference and keeps
> the full command/output/runtime record.

Data and code behind the submitted article *Animal contact-risk prediction and
action-conditioned rollout for multi-robot shepherding* (Computers and
Electronics in Agriculture). The article has 9 figures, 12 tables, 8 equations,
1 algorithm and 41 references; it is submitted separately and is **not**
redistributed here 閳?see [`PAPER_VERSION.md`](PAPER_VERSION.md).

The canonical public repository for this package is
**https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2**; the
manuscript's data-availability sentence points to it.

This is a **simulation and software-measurement** artifact. It does not contain
a real-flock or physical-robot experiment, and nothing here demonstrates
zero animal閳ユ悜obot contact.

Every command in this README was executed on this exact tree; the expected
output and the wall-clock time are printed with it. Commands that could not be
run are listed in [鎼?](#7-what-is-not-runnable-here) instead of being quoted as
if they worked.

---

## 1. Layout

```
TTC_Reproducibility_Package_v2/
閳规壕鏀㈤埞鈧?00_Documentation/          this README + paper version, data notes, figure map
閳规壕鏀㈤埞鈧?01_Main_Experiment/        V2 main experiment (2400 runs), stage reports
閳?  閳规壕鏀㈤埞鈧?code/                  simulator + controllers + v2/stage5 tooling
閳?  閳规壕鏀㈤埞鈧?paper/                 data/, tables/, scripts/ (figure plots are regenerated)
閳?  閳规壕鏀㈤埞鈧?reports/               stage 1閳? and credibility reports
閳?  閳规柡鏀㈤埞鈧?results/               results_v2_stage1 閳?stage5, results_v2_credibility
閳规壕鏀㈤埞鈧?02_Perception_Robustness/  the 2340-cell robustness corpus (authoritative)
閳规壕鏀㈤埞鈧?03_Online_Warning_and_Timing/  online warning / latency measurement (132 + 24)
閳规壕鏀㈤埞鈧?04_Quick_Access/           convenience copies: key tables, reports, figures
閳?  閳规柡鏀㈤埞鈧?Submitted_Figures/     Figure_1閳? (.pdf + .png), byte-for-byte from the submission
閳规壕鏀㈤埞鈧?05_Curved_Path_240/        three-turn panel, 240 runs 閳?Fig. 8 / Table 11
閳规壕鏀㈤埞鈧?06_Robot_Loss_1080/        robot-loss panel, 1,080 runs 閳?Fig. 9 / Table 12
閳规壕鏀㈤埞鈧?07_Figure_Source/          the plotting pipeline 閳?submitted Figures 1閳?
閳?  閳规壕鏀㈤埞鈧?fig_body.py            Figs. 1閳?          fig_loss.py   Fig. 9
閳?  閳规壕鏀㈤埞鈧?fig_si.py              S1閳ユ彎4 (not shipped as files)
閳?  閳规壕鏀㈤埞鈧?make_graphical_abstract.py   the CEA graphical abstract
閳?  閳规壕鏀㈤埞鈧?v9style.py             palette, type scale, 183.6 mm canvas, QA gate
閳?  閳规壕鏀㈤埞鈧?pkgpaths.py            path map for this repository
閳?  閳规柡鏀㈤埞鈧?compare_figures.py     rebuild + grade against Submitted_Figures/
閳规壕鏀㈤埞鈧?tools/                     spot-check, standardiser, hygiene gate, stager
閳规壕鏀㈤埞鈧?LICENSE                    MIT 閳?all code
閳规壕鏀㈤埞鈧?LICENSE-DATA               CC BY 4.0 閳?data, results, tables, figures, docs
閳规壕鏀㈤埞鈧?LICENSING.md               which paths fall under which licence
閳规壕鏀㈤埞鈧?CITATION.cff               citation metadata (canonical repository URL recorded)
閳规壕鏀㈤埞鈧?.gitignore / .gitattributes  GitHub-ready ignore and binary rules
閳规壕鏀㈤埞鈧?SHA256SUMS.txt             every file, sha256
閳规壕鏀㈤埞鈧?PACKAGE_SHA256_MANIFEST.json / PACKAGE_FILE_INDEX.csv
閳规柡鏀㈤埞鈧?verify_package.py          one-command integrity check
```

## 2. Environment

| item | value |
|---|---|
| OS used for verification | Windows 11 x64 |
| Python | **3.13.7** (the original runs were made on 3.12) |
| numpy | **2.5.2** (original runs: 2.3.5) |
| scipy | **1.17.1** (original runs: 1.17.0) |
| pandas | 2.3.3 (only some analysis scripts) |
| matplotlib | 3.10.8 (the figure pipeline `07_Figure_Source/` and `build_figures.py`; the submitted Figures 1閳? come back pixel-identical with this version) |
| native kernels | Windows `_rollout.dll` shipped; `rollout.cpp`, `rs_safety.cpp` provided for other platforms |

**Pin BLAS to one thread.** Several reported numbers are sums over runs, and a
multi-threaded BLAS changes float accumulation order:

```bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# PowerShell:
$env:OMP_NUM_THREADS=1; $env:OPENBLAS_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
```

Rebuilding the native kernels elsewhere (needs C++17):

```bash
g++ -O3 -std=c++17 -shared -fPIC rollout.cpp -o _rollout.so
g++ -O3 -ffp-contract=off -std=c++17 -shared -fPIC rs_safety.cpp -o _rs_safety.so
```

## 3. Quick start 閳?the tested entry points

Run from the package root unless the row says otherwise. `R` = runtime
measured on this tree.

| # | command | expected output | R |
|---|---|---|---|
| 1 | `python tools/spotcheck.py` | 12 `[PASS]` lines, then `RESULT: PASS - 閳ヮ泦 | 0.4 s |
| 2 | `python tools/standardise_formal.py --verify` | `ROUND-TRIP: PASS - every cell re-read matches 閳ヮ泦 | 0.7 s |
| 3 | `python tools/check_hygiene.py` | per-extension counts, then `HYGIENE: PASS` (repo mode is the default and tolerates `.git/`, `.gitignore`, `.gitattributes`, `.github/`; `--package-mode` is the old strict check) | 0.4 s |
| 4 | `python 01_Main_Experiment/code/reproduce_stage5.py` | `wrote 12 tables + report_numbers.json to 閳ヮ泦 | 1.6 s |
| 5 | `python 01_Main_Experiment/code/reproduce_stage5.py --check-report` | `CHECK-REPORT: PASS (the document is byte-identical 閳?` | 1.8 s |
| 6 | `python tools/verify_numbers.py` | after snapshotting `reproduced/` (see below): `RESULT: PASS - 13/13 files; N numeric difference(s) within tolerance; 0 error(s)` | 0.1 s |
| 7 | `python 05_Curved_Path_240/reproduce_table11.py` | Table 11 rows, `RESULT: PASS - Table 11 / Fig. 8 閳ヮ泦 | 0.1 s |
| 8 | `python 06_Robot_Loss_1080/reproduce_table12.py` | V2S3 1.67 閳?6.94 %, `1080/1080`, `RESULT: PASS - 閳ヮ泦 | 0.2 s |
| 9 | `python 01_Main_Experiment/paper/scripts/build_data.py` | ends `censored total: 377` / `ablation 300 rows; formal 2400 rows` | 59 s |
| 10 | `python 01_Main_Experiment/paper/scripts/build_figures.py` | 9 lines `figure fig01_observability` 閳?| 12 s |
| 11 | `python tools/stage_online_inputs.py` | `staged hard link 閳ヮ泦 + sha256 of the corpus | 0.2 s |
| 12 | `python 03_Online_Warning_and_Timing/analysis/analyze_online_evidence.py` | JSON summary on stdout, exit 0 | 5.9 s |
| 13 | `python tools/stage_online_inputs.py --clean` | `removed 閳?results/formal.jsonl` | 0.1 s |
| 14 | `python verify_package.py` | `Verified 1040 files; errors = 0` | see 鎼? |
| 15 | `python 07_Figure_Source/fig_body.py` | 8 lines `Fig1 閳?Fig8  183.60 mm  texts=閳? min 7.5 pt`, `wrote 8 figure(s) to 閳ヮ泦 | 5.2 s |
| 16 | `python 07_Figure_Source/fig_loss.py` | `Fig9  183.60 mm  texts=60  min 7.5 pt` | 1.6 s |
| 17 | `python 07_Figure_Source/fig_si.py` | `FigS1 閳?FigS4`, `wrote 4 figure(s) to 閳ヮ泦 | 2.6 s |
| 18 | `python 07_Figure_Source/make_graphical_abstract.py` | `canvas 130.00 x 66.00 mm` 閳?`written 閳?graphical_abstract.pdf` | 1.4 s |
| 19 | `python 07_Figure_Source/compare_figures.py` | graded table, `RESULT: BYTE-IDENTICAL=1, PIXEL-IDENTICAL=9, VISUALLY-EQUIVALENT=4` | 12 s |

Commands 4閳?, 9閳?0 and 15閳?9 write into `results_v2_stage5/reproduced/`,
`paper/figures/` and `07_Figure_Source/build/`; within one environment they are
idempotent (a second run produces byte-identical output). Across environments
the bytes may differ in line endings or a float's last digit even though every
reported number agrees; command 6 checks that case (see *Two kinds of
verification* below).
`paper/figures/` is **not shipped**: the submitted Figures 1閳? live in
`04_Quick_Access/Submitted_Figures/`, and `build_figures.py` regenerates only
the auxiliary V7-era re-plots on demand (its `fig07`閳ユ彵fig09` are not submitted
Figs. 7閳?). Commands 15閳?9 are the **submitted** figure pipeline 閳?see 鎼?.9:
all nine come back pixel-identical. Commands 11閳?3 stage the shared corpus
temporarily and clean it up; after `--clean` the tree is byte-identical to the
delivered one.

## Two kinds of verification

This package is accepted in two separate steps. They answer different
questions, and they fail for different reasons.

1. **Archive integrity** 閳?*before running anything*, every file must match the
   SHA-256 recorded in `PACKAGE_SHA256_MANIFEST.json` (and listed in
   `SHA256SUMS.txt`). That is the byte-level check performed by
   `verify_package.py`; a failure means the archive was altered or damaged, and
   no numerical claim should be made from it.

2. **Numerical reproduction** 閳?*after* re-running a generator, compare the
   **numbers**, not the bytes. Line endings are environment-specific (a Windows
   run writes CRLF, a POSIX run LF), and the last digit of a float can depend on
   the NumPy/BLAS/SciPy build. The regenerated tables may therefore differ from
   the shipped copies in exactly those respects while every reported statistic
   is unchanged. `tools/verify_numbers.py` performs this check.

Tolerance: `rtol = 1e-12`, `atol = 0`. For values of order one this is about
four orders of magnitude above IEEE-754 double round-off, and far below the
three-significant-decimal precision at which results are reported; it absorbs
last-bit cross-environment drift without hiding a real change.

Stage-5 usage:

```powershell
# 1. snapshot the shipped outputs
Copy-Item 01_Main_Experiment\results\results_v2_stage5\reproduced `
          01_Main_Experiment\results\results_v2_stage5\reproduced_archive -Recurse
# 2. regenerate
python 01_Main_Experiment\code\reproduce_stage5.py
# 3. compare numbers, not bytes
python tools\verify_numbers.py
```

The tool normalises CRLF/CR to LF, compares every non-numeric character and
every JSON key exactly, and compares every number with the tolerance above. It
lists every difference it tolerates (one `[EOL]` line per line-ending-only
file, one `[TOL]` line per accepted float), and exits non-zero only for a
missing/extra file or key, a changed non-numeric field, or a number outside
tolerance. With no arguments it compares `.../reproduced_archive` against
`.../reproduced`; `--archive` and `--regen` select other directories. The
snapshot is a temporary verification artefact -- delete `reproduced_archive/`
when finished; it is not part of the shipped package.

## 4. Entry points in detail

### 4.1 Stage-5 recomputation 閳?`reproduce_stage5.py`

```bash
python 01_Main_Experiment/code/reproduce_stage5.py              # tables + numbers
python 01_Main_Experiment/code/reproduce_stage5.py --write-report
python 01_Main_Experiment/code/reproduce_stage5.py --check-report
```

Rebuilds all 12 Stage-5 tables and `results_v2_stage5/reproduced/report_numbers.json`
from the CSVs, then (with `--write-report`) rewrites
`reports/V2_STAGE5_REPORT.md` from a prose template plus those tables, so the
document cannot drift from the data. `--check-report` fails unless the file on
disk matches the regenerated one after line-ending normalisation; it does not
require byte identity across environments (see *Two kinds of verification*).

> **The module this entry needs was missing from the previous package.** The
> shipped `reproduce_stage5.py` imported `v2.stage5`, and `code/v2/stage5/` did
> not exist in any of the three experiment folders 閳?the entry died on
> `ModuleNotFoundError: No module named 'v2.stage5'` (exit 1). The module was
> **recovered intact from the author's archive** (15 files) and is now shipped
> at `01_Main_Experiment/code/v2/stage5/`. It is not a stand-in or a rewrite.
> Two paths were adapted so it runs inside this layout: the results root now
> resolves to the sibling `results/` directory, and the command block printed
> inside the report names this package instead of the author's build machine.
> Everything else is the original code, and the regenerated report differs from
> the previously shipped one on exactly one line 閳?the `cd` line. Proof:
> `--check-report` passes.

### 4.2 Spot-check of the published headline numbers 閳?`tools/spotcheck.py`

Recomputes, from the shipped records and nothing else:

* the three graphical-abstract contacted-animal fractions and completions 閳?  `0.05463`, `0.248611`, `0.244444` and `54/60`, `51/60`, `49/60` for
  Proposed / Path-only / Distance-reactive;
* the three headline counts 閳?2400 main runs, 2340 robustness cells
  (1980 perception + 360 exact-observation control), 24 timing runs.

Details and the arithmetic are printed by the script; see also 鎼?.1.

### 4.3 Main-experiment tables 閳?`paper/scripts/build_data.py`

Reads `results/results_v2_stage5/{matrix,ablation,checks,trec_sweep,c1_audit}`
and writes `paper/data/numbers.json` plus the eight `paper/tables/*.csv`.
It recomputes the censored-run total (377) and the seed-clustered contrast
count, so it doubles as a consistency check on the archived matrix.

### 4.4 Figures 閳?`paper/scripts/build_figures.py`

Draws the auxiliary V7-era re-plots `paper/figures/fig01閳ヮ泴ig09` from
`paper/data/numbers.json` and `paper/tables/*.csv` 閳?run `build_data.py` first.
`fig01閳ヮ泴ig06` correspond to submitted Figs. 1閳?; `fig07_intervals`,
`fig08_timing` and `fig09_radar` are **extra plots of archived data, not**
submitted Figs. 7閳?. See
[`FIGURES_TO_EVIDENCE_MAP.md`](../00_Documentation/FIGURES_TO_EVIDENCE_MAP.md).
The submitted figures themselves are shipped in
`04_Quick_Access/Submitted_Figures/`.

### 4.5 Robustness corpus 閳?read-only

`02_Perception_Robustness/results/formal.jsonl` is the 2,340-cell record and is
**the only formal data** for the robustness layer. `analysis/all_runs.csv`,
`summary.csv`, `paired_comparisons.csv`, `degradation.csv` and
`decomposition.csv` are its flattened forms and are shipped so that the paper's
robustness numbers can be checked without rerunning 2,340 simulations.
Reading them needs nothing beyond Python.

### 4.6 Online warning / latency 閳?stage, run, clean

`03_Online_Warning_and_Timing/analysis/analyze_online_evidence.py` reads
`results/formal.jsonl` **relative to its own folder**, and two scripts in that
folder are fingerprint-guarded (they hash each other), so the path cannot be
rewritten inside them. The corpus is stored once, in
`02_Perception_Robustness/results/`. Commands 11閳?3 stage it as a hard link
(no extra disk space), run the analysis, and remove the link.

### 4.7 Three-turn panel 閳?`05_Curved_Path_240/reproduce_table11.py`

```bash
python 05_Curved_Path_240/reproduce_table11.py
```

Reads `05_Curved_Path_240/detail.csv` only (240 runs, 47 columns) and
recomputes the **Table 11 / Fig. 8** key numbers: per-method mean
animal閳ユ悐bstacle contact fraction and goal-and-cohesion completion on V2S5B,
plus the literal `D_turn`/`E_turn` 30-seed means. It also cross-checks the
shipped `table_three_turn.csv` and states that the centroid-window `D_turn`
exists for seed 5000 only. Measured output (abridged; every line is `[PASS]`):

```
   method        n    C mean +/- sd      runs C>0   D_turn mean +/- sd   E_turn mean / max   GC      n_enc
   Ours          30   0.0028 +/- 0.0112     2/30      0.988 +/- 0.187      0.045 / 0.904     27/30   27/30
   B1            30   0.0250 +/- 0.0616     7/30      1.082 +/- 0.520      0.161 / 1.947     25/30   29/30
   B2            30   0.0231 +/- 0.0632     5/30      1.016 +/- 0.316      0.087 / 1.340     26/30   28/30
   VH-Path       30   0.1639 +/- 0.1935    17/30      2.052 +/- 0.792      1.052 / 3.306     17/30   18/30
   ...
   NOTE: the other 29 seeds carry no centroid-window value, so these four
   numbers are a single-seed illustration, not panel means.
RESULT: PASS - Table 11 / Fig. 8 key numbers reproduce from 05_Curved_Path_240/detail.csv
```

Measured runtime **0.1 s**, exit 0.

### 4.8 Robot-loss panel 閳?`06_Robot_Loss_1080/reproduce_table12.py`

```bash
python 06_Robot_Loss_1080/reproduce_table12.py
```

Reads `06_Robot_Loss_1080/detail.csv` only (1,080 runs, 347 columns) and
recomputes the **Table 12 / Fig. 9** key numbers: the V2S3
animals-with-obstacle-contact fraction across F0 閳?F3, the proposed
controller's goal-and-cohesion completion in all 12 scene 鑴?condition cells,
and the 1,080/1,080 arrival count. Measured output (abridged; every line is
`[PASS]`):

```
2. Arrival and censoring
   arrival: 1080/1080   timeout: 0   censored: 0
3. Fig. 9(a): animals with obstacle contact, Ours, by scene x condition [%]
   scene   F0     F1     F2     F3
   V2S1     0.00   0.00   0.00   0.00
   V2S2     0.56   0.56   0.56   0.46
   V2S3     1.67   3.15   3.24   6.94
   V2S3: 1.67 % -> 3.15 % -> 3.24 % -> 6.94 % across F0 -> F3 (got 1.67 -> 6.94)
4. Table 12, Ours: goal-and-cohesion completion [%]
   scene   F0     F1     F2     F3
   V2S1     86.7   83.3   76.7   83.3
   V2S2     90.0   86.7   80.0   86.7
   V2S3     76.7   73.3   73.3   70.0
RESULT: PASS - Table 12 / Fig. 9 key numbers reproduce from 06_Robot_Loss_1080/detail.csv
```

Measured runtime **0.2 s**, exit 0.

### 4.9 Submitted figures 閳?`07_Figure_Source/`

The plotting pipeline of the submitted article. It is the submission package's
`06_閸ユ崘銆冨┃鎰垳/`, ported with only its path resolution changed: the scripts used
to read the V2 result tables out of a nested archive
(`TTC_Shepherding_Complete_Reproducibility_Package.zip`, kept beside the
submission tree); this repository ships those tables directly, and every input
the scripts read is **byte-identical** to the archived copy
(`07_Figure_Source/README.md` lists them one by one).

```bash
python 07_Figure_Source/fig_body.py                 # submitted Figs. 1閳?
python 07_Figure_Source/fig_loss.py                 # submitted Fig. 9
python 07_Figure_Source/fig_si.py                   # S1閳ユ彎4
python 07_Figure_Source/make_graphical_abstract.py  # graphical abstract
python 07_Figure_Source/compare_figures.py          # rebuild everything and grade it
```

Output goes to `07_Figure_Source/build/` (covered by `.gitignore`). Measured on
this tree 閳?`python 07_Figure_Source/compare_figures.py`:

```
figure             file      verdict              note
Figure_1 閳?Figure_8 Fig1閳?.pdf PIXEL-IDENTICAL    png byte-identical; pdf differs only in /CreationDate
Figure_9           Fig9.pdf  PIXEL-IDENTICAL      png byte-identical; pdf differs only in /CreationDate
Figure_S1 閳?S4     FigS*.pdf VISUALLY-EQUIVALENT  183.6 mm vs the shipped 190 mm originals
graphical_abstract 閳ヮ洊df      BYTE-IDENTICAL
RESULT: BYTE-IDENTICAL=1, PIXEL-IDENTICAL=9, VISUALLY-EQUIVALENT=4
```

Runtimes: `fig_body.py` 5.2 s, `fig_loss.py` 1.6 s, `fig_si.py` 2.6 s,
`make_graphical_abstract.py` 1.4 s, the whole graded run 12 s.

So the nine submitted figures are **regenerated, not merely re-described**.
Two limits are documented rather than hidden: `fig_body.py --only 9` rebuilds
the *earlier Word manuscript's* 100 s-scale Fig. 9 and cannot run without an
input that neither tree ships (the submitted Fig. 9 is `fig_loss.py`); and the
shipped S1閳ユ彎4 were drawn before the design width was frozen at 183.6 mm, so
`fig_si.py` reproduces their content at 183.6 mm, not the 190 mm files.

## 5. Seed blocks

| block | seeds | used by | in this package |
|---|---|---|---|
| main evaluation | **2000閳?029** | 8 methods 鑴?10 scenes 鑴?30 = **2400** runs; ablation R0閳ユ彍3; robot-loss panel reuses the block as paired seeds | yes 閳?`results_v2_stage5/matrix/*.csv`, `ablation/detail.csv`, `06_Robot_Loss_1080/` |
| corrected protocol | **3000閳?029** | perception robustness, 13 conditions 鑴?3 methods 鑴?2 scenes = 78 閳?**2340** cells | yes 閳?`02_Perception_Robustness/results/formal.jsonl` |
| curved corridor | **5000閳?029** | confirmatory three-turn panel, 4 controller rows 鑴?2 scenes 鑴?30 = **240** | yes 閳?`05_Curved_Path_240/detail.csv` |
| calibration | **1000閳?019** | M1/M2 calibration and the C1 / T_REC sweeps | partly 閳?the shipped Stage-5 sweeps use 1000閳?004 (`trec_sweep`) and 1000閳?002 (C1 stress) |
| diagnostics | **1005閳?009** and **3000** | geometry/spacing diagnostics; 3000 is the single seed of the serial timing traces | partly 閳?timing traces are `閳ヮ泤_3000.npz`; 1005 appears in the spacing calibration |

Seeds are never shared between blocks, and no denominator is carried from one
layer to another. The robot-loss panel reuses seeds 2000閳?029 deliberately (it
replays the frozen F0 matrix as its control) but is a **separate experiment**,
not 1,080 more main runs.

## 6. Denominators, and what may not be merged

| layer | formal N | file |
|---|---|---|
| main V2 matrix | **2400** runs (incl. 377 right-censored, which stay in the denominator) | `01_Main_Experiment/results/results_v2_stage5/matrix/*.csv` |
| robustness | **2340** cells = 1980 perception + 360 exact-observation control | `02_Perception_Robustness/results/formal.jsonl` |
| online measurement | **132** event replays + **24** independent serial timing runs | `03_Online_Warning_and_Timing/analysis/online_measurement/` |
| three-turn panel | **240** runs = 4 controller rows 鑴?2 scenes 鑴?30 seeds | `05_Curved_Path_240/detail.csv` |
| robot-loss panel | **1,080** runs = 3 methods 鑴?3 scenes 鑴?4 conditions 鑴?30 seeds | `06_Robot_Loss_1080/detail.csv` |

The 240 ablation runs, the 60 probe runs, the 132 replays, the 24 timing runs,
the **240 three-turn** runs and the **1,080 robot-loss** runs are **separate
analyses**. They must not be added to any of the three main denominators, and
`formal_epoch1.jsonl` / `formal_attempt1.jsonl` must not be concatenated with
`formal.jsonl` (the checkpoint is already inside it).

### 6.1 `formal.jsonl` contains `NaN` and `Infinity`

It is **not strict JSON**: it uses the bare tokens `NaN`, `Infinity` and
`-Infinity`, which RFC 8259 does not define. Python accepts them; `jq`,
JavaScript, Go, Rust and most parquet/CSV loaders do not. Here `Infinity` means
"this contact never happened", and `NaN` means "this phase did not occur" 閳?substituting `0` would invent a contact at t = 0 and a zero-length recovery.

A standardised copy is shipped beside it:

```
02_Perception_Robustness/results/formal_standardised.csv   4.22 MiB, 2340 rows, 226 cols
```

Non-finite cells are empty (never `0`), genuine JSON `null` is also empty but
distinguishable, and a trailing **`_nonfinite`** column names every non-finite
cell and its kind, e.g. `recovery=NaN;first_contact_time_rr=Infinity`. Regenerate
and prove it with `python tools/standardise_formal.py --verify`. Full details:
[`DATA_NOTES.md`](DATA_NOTES.md).

## 7. What is not runnable here

Listed so that nobody wastes a day on them. These files are shipped because
they are part of the record, not because they run in this layout.

| entry | why it cannot run |
|---|---|
| `python -m v2.acceptance_v2` | needs the V1 comparison archive `results_cea_matrix/_parts/`, which is not part of this package |
| `python -m v2.acceptance_s2` | reaches `acceptance_v2.archived_row`, i.e. the same missing archive |
| `python -m v2.acceptance_s3`, `閳ヮ泤s4` | their own stage archives are now found, but the suites are long and depend on the same V1 archive for the inertness tests. **`acceptance_s4.py` now compiles** (its indentation error is fixed), but that is a syntax repair, not an end-to-end run: the V1 archive is still required |
| `measure_online_evidence.py` | re-runs the 132-event measurement; Linux-oriented and expensive by design |
| `check_timing_overhead.py` | calls `os.sched_setaffinity`, which does not exist on Windows |
| the full 2400-run matrix (`v2/runner_v2`, `v2/stage5.matrix`) | the archived CSVs are shipped, so the rerun is optional; it needs 12 jobs and hours |
| re-running the 240-run three-turn panel or the 1,080-run robot-loss panel | the per-run records are shipped, so the recomputations above suffice; re-simulating needs the author's panel drivers, thousands of CPU-hours and a byte-identical build of the native kernels |
| `python 07_Figure_Source/fig_body.py --only 9` | needs the 100 s-scale panel (`04_閺佺増宓?閻у墽顫楃憴鍕乢閺堫亪娈㈤崠鍛絺鐢?`), which neither this package nor the submission package ships. It is the **earlier Word manuscript's** Fig. 9, not submitted Fig. 9 (that one is `fig_loss.py`) |
| `07_Figure_Source/fig_si.py` reproducing S1閳ユ彎4 byte for byte | the shipped S1閳ユ彎4 were drawn on the earlier 190 mm canvas; the design width is now frozen at 183.6 mm, so the content reproduces at 183.6 mm (same text, same layout, 3.49 % narrower) |

The previous edition quoted `python -m v2.reproduce_stage5` and
`python -m v2.credibility.run_all`. **Neither ever existed** 閳?`reproduce_stage5`
is a top-level script, not a `v2` module, and there is no `v2/credibility`
package. Both have been removed. Every remaining command was executed, and its
measured output is in 鎼?.

## 8. Figure and table map

| Fig. | what | where the number comes from | drawn by |
|---|---|---|---|
| 1 | observable warning time | `paper/data/numbers.json` 閳?`build_data.py`; probe `code/v2/observability_v2.py` | `07_Figure_Source/fig_body.py` |
| 2 | the coupling (schematic) | definitions in `code/v2/controller_v2.py` | `07_Figure_Source/fig_body.py` |
| 3 | scenario geometry | `code/v2/scenarios_v2.py` | `07_Figure_Source/fig_body.py` |
| 4 | nine-scenario comparison | `results_v2_stage5/matrix/*.csv` (2400 runs) | `07_Figure_Source/fig_body.py` |
| 5 | contact fraction by scene 鑴?method | same matrix | `07_Figure_Source/fig_body.py` |
| 6 | ablation ladder | `results_v2_stage5/ablation/detail.csv` | `07_Figure_Source/fig_body.py` |
| 7 | perception-response profile | `02_Perception_Robustness/results/formal.jsonl` 閳?`analysis/summary.csv` | `07_Figure_Source/fig_body.py` |
| 8 | three-turn panel | `05_Curved_Path_240/detail.csv` + `traces/*.npz` 閳?`reproduce_table11.py` | `07_Figure_Source/fig_body.py` |
| 9 | robot-loss panel | `06_Robot_Loss_1080/detail.csv` 閳?`reproduce_table12.py` | `07_Figure_Source/fig_loss.py` |

| Table | what | where |
|---|---|---|
| 1閳? | cost terms, frozen configuration, scenes | `code/v2/{controller_v2,config_v2,scenarios_v2}.py` |
| 4 | the eight methods | `results_v2_stage5/matrix/` (one CSV per method) |
| 5 | evidence layers / denominators | 鎼? of this README |
| 6 | corrected protocol (3000閳?029) | `formal.jsonl`, `version=safe` 閳?`factor=exact` |
| 7閳? | archived comparison + paired contrasts | `paper/tables/table_main9.csv`, `table_contrasts*.csv` |
| 9閳?0 | 11-condition contact fraction, decomposition | `02_Perception_Robustness/analysis/{summary,decomposition}.csv` |
| 11 | three-turn panel | `05_Curved_Path_240/table_three_turn.csv` 閳?`detail.csv` |
| 12 | robot-loss panel | `06_Robot_Loss_1080/table_failure.csv` 閳?`detail.csv` |

The nine submitted plot files are in `04_Quick_Access/Submitted_Figures/`
(`Figure_1閳?`, `.pdf` + `.png`, byte-for-byte from the submission) and are
**regenerated** by `07_Figure_Source/fig_body.py` (Figs. 1閳?) and
`07_Figure_Source/fig_loss.py` (Fig. 9) 閳?see 鎼?.9. Full version, with one row
per figure/table and the producing script:
[`FIGURES_TO_EVIDENCE_MAP.md`](FIGURES_TO_EVIDENCE_MAP.md).

## 9. Integrity

`SHA256SUMS.txt` lists every file in the package. Verify everything:

```bash
python verify_package.py                    # uses PACKAGE_SHA256_MANIFEST.json
```

or with standard tools:

```bash
sha256sum -c SHA256SUMS.txt                 # Linux / macOS
Get-FileHash -Algorithm SHA256 -Path 閳?     # PowerShell, per file
```

`PACKAGE_FILE_INDEX.csv` gives path, category, byte count and SHA-256 for each
entry. The manifest was regenerated after the last edit; see the report that
accompanies this package for the verified count and a clean-extraction
recompute.

## 10. Provenance and licence

`00_Documentation/SOURCE_PROVENANCE.md` records the SHA-256 of each source
archive this package was assembled from. The two extra panels were copied
byte-for-byte from the submission package's `04_閺佺増宓?娑撳闆嗛柆鎻?40/` and
`04_閺佺増宓?閺堝搫娅掓禍鐑樺疮濮ｄ耿1080/`; the submitted figures from `03_閸ユ崘銆?濮濓絾鏋冮崶?`.

Licence: the package is dual-licensed. **Code** (`.py`, `.cpp`, `.cu`, build
files, scripts) is under the **MIT License** ([`LICENSE`](../LICENSE));
**data, results, tables, figures and documentation** are under **CC BY 4.0**
([`LICENSE-DATA`](../LICENSE-DATA)). The path-by-path split and the required
pre-publication confirmation are in [`LICENSING.md`](../LICENSING.md).
Copyright (c) 2026 Shuaihu Wang, Gaowei Zhang, Zehao Xu, Yun Huang.

`CITATION.cff` records the repository in `repository-code`:
`https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2`. The two
filled slots (the manuscript data-availability sentence and the `CITATION.cff`
field) are recorded with their date in [`REPOSITORY_URL.md`](REPOSITORY_URL.md).

## 11. Known limits

* Simulation and software timing only; no real animals, no physical robots, no
  camera, network or hardware real-time claim.
* The contact measure `C` is the fraction of animals that touch an obstacle in a
  run 閳?not a per-run collision probability. Robot閳ユ彸nimal, robot閳ユ悐bstacle and
  robot閳ユ悜obot contacts are reported separately and are not merged.
* The model does **not** demonstrate zero robot閳ユ彸nimal contact; residual
  contacts remain and are reported.
* The three-turn panel (`05_Curved_Path_240/`) is a **single build** and is not
  bit-reproducible: its closed loop is sensitive to a 10閳﹀瀛?relative
  perturbation. Its absolute values are therefore not portable, although the
  paired seed comparisons are internally valid. Its centroid-window `D_turn`
  exists for seed 5000 only and is a single-seed illustration.
* The robot-loss panel (`06_Robot_Loss_1080/`) measures what the existing
  controller does when a robot is lost; the implementation has **no failure
  detection, no treatment of a disabled robot as an obstacle and no
  failure-aware reassignment**, so it is not a claim that the fleet tolerates a
  loss. V2S1 has no obstacle, so its F3 cells repeat F1 (`f3_applicable = 0`).
* The archived (2000閳?029) matrix is auxiliary evidence in the article; the
  headline comparison is the corrected protocol (3000閳?029).
