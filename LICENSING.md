**Before publication the authors must confirm this dual licence with all rights holders; this file is a real grant, not a placeholder.**

# Licensing

This package is dual-licensed by file type: **code under the MIT License**
(see [`LICENSE`](LICENSE)), and **data, results, tables, figures and
documentation under Creative Commons Attribution 4.0 International
(CC BY 4.0)** (see [`LICENSE-DATA`](LICENSE-DATA)). Copyright (c) 2026
Shuaihu Wang, Gaowei Zhang, Zehao Xu, Yun Huang.

The split exists so that the software can be reused and re-run with minimal
friction, while the measurements and the written material remain credited to
the authors. Both licences are grants; neither is a placeholder.

## Code — MIT

Everything that is executed or built:

| Path / pattern | Examples |
|---|---|
| `01_Main_Experiment/code/**` | simulator, controllers, `v2/` stages, `v2/stage5/` |
| `02_Perception_Robustness/code/**` | robustness drivers and analysers |
| `03_Online_Warning_and_Timing/code/**` | online-measurement drivers |
| `**/_stage2_code_snapshot/**`, `**/_stage3_code_snapshot/**` | archived code snapshots inside the results trees |
| `01_Main_Experiment/paper/scripts/**` | `build_data.py`, `build_figures.py` |
| `05_Curved_Path_240/reproduce_table11.py`, `06_Robot_Loss_1080/reproduce_table12.py` | panel recomputation scripts |
| `02_Perception_Robustness/analysis/**/*.py`, `03_Online_Warning_and_Timing/analysis/**/*.py` | analysis scripts |
| `tools/**/*.py`, `verify_package.py` | hygiene gate, spot-check, standardiser, verifier |
| `*.py`, `*.cpp`, `*.cu`, `*.h`, `*.hpp`, `*.dll`, `*.so` | all source and compiled native kernels |
| `Makefile`, `CMakeLists.txt`, `*.bat`, `*.ps1`, `*.sh`, `*.toml`, `*.cfg`, `*.ini` | build and run configuration |
| `.gitignore`, `.gitattributes`, `CITATION.cff` | repository metadata |

## Data, results and documentation — CC BY 4.0

Everything that is measured, tabulated, drawn or written:

| Path / pattern | Examples |
|---|---|
| `*.csv`, `*.jsonl`, `*.json`, `*.npz`, `*.npy` | `formal.jsonl`, matrix CSVs, reference traces, manifests |
| `01_Main_Experiment/results/**`, `02_Perception_Robustness/results/**`, `03_Online_Warning_and_Timing/results/**` | per-run records and archived outputs |
| `01_Main_Experiment/paper/data/**`, `01_Main_Experiment/paper/tables/**` | `numbers.json`, table CSVs |
| `05_Curved_Path_240/**`, `06_Robot_Loss_1080/**` | panel detail/summary/table CSVs and `traces/*.npz` (the two `reproduce_*.py` scripts are MIT, above) |
| `04_Quick_Access/**` | submitted figures (`.pdf`, `.png`), key tables and reports |
| `*.pdf`, `*.png`, `*.svg`, `*.eps` | figures and rendered documents |
| `*.md`, `*.txt` | READMEs, stage reports, documentation (except the two licence files themselves) |
| `PACKAGE_FILE_INDEX.csv`, `PACKAGE_SHA256_MANIFEST.json`, `SHA256SUMS.txt` | package metadata and checksums |

If a file fits both lists, the more specific path rule wins (for example, a
`.py` script under `05_Curved_Path_240/` is MIT even though the rest of that
folder is CC BY 4.0).

## Suggested attribution

For the data and results:

> Wang, Shuaihu; Zhang, Gaowei; Xu, Zehao; Huang, Yun (2026). Animal
> contact-risk prediction and action-conditioned rollout for multi-robot
> shepherding — reproducibility package (v2). Licensed under CC BY 4.0.

For the code, keep the copyright line and the MIT permission notice from
[`LICENSE`](LICENSE) with any copy or substantial portion.

## Third-party material

No third-party data or code is redistributed in this package. The native
kernels (`rollout.cpp`, `rs_safety.cpp`) and all Python sources are the
authors' own. The article text and its bibliography are not part of this
package, so no publisher agreement is engaged by it.
