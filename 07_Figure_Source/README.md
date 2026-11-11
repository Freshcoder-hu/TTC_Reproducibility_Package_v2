# Figure source — the plotting pipeline, run inside this repository

The nine submitted figures used to be shipped only as files
(`04_Quick_Access/Submitted_Figures/`). This directory adds the **scripts that
draw them**, ported from the submission package's `06_图表源码/` and adapted to
this repository's own layout.

| file | what it is |
|---|---|
| `fig_body.py` | submitted Figs. 1–8 (observability, coupling, scenes, main comparison, per-scenario, ablation, perception, three-turn panel) |
| `fig_loss.py` | submitted Fig. 9 (robot-loss panel) |
| `fig_si.py` | supplementary Figs. S1–S4 (not shipped as files here) |
| `make_graphical_abstract.py` | the CEA graphical abstract (not a body figure) |
| `v9style.py` | the shared design system: Okabe–Ito palette, type scale, 183.6 mm canvas, QA gate that refuses to write a figure below 7 pt or with overlapping text |
| `pkgpaths.py` | path resolution for this repository (see below) |
| `compare_figures.py` | runs the four scripts and grades every output against the shipped figure |

Only path resolution was changed. The drawing code is the submission package's,
byte for byte, apart from the header blocks listed in *What was adapted*.

## Inputs: where they now come from

The submission package read its result tables out of a nested archive
(`TTC_Shepherding_Complete_Reproducibility_Package.zip`, kept beside its tree)
and its two panel data sets out of `04_数据/`. Those paths do not exist here.
`pkgpaths.py` maps them onto this tree:

| submission package | this repository |
|---|---|
| nested archive, V2 tables (`paper/tables/`) | `01_Main_Experiment/paper/tables/` |
| nested archive, stage-3 observability (`results/results_v2_stage3/`) | `01_Main_Experiment/results/results_v2_stage3/` |
| nested archive, robustness `analysis/` | `02_Perception_Robustness/analysis/` |
| nested archive, online `analysis/online_measurement/` | `03_Online_Warning_and_Timing/analysis/online_measurement/` |
| `04_数据/三弯道_240/` | `05_Curved_Path_240/` |
| `04_数据/机器人损毁_1080/` | `06_Robot_Loss_1080/` |
| `04_数据/百秒规模_未随包发布/` | **not shipped** (see *Limits*) |
| `03_图表/正文图/`, `03_图表/补充图/`, `03_图表/` (output) | `07_Figure_Source/build/`, `…/build/body/`, `…/build/si/` |

Every input was checked one by one: each file the scripts read is
**byte-identical** (SHA-256) to the copy the submission package used, so the
figures are reproducible from this tree alone. The tables compared were
`table_main9.csv`, `table_per_scenario.csv`, `table_ablation.csv`,
`table_contrasts.csv`, `table_census.csv`, `observable_table_R70.csv`,
`required_range.csv`, the robustness `summary.csv`, the online
`timing_summary.csv`, `05_Curved_Path_240/{detail,summary,table_three_turn}.csv`,
its `traces/V2S5B__Ours_rollout__seed5000.npz`, and
`06_Robot_Loss_1080/{summary,attribution,detail}.csv`.

## Run

From the repository root (the folder that contains `01_Main_Experiment/`):

```bash
python 07_Figure_Source/fig_body.py                 # Figs. 1–8
python 07_Figure_Source/fig_loss.py                 # Fig. 9
python 07_Figure_Source/fig_si.py                   # S1–S4
python 07_Figure_Source/make_graphical_abstract.py  # graphical abstract
```

Output goes to `07_Figure_Source/build/` — `build/body/` for the body figures,
`build/si/` for the supplementary set, `build/` for the graphical abstract
(covered by `.gitignore`, so a run never dirties the shipped tree or the
integrity manifests). Each script also accepts `--out DIR` (`--out-dir` for the
graphical abstract) and `--dry-run`.

One command rebuilds everything **and grades it**:

```bash
python 07_Figure_Source/compare_figures.py
```

It runs the four scripts, prints their exit codes and runtimes, and compares
each output with `04_Quick_Access/Submitted_Figures/Figure_N.{pdf,png}`,
classifying it as `BYTE-IDENTICAL`, `PIXEL-IDENTICAL`, `VISUALLY-EQUIVALENT`,
`DIFFERENT` or `FAILS`. The supplementary set and the graphical abstract have
no shipped counterpart in this repository; grade them against the submission
package with `--si-ref DIR --ga-ref FILE`.

## Measured outcome (this tree, Windows 11, Python 3.13.7, matplotlib 3.10.8)

```
python 07_Figure_Source/compare_figures.py
figure             file     verdict              detail
Figure_1           Fig1.pdf PIXEL-IDENTICAL      png byte-identical, pdf differs (only /CreationDate)
…                                                        (same for Figures 2–8)
Figure_9           Fig9.pdf PIXEL-IDENTICAL      png byte-identical, pdf differs (only /CreationDate)
Figure_S1…S4       FigS*.pdf VISUALLY-EQUIVALENT 183.6 mm vs the shipped 190 mm; 1.0–2.1 % rescaled pixel diff
graphical_abstract …pdf     BYTE-IDENTICAL

script                       exit   runtime
fig_body.py                  0      5.2 s
fig_loss.py                  0      1.6 s
fig_si.py                    0      2.6 s
make_graphical_abstract.py   0      1.4 s
RESULT: BYTE-IDENTICAL=1, PIXEL-IDENTICAL=9, VISUALLY-EQUIVALENT=4
```

Read it precisely:

* **Figures 1–9 — pixel-identical.** The `.png` at 600 dpi is byte-identical to
  the shipped `Figure_N.png`; the `.pdf` is identical in every byte except the
  `/CreationDate` line (the shipped files carry the authoring date, a rerun
  carries its own). Every pixel of the 300 dpi PDF raster is equal. This is the
  strongest evidence the package can offer: the submitted figures are
  regenerated, not merely re-described.
* **Graphical abstract — byte-identical**, PDF and PNG, because the script pins
  `SOURCE_DATE_EPOCH` (reproducible-build timestamp).
* **Supplementary S1–S4 — visually equivalent but not identical.** The shipped
  SI figures were drawn on the earlier **190 mm** canvas; the design width was
  later frozen at **183.6 mm** (`v9style.W_MM`; `W190` is now an alias of it)
  and the body set was redrawn at that width. `fig_si.py` therefore reproduces
  the S1–S4 content at 183.6 mm: the extracted text is identical character for
  character, the layout is identical, and the page is 3.49 % narrower. This is
  a **version skew in the drawing library, not a path problem**; regenerating
  the 190 mm originals byte for byte would need the pre-freeze `v9style.py`,
  which is not part of either tree.

## What was adapted

| file | change |
|---|---|
| all four | header path block now uses `pkgpaths`; `PKG` still overrides the repository root |
| `fig_body.py` | `pkgpaths.raw_tree()` → `01_Main_Experiment/…` etc.; `T3` → `05_Curved_Path_240`; default figure list is **1–8** (the submitted set) instead of 1–9; `--only 9` stops with an explanation when the unshipped `P100` input is absent; added `--out` |
| `fig_si.py` | same path adaptation; output → `build/si`; added `--out` |
| `fig_loss.py` | `04_数据/机器人损毁_1080` → `06_Robot_Loss_1080`; added the `pkgpaths` import |
| `make_graphical_abstract.py` | default `--out-dir` → `build/`; everything else unchanged (it needs no data file) |
| `pkgpaths.py` | rewritten for this tree; the archive-extraction helper is gone because the data is shipped directly |

## Limits — what still cannot be regenerated here

| item | status |
|---|---|
| `fig_body.py --only 9` (the 100 s-scale 4-panel figure) | **cannot run**: needs `04_数据/百秒规模_未随包发布/{p1_ratio.csv,p2_methods.csv}`, which neither the submission package nor this repository ships. It is the *earlier Word manuscript's* Fig. 9, **not** submitted Fig. 9 — the submitted one is `fig_loss.py`. The script now says so instead of failing with a bare `FileNotFoundError`. |
| the 190 mm supplementary originals | see above: regenerated at 183.6 mm, visually equivalent, not byte-identical |
| submitted Fig. 2 | a schematic with no data input; it regenerates from code alone (pixel-identical) |
| exact `.pdf` bytes of Figs. 1–9 | not reachable through `SOURCE_DATE_EPOCH`: matplotlib writes the date as UTC (`D:…Z`) while the shipped files carry the local offset (`+08'00'`), so the one-line `/CreationDate` difference persists. The PNG is byte-identical, which is why the verdict is *pixel-identical*, not *byte-identical*. |

## A note on `paper/scripts/build_figures.py`

That older script is still shipped and still regenerates the V7-era auxiliary
plots (`fig01_observability` … `fig09_radar`). Its `fig07`–`fig09` are **not**
submitted Figs. 7–9. The submitted nine are the ones drawn by `fig_body.py` and
`fig_loss.py` in this directory.
