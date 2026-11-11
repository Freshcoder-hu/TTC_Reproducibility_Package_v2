# Paper version — what this package reproduces

This package reproduces the **currently submitted** article. It does not
contain the article itself, and it does not contain any earlier draft.

## The submitted manuscript

| Field | Value |
|---|---|
| Title | *Animal contact-risk prediction and action-conditioned rollout for multi-robot shepherding* |
| Journal | Computers and Electronics in Agriculture (Elsevier) |
| Form | LaTeX, `cas-dc` two-column class, compiled to PDF |
| Body figures | **9** (Figs. 1–9) |
| Numbered tables | **12** (Tables 1–12) |
| Numbered equations | **8** |
| Algorithms | **1** |
| References | **41** (40 journal articles + the `[dataset]` entry) |
| Frozen configuration | `config_id = 89fe865b2e1d` (main matrix; the two extra panels use their own recorded ids) |

These counts are not copied from a report; they were measured directly on the
manuscript sources with

```
grep -c '\\label{fig:'   sections/*.tex   -> 9
grep -c '\\label{tab:'   sections/*.tex   -> 12
grep -c '\\label{eq:'    sections/*.tex   -> 8
grep -c '\\label{alg:'   sections/*.tex   -> 1
grep -c '^@'             refs.bib         -> 41   (40 articles + wang2026ttc_repro)
```

`refs.bib` belongs to the manuscript source, which is submitted separately and
is not redistributed here; the same URL is in `CITATION.cff` and in §10 of the
package README.

**The article is submitted to the journal separately and is deliberately not
redistributed here.** Cite and read it from the journal; this package is the
evidence behind it.

## Coverage — all 9 figures and all 12 tables

Every figure and table now has a shipped data source. The two panels that the
earlier edition of this package lacked are in **`05_Curved_Path_240/`** and
**`06_Robot_Loss_1080/`**; nothing is marked "not shipped" any more.

| item | label in the article | data in this package |
|---|---|---|
| Fig. 1 | `fig:obs` | `01_Main_Experiment/` (observability probe + traces) |
| Fig. 2 | `fig:coupling` | schematic; `01_Main_Experiment/code/v2/controller_v2.py` |
| Fig. 3 | `fig:scenes` | `01_Main_Experiment/code/v2/scenarios_v2.py` |
| Fig. 4 | `fig:main` | `01_Main_Experiment/results/results_v2_stage5/matrix/` |
| Fig. 5 | `fig:perscene` | same matrix → `paper/tables/table_per_scenario.csv` |
| Fig. 6 | `fig:ablation` | `results_v2_stage5/ablation/detail.csv` |
| Fig. 7 | `fig:perception` | `02_Perception_Robustness/results/formal.jsonl` |
| **Fig. 8** | `fig:curved` | **`05_Curved_Path_240/detail.csv` + `traces/*.npz`** |
| **Fig. 9** | `fig:loss` | **`06_Robot_Loss_1080/detail.csv`** |
| Table 1 | `tab:cost` | `01_Main_Experiment/code/v2/controller_v2.py` |
| Table 2 | `tab:config` | `code/v2/config_v2.py` |
| Table 3 | `tab:scenes` | `code/v2/scenarios_v2.py` |
| Table 4 | `tab:methods` | `results_v2_stage5/matrix/` |
| Table 5 | `tab:layers` | the three denominators (README §6) |
| Table 6 | `tab:corrected` | `formal.jsonl` (`version=safe` ∧ `factor=exact`) |
| Table 7 | `tab:main` | `paper/tables/table_main9.csv`, `table_main10.csv` |
| Table 8 | `tab:contrasts` | `paper/tables/table_contrasts*.csv` |
| Table 9 | `tab:perception` | `02_Perception_Robustness/analysis/summary.csv` |
| Table 10 | `tab:decomp` | `02_Perception_Robustness/analysis/decomposition.csv` |
| **Table 11** | `tab:curved` | **`05_Curved_Path_240/table_three_turn.csv`** ← `detail.csv` |
| **Table 12** | `tab:loss` | **`06_Robot_Loss_1080/table_failure.csv`** ← `detail.csv` |

The per-figure/per-table map with the producing script is
[`FIGURES_TO_EVIDENCE_MAP.md`](FIGURES_TO_EVIDENCE_MAP.md); the tested
recomputations for the two new panels are
`05_Curved_Path_240/reproduce_table11.py` and
`06_Robot_Loss_1080/reproduce_table12.py`.

The two panels are separate analyses with their own denominators and must not
be merged with the 2,400-run main matrix:

* `05_Curved_Path_240/` — 240 runs, seeds 5000–5029, 2 scenes × 4 controller
  rows (3 main methods + the external `VH-Path` extension) × 30 seeds;
* `06_Robot_Loss_1080/` — 1,080 runs, seeds 2000–2029, 3 methods × 3 scenes ×
  4 failure conditions × 30 paired seeds.

## The submitted figure files

The nine figures of the submitted article are shipped, byte-for-byte from the
submission package (`03_图表/正文图/`), as

```
04_Quick_Access/Submitted_Figures/Figure_1.pdf … Figure_9.pdf
04_Quick_Access/Submitted_Figures/Figure_1.png … Figure_9.png
```

They are the files the article places. The earlier edition of this package also
carried a **different V7-era nine-plot arrangement** (`fig01_observability` …
`fig09_radar`, twice: under `01_Main_Experiment/paper/figures/` and
`04_Quick_Access/Original_Editable_Figures/Main_V2/`). Only `fig01…fig06`
coincided with submitted Figs. 1–6; `fig07_intervals`, `fig08_timing` and
`fig09_radar` were extra plots of the archived data and could be mistaken for
submitted Figs. 7–9. **That set has been deleted from both locations**, so the
package no longer contains two "nine-figure" sets. `paper/scripts/build_figures.py`
is still shipped and still regenerates those auxiliary plots on demand into
`paper/figures/`; its output is **not** the submitted figure set and is not
redistributed.

## What was removed, and why

The previous editions of this package mixed several paper generations:

* a 40-reference **Word** manuscript carrying **Fig. 1–12**;
* an older **V7** draft (`.docx`, `.pdf` and its LaTeX source) under the main
  experiment folder;
* the V7-era nine-plot figure set described above (two copies);
* the Word-derived 12-figure PNG set, 16-table CSV export and their caption
  sheets in the quick-access folder.

None of those is the submitted article, and all of them contradicted the
9-figure, 12-table LaTeX version on figure count, table count or wording. They
have been removed, together with the V7 prose modules, the V7 document
builders and the internal rewrite report that described them.

What remains is one paper generation only: the submitted one, described above,
with every number traceable to a shipped record.

> If you arrived here expecting a manuscript file: it is not missing by
> accident. It is not part of a data/code reproducibility package.
