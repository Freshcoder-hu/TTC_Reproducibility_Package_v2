# Figure and table → data map

Every figure and table number of the submitted article, and the shipped file
that produces it. Paths are relative to the package root. Nothing is missing:
the two panels that the earlier edition could not ship are now in
`05_Curved_Path_240/` and `06_Robot_Loss_1080/`.

The article has **9 figures**, **12 tables**, 8 equations, 1 algorithm and 40
references (see [`PAPER_VERSION.md`](PAPER_VERSION.md)).

---

## Figures

| Fig. | Topic (label) | Numeric source in this package | Producer | Status |
|---|---|---|---|---|
| 1 | Observable warning time (`fig:obs`) | `paper/data/numbers.json` → `derived`; observability probe `01_Main_Experiment/code/v2/observability_v2.py`, traces in `results/results_v2_stage4/` | `paper/scripts/build_data.py` → `build_figures.py` (`fig01_observability`) | data + submitted plot |
| 2 | The coupling — risk term feeding timing and rollout cost (`fig:coupling`) | schematic; definitions in `code/v2/controller_v2.py` and `code/v2/stage5/common.py` (`ENDPOINTS`, TTC tuple) | `paper/scripts/build_figures.py` (`fig02_coupling`) | no data by design |
| 3 | Scenario geometry, V2S1/V2S1N/V2S2… (`fig:scenes`) | `01_Main_Experiment/code/v2/scenarios_v2.py` (arc length, lateral offset, obstacle speed) | `paper/scripts/build_figures.py` (`fig03_geometry`) | data + submitted plot |
| 4 | Archived nine-scenario comparison (`fig:main`) | `01_Main_Experiment/results/results_v2_stage5/matrix/*.csv` — 8 files × 300 runs, seeds 2000–2029; aggregated to `paper/tables/table_main9.csv` + `table_main10.csv` | `build_data.py` → `build_figures.py` (`fig04_main_comparison`) | data + submitted plot |
| 5 | Contact fraction by scene × method (`fig:perscene`) | same matrix → `paper/tables/table_per_scenario.csv` | `build_data.py` → `build_figures.py` (`fig05_per_scenario`) | data + submitted plot |
| 6 | Archived ablation ladder R0→R3 (`fig:ablation`) | `results/results_v2_stage5/ablation/detail.csv` (4 rungs × 2 scenes × 30 seeds) + `R0…R3.csv` → `paper/tables/table_ablation.csv` | `build_data.py` → `build_figures.py` (`fig06_ablation`) | data + submitted plot |
| 7 | Perception-response profile, 11 observation conditions (`fig:perception`) | `02_Perception_Robustness/results/formal.jsonl` (`version=safe`, seeds 3000–3029) → `analysis/summary.csv`, `degradation.csv` | `02_Perception_Robustness/analysis/analyze.py`, plots in `analysis/figures/` | data + submitted plot |
| 8 | Confirmatory three-turn panel, seeds 5000–5029 (`fig:curved`) | `05_Curved_Path_240/detail.csv` (240 runs) + `traces/*.npz` (plan view, seed 5000) | `05_Curved_Path_240/reproduce_table11.py` recomputes the numbers | **shipped** |
| 9 | Robot-loss panel, 1,080 runs (`fig:loss`) | `06_Robot_Loss_1080/detail.csv` (1,080 runs) + `summary.csv`, `attribution.csv` | `06_Robot_Loss_1080/reproduce_table12.py` recomputes the numbers | **shipped** |

The nine **submitted** plot files are shipped byte-for-byte from the
submission package at `04_Quick_Access/Submitted_Figures/Figure_1…9`
(`.pdf` + `.png`). They are the files the article places.

`paper/scripts/build_figures.py` still regenerates the V7-era auxiliary
arrangement into `paper/figures/` (`fig01_observability` … `fig09_radar`):
`fig01…fig06` correspond to submitted Figs. 1–6, while `fig07_intervals`,
`fig08_timing` and `fig09_radar` are **not** submitted Figs. 7–9 — they are
extra plots of the same archived data. That set is no longer shipped as files
(it was deleted to avoid a second nine-figure version); only the script that
can rebuild it remains.

## Tables

| Table | Topic (label) | Numeric source in this package | Status |
|---|---|---|---|
| 1 | Cost terms and weights (`tab:cost`) | definitional: `01_Main_Experiment/code/v2/controller_v2.py`, `code/v2/config_v2.py` | code shipped |
| 2 | Frozen configuration, corrected vs archived protocol (`tab:config`) | `code/v2/config_v2.py` (`config_id = 89fe865b2e1d`), `results/results_v2_stage5/matrix/_formal_batches.json` | shipped |
| 3 | Evaluation scenes (`tab:scenes`) | `code/v2/scenarios_v2.py` | code shipped |
| 4 | Compared methods, eight of them (`tab:methods`) | `results/results_v2_stage5/matrix/*.csv` (one file per method) | shipped |
| 5 | Evidence layers and denominators (`tab:layers`) | `results_v2_stage5/matrix` (2400), `02_Perception_Robustness/results/formal.jsonl` (2340), `03_Online_Warning_and_Timing/analysis/online_measurement/` (132 replays / 24 timing) | shipped |
| 6 | Corrected protocol, exact observation, 3000–3029 (`tab:corrected`) | `02_Perception_Robustness/results/formal.jsonl`, `version=safe` ∧ `factor=exact` — 3 methods × 60 runs | shipped |
| 7 | Archived nine-scenario comparison (`tab:main`) | `results_v2_stage5/matrix/*.csv` → `paper/tables/table_main9.csv`, `table_main10.csv` | shipped |
| 8 | Seed-clustered paired contrasts + Holm (`tab:contrasts`) | `paper/tables/table_contrasts.csv`, `table_contrasts10.csv`; statistics in `results_v2_stage5/reproduced/report_numbers.json` | shipped |
| 9 | Contact fraction under 11 observation conditions (`tab:perception`) | `02_Perception_Robustness/analysis/summary.csv` (from `formal.jsonl`) | shipped |
| 10 | Nominal decomposition for Ours, 3000–3029 (`tab:decomp`) | `02_Perception_Robustness/analysis/decomposition.csv` | shipped |
| 11 | Three-turn panel, 5000–5029 (`tab:curved`) | `05_Curved_Path_240/table_three_turn.csv` ← `detail.csv` (V2S5B rows) | **shipped** |
| 12 | Robot-loss panel, 1,080 runs (`tab:loss`) | `06_Robot_Loss_1080/table_failure.csv` ← `detail.csv` (Ours, 3 scenes × 4 conditions) | **shipped** |

Equations (8) and the single algorithm are defined in
`01_Main_Experiment/code/v2/controller_v2.py`, `code/v2/limits_v2.py`,
`code/v2/feasibility_v2.py` and `code/v2/proposed_rollout.py`; they contain no
fitted number that a data file could disagree with.

## The two formerly missing panels

* **Tables 11 / Figure 8** — the confirmatory three-turn panel (seeds
  5000–5029, V2S5A control + V2S5B with obstacle, 240 runs including the
  external `VH-Path` extension) is in `05_Curved_Path_240/`. Its key numbers
  are recomputed by `05_Curved_Path_240/reproduce_table11.py`.
* **Tables 12 / Figure 9** — the robot-loss panel (seeds 2000–2029, 3 methods
  × 3 scenes × 4 failure conditions, 1,080 runs) is in
  `06_Robot_Loss_1080/`. Its key numbers are recomputed by
  `06_Robot_Loss_1080/reproduce_table12.py`.

Both are separate analyses with their own denominators. They are not part of
the 2,400-run main matrix, the 2,340-cell robustness corpus or the online
measurement, and their runs must not be added to any of those counts.
