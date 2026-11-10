TTC shepherding: perception robustness and observation-only safety supplement

Read Information_Boundary_Acceptance_ZH.txt first for the 2026-10-29 addendum.
It reports 144 passing counterfactual pairs, with a truth-sensitive archive
positive control. This stronger audit was added AFTER evaluation. The frozen
implementation and formal performance data are unchanged. Physical safety
tradeoffs remain; false-negative rates, actual delivered-warning lead and full
online-pipeline timing have not yet met the expanded acceptance requirements.

This package is a new simulation supplement. It does not overwrite V7, and does
not constitute physical robot or real-animal validation.

CODE
  code/ includes the submitted simulator plus opt-in strict sensing, an
  observation-only finite-candidate safety supervisor, and a resumable runner.
  Defaults keep the archived trajectory computation path. New config hashes
  differ because new fields were added. M1 plant / M0 native predictor retained.

BUILD (Linux, Python 3.12, NumPy 2.3.5, SciPy 1.17.0)
  cd code
  g++ -O3 -std=c++17 -shared -fPIC rollout.cpp -o _rollout.so
  g++ -O3 -ffp-contract=off -std=c++17 -shared -fPIC rs_safety.cpp -o _rs_safety.so
  export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
  cd ..
  python3 analysis/validate_mechanisms.py
  python3 analysis/verify_information_boundary.py
  python3 analysis/build_boundary_addendum.py
  python3 code/run_robustness.py --out results/reproduced.jsonl --include-decomposition --jobs 9

The default seeds are 3000..3029, across two scenes, three controllers, eleven
sensing conditions. Main matrix: 1980 cells. Additional archive/strict exact
observation comparisons: 360 cells. Total final evaluation: 2340 cells.
Use --out results/formal.jsonl to resume only with identical source/binary hash.
Use a NEW output path if compilation changes the binaries.

ANALYSIS
  python3 analysis/analyze.py
  python3 analysis/build_report.py
The analyzer validates a complete formal.jsonl and refuses missing/duplicate
cells or source drift. If reproducing into another path, copy the new JSONL and
its manifest to formal.jsonl/formal.manifest.json in a fresh package copy first.
Tables are UTF-8 CSV; undefined metrics in the raw Python JSONL may use NaN or
Infinity. statistics.json converts these to null for strict JSON consumers.

DESIGN
  Read protocol.json for definitions, predeclared tests, assumptions and limits.
  The outage factors are periodic shared animal-channel interruptions, not a
  rendered geometric occlusion model. Position sigma is per Cartesian axis.
  Velocities, identities and sizes remain exact or delayed. All robots share
  observations. The safety layer is given the injected position-noise sigma.

FAILURES AND TRANSPARENCY
  formal_attempt1.jsonl preserves the first stopped batch, including the old
  post-hoc diagnostic's IndexError under empty observations. It is excluded from
  inference because the entire matrix was rerun under the corrected revision.
  regression_blackout.jsonl reruns that exact failure cell successfully.
  pilot_v1.jsonl contains 36 independent implementation checks (seeds 1100-1101).
  The strict/safe versions disable only that post-hoc avoidability classifier;
  the physical swept contact monitor and all contact endpoints remain active.
  No evaluation outcome was used to retune the controller or safety parameters.

No contact-free deployment claim is made. The supervisor searches a finite
candidate set with a limited motion forecast. All fallback/overlap/failure counts
are reported alongside task completion, including adverse outcomes.

TRANSPORT RECOVERY
The execution transport disconnected after more than 1200 records. The original
writer stopped at 1219. formal_epoch1.jsonl preserves that checkpoint; its rows
are already INCLUDED in formal.jsonl, so DO NOT concatenate the two files. The
unchanged runner resumed only missing cell keys. formal.jsonl is the sole final
evaluation dataset (2340 unique cells). Regression and dense replay files repeat
selected cells for verification and do not add independent samples.
