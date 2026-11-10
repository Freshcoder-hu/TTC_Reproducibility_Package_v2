"""Stage 5 -- formal 8-method comparison, ablation ladder, seed-clustered stats.

Everything in this sub-package is **new Stage-5 code**.  Nothing in it modifies
the frozen V1 tree, the Stage-1..4 V2 modules, or any pre-existing ``results_*``
directory.  The Stage-5 runner *reuses* ``v2.runner_s4.run_cell`` (which in turn
reuses the accepted Stage-2/3 stack) and only adds columns, so a Stage-4 cell
re-run through it must reproduce the Stage-4 row bit-for-bit (acceptance A of
the Stage-5 pass).

Modules
-------
``common``      the frozen Stage-5 configuration: methods, scenes, seeds,
                endpoint definitions, paths, hashing helpers.
``preempt``     the ``preempt_ok`` fix (item 1): preemption from the
                **controller's information set** (visibility-gated TTC) plus a
                truth-based variant, RECOVER-window bookkeeping.
``runs``        one Stage-5 cell: ``run_cell`` + the Stage-5 columns.
``matrix``      the formal 8 x 10 x 30 matrix, checkpointed per method.
``ablation``    the four-rung ladder on V2S2R2 / V2S3 x 30 seeds.
``trec_sweep``  the ``T_REC`` re-validation at 220 m (item 2).
``c1_probe``    the C1 audit (item 3): structural proof + a documented stress
                case in which C1-low does bind.
``gated_probe`` the equivalence proof behind ``preempt`` (masked truth TTC ==
                TTC of the published gated view).
``checks``      acceptance A (inertness), B (counts), C (determinism).
``stats``       seed-clustered Wilcoxon/Holm family, run-level descriptive
                family, exact McNemar for the binary endpoints.
``tables``      every table/number that appears in ``V2_STAGE5_REPORT.md``.
"""
from __future__ import annotations

VERSION = "stage5-v1"
