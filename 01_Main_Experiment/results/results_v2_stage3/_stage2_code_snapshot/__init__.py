"""V2 Stage 1 -- physical execution layer for the TTC-shepherding simulator.

Nothing in this package imports into, patches, or mutates the frozen V1 tree at
import time (``corrections.install`` is applied by the runners exactly as the V1
runner does).  See ``V2_STAGE1_REPORT.md`` in the code root.

Module map
----------
config_v2        parameters, V1 reference values, content hash
limits_v2        the (V2) box and R1-R4 + lateral joint projections
animal_m0        M0 desired velocity (V1 transcription) -- Stage-2 hook
robot_safety_v2  soft spacing target + hard-geometry guard
fleet_v2         RobotFleetV2 (turn rate as state, 4-phase step)
feasibility_v2   braking supervisor, prediction, escape, S/P/C classification
contacts_v2      swept contact detector and per-run monitor
sim_v2           run_scene_v2 (V1 loop + V2 hooks)
metrics_v2       summarize_v2 (V1 columns untouched)
runner_v2        single cell / batch / --jobs / --compat / --set
acceptance_v2    acceptance tests A-I
unit_tests_v2    transcription and projection unit tests
compare_v1_v2    V1 vs V2 comparison matrix
diag_v2          design-variant diagnostics
probe_v1_repro   V1 archive reproduction probe
"""

__all__ = ["config_v2", "limits_v2", "animal_m0", "robot_safety_v2", "fleet_v2",
           "feasibility_v2", "contacts_v2", "sim_v2", "metrics_v2",
           "runner_v2"]
