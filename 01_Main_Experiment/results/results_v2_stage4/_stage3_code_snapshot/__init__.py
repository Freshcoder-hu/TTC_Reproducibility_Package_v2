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

Stage-2 additions
-----------------
animal_models    M1/M2 desired velocity, delay buffers, per-animal gains
corridor_v2      no-robot single-obstacle corridor test bed
calibrate_v2     M1/M2 calibration grids and selection rule
controller_v2    OursRolloutV2 / SpeedPolicyController / ShadowPredictor
speed_policy_v2  r_req derivation and the layered speed bound
predictor_v2     NumPy rollout (M0/M1/M2) for the decoupling proof
acceptance_s2    Stage-2 tests A-F
capture_reference  Stage-2 whole-trajectory reference capture
plots_s2         Stage-2 figures
speed_policy_ab  speed-policy A/B on S3

Stage-3 additions
-----------------
sensing_v2        two-channel sensing interface (25 m / 70 m), tracks,
                  delivery delay, loss, position error, per-tick assertions
observability_v2  observable-warning-time probe (speed x geometry x seeds)
calibrate_ttc_v2  TTC tuple derivation (rules R1-R4), required range, freeze
capture_reference_s3  Stage-2 reference capture for the Stage-3 inertness test
acceptance_s3     Stage-3 tests A-F
plots_s3          Stage-3 figures
regress_s2_readonly  re-run the Stage-2 suite without writing into its archive
"""

__all__ = ["config_v2", "limits_v2", "animal_m0", "robot_safety_v2", "fleet_v2",
           "feasibility_v2", "contacts_v2", "sim_v2", "metrics_v2",
           "runner_v2"]
