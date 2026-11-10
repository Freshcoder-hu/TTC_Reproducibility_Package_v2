"""V2 metrics: the V1 summary untouched, plus the Stage-1 additions.

``summarize_v2`` calls ``metrics.summarize`` (the frozen V1 implementation) and
merges the V2 fields, so every V1 column keeps its exact meaning and value while
the new columns are added beside it.
"""
from __future__ import annotations

from metrics import summarize as summarize_v1

# ---------------------------------------------------------------------------
V2_KEYS = [
    # parameters / provenance
    "config_id",
    # continuous contact detection: per pair class
    "min_surface_clearance_rr", "min_surface_clearance_ro",
    "min_surface_clearance_rs", "min_surface_clearance_ss",
    "min_surface_clearance_so",
    "any_contact_rr", "any_contact_ro", "any_contact_rs",
    "any_contact_ss", "any_contact_so",
    "n_contact_events_rr", "n_contact_events_ro", "n_contact_events_rs",
    "n_contact_events_ss", "n_contact_events_so",
    "n_preexisting_contact_rr", "n_preexisting_contact_ro",
    "n_preexisting_contact_rs", "n_preexisting_contact_ss",
    "n_preexisting_contact_so",
    "first_contact_time", "first_contact_class", "first_contact_pair",
    "first_contact_time_ro", "first_contact_pair_ro",
    "first_contact_time_rs", "first_contact_pair_rs",
    "first_contact_time_rr", "first_contact_pair_rr",
    # feasibility layer
    "n_infeasible", "n_infeasible_S", "n_infeasible_P",
    "n_infeasible_resolved", "n_infeasible_ro", "n_infeasible_rs",
    "n_infeasible_rr", "n_contact_C", "n_brake_events",
    # limit violations (state-based, per run)
    "violation_C1_low", "violation_C1_high", "violation_C2", "violation_C3",
    "violation_R1", "violation_R2", "violation_R3", "violation_R4",
    "violation_lat", "violation_R4_src", "violation_lat_src",
    # how often each bound had to clip (binding frequency)
    "bind_C1_low_sheep", "bind_C1_high_sheep", "bind_C2_sheep",
    "bind_C3_sheep", "bind_C3_turn_sheep", "bind_C3_speed_sheep",
    "bind_R1_robot", "bind_R2_robot", "bind_R3_robot", "bind_R4_robot",
    "bind_LAT_robot",
    "opp_sheep_steps", "opp_robot_steps",
    "LAT_UNRESOLVED", "LAT_JOINT_FALLBACK", "plant_infeasible",
    "n_guard_rr", "n_guard_ro",
    # V1-compatible extras that V2 exposes for the accounting table
    "dmin_robot_obs", "dmin_robot_robot", "dmin_sheep_obs",
    # ---- Stage 2: animal model ----
    "animal_model", "stim_mean", "stim_max", "n_sensed_mean", "sep_guard",
    "body_priority", "n_animal_steps", "n_animal_steps_sensing",
    "obs_delay_ticks", "delay_ticks_mean", "gain_mean", "gain_sd", "gain_min",
    "gain_max",
    # ---- Stage 2: controller layer ----
    "speed_policy", "sp_n_ticks", "sp_n_bounded_ticks", "sp_n_clamped_robots",
    "sp_r_req_mean", "sp_r_req_min", "sp_v_allow_mean", "sp_v_allow_min",
    "sp_speed_loss_sum", "predictor_engine", "predictor_model", "n_plans",
    "shadow_n",
]

# mapping counter -> metric name
_MAP = {
    "viol_C1_low": "violation_C1_low", "viol_C1_high": "violation_C1_high",
    "viol_C2": "violation_C2", "viol_C3": "violation_C3",
    "viol_R1": "violation_R1", "viol_R2": "violation_R2",
    "viol_R3": "violation_R3", "viol_R4": "violation_R4",
    "viol_LAT": "violation_lat",
    "guard_rr": "n_guard_rr", "guard_ro": "n_guard_ro",
}


def summarize_v2(res, v2cfg=None):
    m = summarize_v1(res)
    v2 = dict(res.get("v2") or {})
    for k in V2_KEYS:
        m.setdefault(k, v2.get(k))
    # counters -> metric names
    for src, dst in _MAP.items():
        m[dst] = v2.get(src, 0)
    m["violation_R4_src"] = v2.get("viol_R4", 0)
    m["violation_lat_src"] = v2.get("viol_LAT", 0)
    if v2cfg is not None:
        m["config_id"] = v2cfg.config_id
    # derived: binding rate over available agent-steps
    for b, opp in (("bind_C3_sheep", "opp_sheep_steps"),
                   ("bind_C2_sheep", "opp_sheep_steps"),
                   ("bind_C1_low_sheep", "opp_sheep_steps")):
        m.setdefault(f"rate_{b}", None)
        n = m.get(b) or 0
        d = m.get(opp) or 0
        m[f"rate_{b}"] = (float(n) / d) if d else None
    for b in ("bind_R1_robot", "bind_R2_robot", "bind_R3_robot",
              "bind_R4_robot", "bind_LAT_robot"):
        n = m.get(b) or 0
        d = m.get("opp_robot_steps") or 0
        m[f"rate_{b}"] = (float(n) / d) if d else None
    return m
