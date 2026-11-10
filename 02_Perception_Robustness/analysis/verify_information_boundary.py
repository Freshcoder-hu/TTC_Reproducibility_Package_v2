"""Counterfactual acceptance audit of the *frozen* online decision path.

This is a causal replay test, not an additional performance experiment. The
actual sensor sampler, publisher, controller, feasibility filter, RS filter and
robot integration code run unchanged. Sensor samples are replayed from a common
measurement world. At every tick the physical hidden sheep are intervened on;
robot states and pending commands are clamped to the reference replay before
checking downstream outputs. A truth-fed archive path is the positive control.
"""
from __future__ import annotations

import os
for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = "1"

import csv
import hashlib
import json
import sys
import time
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))
from run_robustness import config, source_hash
from v2 import sim_v2, fleet_v2
from v2.controller_v2 import make_controller, OursRolloutV2
from v2.feasibility_v2 import FeasibilityLayer
from v2.fleet_v2 import RobotFleetV2
from v2.robust_sensing import RobustSensorium
from v2.rs_safety import RSSafety
from v2.runner_s4 import ALGS
from v2.scenarios_v2 import cfg_for, resolve_scene
from obstacle import Obstacle
from sheep import init_flock

OUT = ROOT / "analysis" / "information_boundary"
VISIBLE = 8
STEPS = 30
DT = .05
PROFILES = {
    "partial_fresh": {},
    "partial_delayed_noisy": dict(sense_delay=.5, sense_animal_err=.2, sense_err=.2),
    "partial_blackout_expiry": dict(sense_blackout=.5, sense_blackout_start=.3,
                                    sense_blackout_period=1.),
    "empty": dict(sense_r_animal=.001),
}
VARIANTS = ("reference", "far_reposition", "near_intercept", "reverse_velocity")


def robot_state(robot):
    return np.concatenate((robot.pos.ravel(), robot.head, robot.speed, robot.omega))


def set_robot_state(robot, values):
    n = len(robot.pos)
    robot.pos[:] = values[:2*n].reshape(n, 2)
    robot.head[:] = values[2*n:3*n]
    robot.speed[:] = values[3*n:4*n]
    robot.omega[:] = values[4*n:5*n]


def array_digest(values):
    h = hashlib.sha256()
    for value in values:
        a = np.ascontiguousarray(value)
        h.update(str(a.shape).encode())
        h.update(a.dtype.str.encode())
        h.update(a.tobytes())
    return h.hexdigest()


def replay(scene, method, version, profile, variant, reference=None):
    sc = resolve_scene(scene)
    vc = replace(config(version, "exact"), c_diag_enabled=False,
                 **PROFILES[profile])
    cfg = cfg_for(sc, vc)
    center = sc["path"].point_at(sc["s0"])
    tangent = sc["path"].tangent_at(sc["s0"])
    normal = np.array([-tangent[1], tangent[0]])
    canonical = init_flock(cfg.n_sheep, center, radius=2.8,
                           rng=np.random.default_rng(9311))
    # Only the first eight sheep have measurement tracks; the others are
    # outside all measurement-world detection discs.
    canonical.pos[VISIBLE:] = center - 200*tangent + np.arange(
        cfg.n_sheep - VISIBLE)[:, None] * .1 * normal
    canonical.head[:] = np.arctan2(tangent[1], tangent[0])
    canonical.speed[:] = 1.
    canonical_pos = canonical.pos.copy()
    canonical_vel = canonical.vel.copy()

    # Fixed and identical initial robot state in every counterfactual world.
    a = np.arange(cfg.n_robots) * (2*np.pi/cfg.n_robots)
    initial_pos = center + 5*np.stack((np.cos(a), np.sin(a)), axis=1)
    initial_head = np.full(cfg.n_robots, np.arctan2(tangent[1], tangent[0]))
    initial = np.concatenate((initial_pos.ravel(), initial_head,
                              np.full(cfg.n_robots, 2.), np.zeros(cfg.n_robots)))
    trace = {k: [] for k in ("robot_before", "robot_after", "pending",
                              "feasibility", "rs_safety", "execution", "prediction")}
    controller_commands = []
    observations = []
    counts = []
    seen_ages = []
    tags = []
    truth_hashes = []
    predictor_calls = []
    hidden_min = []
    actual_world_digests = []
    holder = {}
    feasibility_changes = []
    rs_changes = []

    def init_robot(*args, **kwargs):
        robot = RobotFleetV2(initial_pos.copy(), cfg=cfg)
        set_robot_state(robot, initial)
        holder["robot"] = robot
        return robot

    original_predict = OursRolloutV2._predict
    def predict(ctrl, view, sim_cfg, offsets):
        predictor_calls.append(float(view.t))
        out = original_predict(ctrl, view, sim_cfg, offsets)
        trace["prediction"].append(out.copy())
        return out

    class ReplaySensor(RobustSensorium):
        def __init__(self):
            super().__init__(vc, cfg, seed=9311)
            self.phase = 0.
            self.tick = 0

        def update(self, t, step, ctrl_tick, robot_pos, obs, sheep_pos, sheep_vel):
            self.tick = int(step)
            robot = holder["robot"]
            if reference is not None:
                set_robot_state(robot, reference["robot_before"][step])
            # The experiment fixes the sensor stream, including sensor RNG,
            # independently of the physical hidden-state intervention.
            obstacle = Obstacle(center + (9. - 1.5*t)*tangent,
                                -1.5*tangent, vc.r_obs, 100., 0., 0.)
            obstacle.pos = center + (9. - 1.5*t)*tangent
            obstacle.active = True
            obstacle.t0_active = 0.
            super().update(t, step, ctrl_tick, robot.pos, [obstacle],
                           canonical_pos + t*canonical_vel, canonical_vel)

        def publish(self, truth, t):
            robot = holder["robot"]
            n_hidden = cfg.n_sheep - VISIBLE
            physical = truth.sheep
            physical.pos[:] = canonical_pos + t*canonical_vel
            physical.head[:] = canonical.head
            physical.speed[:] = canonical.speed
            if variant == "far_reposition":
                physical.pos[VISIBLE:] = center - 300*tangent + 30*normal
                physical.head[VISIBLE:] += .8
                physical.speed[VISIBLE:] = .3
            elif variant == "near_intercept":
                idx = np.arange(n_hidden) % cfg.n_robots
                physical.pos[VISIBLE:] = robot.pos[idx] + .81*tangent
                physical.head[VISIBLE:] += np.pi
                physical.speed[VISIBLE:] = 1.5
            elif variant == "reverse_velocity":
                # Same hidden positions as reference, different velocities.
                physical.head[VISIBLE:] += np.pi
                physical.speed[VISIBLE:] = 1.5
            truth.sheep_pos_global = physical.pos.copy()
            truth.sheep_vel_global = physical.vel.copy()
            truth.robot_pos = robot.pos.copy()
            truth_hashes.append(array_digest((physical.pos[VISIBLE:],
                                             physical.vel[VISIBLE:])))
            actual_world_digests.append(array_digest((physical.pos, physical.vel)))
            hidden_min.append(float(np.linalg.norm(
                physical.pos[VISIBLE:, None] - robot.pos[None], axis=2).min()))
            v = super().publish(truth, t)
            assert not hasattr(v, "sheep_pos_global")
            assert not hasattr(v, "sheep_vel_global")
            assert not hasattr(v, "sheep")
            arrays = [v.robot_pos, v.sheep_pos, v.sheep_vel, v.sheep_age,
                      v.obs_pos, v.obs_vel, v.obs_age,
                      v.mask_direct, v.mask_shared]
            arrays.extend(np.array([o.index, o.radius, o.t_stamp, o.age,
                                    o.source, o.n_updates])
                          for o in v.obstacles_active)
            observations.append(array_digest(arrays))
            counts.append(len(v.sheep_pos))
            seen_ages.extend(v.sheep_age.tolist())
            return v

    original_step = RobotFleetV2.step
    original_feas = FeasibilityLayer.filter
    original_rs = RSSafety.filter
    original_project = fleet_v2.robot_step_project

    def feas(layer, cmd, st, ctrl_tick=True):
        out = original_feas(layer, cmd, st, ctrl_tick=ctrl_tick)
        trace["feasibility"].append(out.copy())
        feasibility_changes.append(int(np.any(out != cmd)))
        return out

    def rs(layer, cmd, *args):
        out = original_rs(layer, cmd, *args)
        trace["rs_safety"].append(out.copy())
        rs_changes.append(int(np.any(out != cmd)))
        return out

    def project(cmd, *args, **kwargs):
        holder["execution"].append(np.asarray(cmd).copy())
        return original_project(cmd, *args, **kwargs)

    def step(robot, cmd, *args, **kwargs):
        i = len(trace["pending"])
        trace["robot_before"].append(robot_state(robot).copy())
        # Isolate the lower layer even if an upstream regression were found.
        if reference is not None:
            cmd = reference["pending"][i].copy()
        trace["pending"].append(cmd.copy())
        holder["execution"] = []
        out = original_step(robot, cmd, *args, **kwargs)
        trace["execution"].append(np.array(holder["execution"]))
        trace["robot_after"].append(robot_state(robot).copy())
        return out

    ctrl = make_controller(method, ALGS, vc, dict(sc.get("params", {})))
    ctrl_step = ctrl.step
    def controller_step(view, sim_cfg):
        cmd, tag = ctrl_step(view, sim_cfg)
        controller_commands.append(cmd.copy())
        tags.append(tag)
        return cmd, tag
    ctrl.step = controller_step

    sensor = ReplaySensor()
    with patch.object(sim_v2, "_init_robots_ring_v2", init_robot), \
         patch.object(sim_v2, "build_sensing", lambda *args: sensor), \
         patch.object(RobotFleetV2, "step", step), \
         patch.object(FeasibilityLayer, "filter", feas), \
         patch.object(RSSafety, "filter", rs), \
         patch.object(fleet_v2, "robot_step_project", project), \
         patch.object(OursRolloutV2, "_predict", predict):
        result = sim_v2.run_scene_v2(cfg, sc["path"], [], ctrl, seed=9311,
                    T_max=STEPS*DT-1e-8, s0=sc["s0"], v2cfg=vc,
                    flock_init=lambda *args: init_flock(
                        cfg.n_sheep, center, rng=np.random.default_rng(9311)))
    assert len(trace["execution"]) == STEPS, result["terminated_by"]
    trace = {k: np.array(v) for k, v in trace.items()}
    trace["controller"] = np.array(controller_commands)
    metadata = dict(scene=scene, method=method, version=version, profile=profile,
                    variant=variant, steps=STEPS, control_calls=len(tags), tags=tags,
                    observed_counts=counts, observation_sha256=observations,
                    hidden_truth_sha256=truth_hashes,
                    plant_truth_sha256=actual_world_digests,
                    hidden_min_robot_distance=min(hidden_min),
                    max_observed_age=max(seen_ages, default=0.),
                    predictor_calls=len(predictor_calls),
                    feasibility_changed_ticks=sum(feasibility_changes),
                    rs_changed_control_ticks=sum(rs_changes),
                    empty_stop_ticks=result["v2"].get("rs_empty_stop_ticks", 0),
                    truth_changes_are_test_interventions=True)
    return trace, metadata


def compare(a, b):
    out = {}
    for key in a:
        assert a[key].shape == b[key].shape, (key, a[key].shape, b[key].shape)
        delta = float(np.max(np.abs(a[key]-b[key]))) if a[key].size else 0.
        out[key + "_max_abs_delta"] = delta
        out[key + "_exact_equal"] = bool(np.array_equal(a[key], b[key]))
    return out


def main():
    started = time.time()
    OUT.mkdir(exist_ok=True)
    frozen = json.loads((ROOT/"results/formal.manifest.json").read_text())["source_sha256"]
    before = source_hash()
    assert before == frozen, "Frozen experiment source has changed"
    pairs, metadata, arrays = [], [], {}
    for scene in ("V2S2R2", "V2S3"):
        for method in ("Ours_rollout", "B1", "B2"):
            for version in ("strict", "safe"):
                for profile in PROFILES:
                    reference, ref_meta = replay(scene, method, version, profile, "reference")
                    prefix = "__".join((scene, method, version, profile))
                    metadata.append(ref_meta)
                    for key, value in reference.items():
                        arrays[prefix + "__reference__" + key] = value
                    for variant in VARIANTS[1:]:
                        trace, meta = replay(scene, method, version, profile, variant, reference)
                        differences = compare(reference, trace)
                        same_observations = ref_meta["observation_sha256"] == meta["observation_sha256"]
                        changed_hidden = all(x != y for x, y in zip(
                            ref_meta["hidden_truth_sha256"], meta["hidden_truth_sha256"]))
                        tags_equal = ref_meta["tags"] == meta["tags"]
                        passed = same_observations and changed_hidden and tags_equal and all(
                            value for key, value in differences.items() if key.endswith("exact_equal"))
                        pairs.append(dict(scene=scene, method=method, version=version,
                            profile=profile, variant=variant, same_observations=same_observations,
                            hidden_truth_changed_every_tick=changed_hidden, tags_equal=tags_equal,
                            passed=passed, **differences))
                        metadata.append(meta)
                        for key, value in trace.items():
                            arrays[prefix + "__" + variant + "__" + key] = value
                    print("CHECKED", prefix, "pairs", len(pairs), flush=True)

    # Positive control uses the actual archive dispatch: truth is passed to
    # FeasibilityLayer. It must expose command sensitivity, otherwise a silent
    # test harness could give a misleading all-pass result.
    ref, ref_meta = replay("V2S3", "B1", "archive", "partial_fresh", "reference")
    alt, alt_meta = replay("V2S3", "B1", "archive", "partial_fresh", "near_intercept", ref)
    positive = compare(ref, alt)
    positive["same_observations"] = ref_meta["observation_sha256"] == alt_meta["observation_sha256"]
    positive["detected_truth_sensitive_execution"] = not positive["execution_exact_equal"]
    for label, trace in (("reference", ref), ("near_intercept", alt)):
        for key, value in trace.items():
            arrays["positive_archive__"+label+"__"+key] = value
    metadata.extend((ref_meta, alt_meta))

    after = source_hash()
    native_calls = sum(m["predictor_calls"] for m in metadata)
    safe_changes = sum(m["rs_changed_control_ticks"] for m in metadata if m["version"]=="safe")
    summary = dict(status="pass" if all(p["passed"] for p in pairs) and
        positive["detected_truth_sensitive_execution"] and before == after and
        native_calls > 0 and safe_changes > 0 else "FAIL",
        audit_kind="post_evaluation_counterfactual_acceptance_not_new_performance_runs",
        frozen_source_sha256=frozen, current_source_sha256=after,
        source_unchanged=before == after, strict_safe_pairs=len(pairs),
        strict_safe_world_replays=len(pairs)//3*4, steps_per_replay=STEPS,
        hidden_variants=list(VARIANTS[1:]),
        comparison_rule="bit-exact np.array_equal, no numerical tolerance",
        failed_pairs=sum(not p["passed"] for p in pairs),
        max_execution_abs_delta=max(p["execution_max_abs_delta"] for p in pairs),
        positive_archive_control=positive,
        total_native_rollout_predictions=native_calls,
        safe_supervisor_changed_ticks=safe_changes,
        wall_s=time.time()-started,
        scope="2 scenes x 3 controllers x 2 repaired execution versions x 4 observation profiles x 3 hidden interventions; fixed population 36, 12 robots",
        limits=["Finite regression evidence, not a universal noninterference proof",
                "Fixed sensor-stream and hidden-state interventions are causal test fixtures, not physical trial trajectories",
                "Robot states and pending lower-layer commands are reset to the reference at each tick; actual outputs are checked before reset",
                "No claim that stopping prevents all contacts with unobserved animals",
                "This audit was added after the 2340-run evaluation; it was not a pre-evaluation gate"])
    (OUT/"summary.json").write_text(json.dumps(summary, indent=2))
    (OUT/"replays.json").write_text(json.dumps(metadata, indent=2))
    with (OUT/"paired_checks.csv").open("w", newline="") as f:
        writer=csv.DictWriter(f, fieldnames=list(pairs[0]))
        writer.writeheader(); writer.writerows(pairs)
    np.savez_compressed(OUT/"online_traces.npz", **arrays)
    print(json.dumps(summary, indent=2), flush=True)
    assert summary["status"] == "pass", summary


if __name__ == "__main__":
    main()
