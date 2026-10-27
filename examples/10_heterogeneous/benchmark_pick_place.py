# SPDX-License-Identifier: Apache-2.0
"""Headless Clawhauser timing and focused action/reset/replication checks.

Run on an allocated CUDA node, in the same environment as clawhauser_pick_place.
The matched timing workload holds the initial finger targets; it measures backend
cost, not pickup success. Use the full example's diagnostics for task acceptance.
"""

from __future__ import annotations

import argparse
import gc
import json
import time
from pathlib import Path

import numpy as np
import warp as wp
from clawhauser_pick_place import CONTROL_HZ, OUTPUT, SUBSTEPS, build_scene


def _timed(sim, operation):
    sim.sync()
    start = time.perf_counter()
    result = operation()
    sim.sync()
    return time.perf_counter() - start, result


def _graph(pipeline):
    return {
        "requested": bool(pipeline.use_cuda_graph),
        "captured": bool(pipeline.captured),
        "capture_error": pipeline.capture_error,
        "steps_replayed": int(pipeline.steps_replayed),
        "steps_eager": int(pipeline.steps_eager),
    }


def _memory(device):
    result = {"device_used_bytes_snapshot": int(device.total_memory - device.free_memory)}
    if device.is_cuda and device.is_mempool_supported:
        result["warp_pool_current_bytes"] = int(wp.get_mempool_used_mem_current(device))
        result["warp_pool_high_bytes_process_cumulative"] = int(wp.get_mempool_used_mem_high(device))
    return result


def _indices(entity, kind, indices):
    worlds = entity.worlds
    if worlds is None:
        return np.asarray([indices], dtype=np.int32)
    return getattr(worlds, f"spread_{kind}")(indices)


def _joint_rows(model, joints, start_name):
    """Expand owned joints using the public coordinate/DOF start tables."""
    starts = getattr(model, start_name).numpy()
    return np.asarray(
        [np.concatenate([np.arange(starts[j], starts[j + 1]) for j in row]) for row in joints],
        dtype=np.int32,
    )


def _snapshot(sim, obj, grip):
    """All owned worlds, with template indices mapped through WorldMap."""
    sim.sync()
    state = sim.pipeline.state_0
    particles = _indices(obj, "particles", obj.particles)
    bodies = _indices(grip, "bodies", getattr(grip, "all_bodies", grip.bodies))
    joints = _indices(grip, "joints", np.arange(*grip.range.joints, dtype=np.int32))
    coords = _joint_rows(sim.model, joints, "joint_q_start")
    dofs = _joint_rows(sim.model, joints, "joint_qd_start")
    target_coords = _joint_rows(sim.model, joints, "joint_target_q_start")
    snapshot = {
        "particle_q": state.particle_q.numpy()[particles].copy(),
        "particle_qd": state.particle_qd.numpy()[particles].copy(),
        "body_q": state.body_q.numpy()[bodies].copy(),
        "body_qd": state.body_qd.numpy()[bodies].copy(),
        "target_poses": grip.target_poses.numpy().copy(),
        "target_velocities": grip.target_velocities.numpy().copy(),
        "joint_q": state.joint_q.numpy()[coords].copy(),
        "joint_qd": state.joint_qd.numpy()[dofs].copy(),
    }
    # Reset writes both ping-pong buffers; observe both rather than relying on
    # the next solve to hide a stale destination pose or velocity.
    for index, buffer in enumerate((sim.pipeline.state_0, sim.pipeline.state_1)):
        for name, rows in (
            ("particle_q", particles),
            ("particle_qd", particles),
            ("particle_f", particles),
            ("body_q", bodies),
            ("body_qd", bodies),
            ("body_f", bodies),
        ):
            snapshot[f"{name}_buffer{index}"] = getattr(buffer, name).numpy()[rows].copy()
    for name, indices in (("joint_target_q", target_coords), ("joint_target_qd", dofs), ("joint_f", dofs)):
        values = getattr(sim.pipeline.control, name, None)
        if values is not None:
            snapshot[name] = values.numpy()[indices].copy()
    if hasattr(grip, "applied_carriage_wrench"):
        for name in (
            "actual_half_gaps",
            "actual_gap_rates",
            "slide_lateral_error",
            "slide_angular_error",
            "applied_carriage_wrench",
            "peak_carriage_force",
            "peak_carriage_torque",
            "peak_motor_effort",
            "motor_efforts_available",
        ):
            snapshot[name] = getattr(grip, name).numpy().copy()
        available = snapshot["motor_efforts_available"].astype(bool)
        # Reset intentionally marks effort unavailable with NaN. Keep that
        # validity flag explicit while comparing only valid numeric samples.
        snapshot["motor_efforts"] = np.where(available[:, None], grip.motor_efforts.numpy(), 0.0)
        if grip.spec.motor_backend == "vbd":
            snapshot["applied_jaw_joint_effort"] = sim.pipeline.control.joint_f.numpy()[
                grip.finger_dofs_by_world
            ].copy()
    return snapshot


def _actuator_check(grip, snapshot, *, atol):
    if not hasattr(grip, "applied_carriage_wrench"):
        return False
    if grip.motor_effort_kind == "native_QFRC_ACTUATOR" and not np.any(snapshot["motor_efforts_available"]):
        # Bare coupled timing does not install native motor observables.
        return False
    if not np.all(snapshot["motor_efforts_available"] == 1):
        raise AssertionError("mechanical jaw effort was not observed after stepping")
    limits = (
        (snapshot["motor_efforts"], grip.spec.jaw_force_limit),
        (snapshot["peak_motor_effort"], grip.spec.jaw_force_limit),
        (snapshot["peak_carriage_force"], grip.spec.carriage_force_limit),
        (snapshot["peak_carriage_torque"], grip.spec.carriage_torque_limit),
    )
    for values, limit in limits:
        if not np.isfinite(values).all() or np.any(np.abs(values) > limit + atol):
            raise AssertionError(f"applied actuator effort exceeds {limit}: {values}")
    wrench = snapshot["applied_carriage_wrench"]
    for values, limit in (
        (wrench[..., :3], grip.spec.carriage_force_limit),
        (wrench[..., 3:], grip.spec.carriage_torque_limit),
    ):
        if not np.isfinite(values).all() or np.any(np.linalg.norm(values, axis=-1) > limit + atol):
            raise AssertionError(f"applied carriage wrench exceeds {limit}: {values}")
    if "applied_jaw_joint_effort" in snapshot:
        np.testing.assert_array_equal(snapshot["motor_efforts"], snapshot["applied_jaw_joint_effort"])
    return True


def _owned_observation(grip, snapshot):
    """Actual batched owned state; separate from the embodiment-free Sim obs."""
    names = (
        "body_q",
        "body_qd",
        "joint_q",
        "joint_qd",
        "target_poses",
        "target_velocities",
        "actual_half_gaps",
        "actual_gap_rates",
        "motor_efforts",
        "motor_efforts_available",
        "applied_carriage_wrench",
    )
    return {
        "motor_effort_kind": getattr(grip, "motor_effort_kind", None),
        "unavailable_motor_efforts": "numeric zero placeholders; validity is motor_efforts_available",
        **{name: snapshot[name].tolist() for name in names if name in snapshot},
    }


def _compare(actual, expected, *, atol, world=None):
    errors = {}
    for name, reference in expected.items():
        value = actual[name]
        if world is not None:
            value = value[world]
            compared = reference[world]
        else:
            compared = reference
        if not np.isfinite(value).all() or not np.allclose(value, compared, rtol=0.0, atol=atol):
            errors[name] = float(np.max(np.abs(value - compared)))
    if errors:
        raise AssertionError(f"state mismatch (absolute tolerance {atol}): {errors}")


def _hold_targets(grip):
    poses = grip.initial_poses.numpy().copy()
    for world, targets in enumerate(poses):
        grip.set_targets(targets, world=world)
    return poses


def core_check(sim, obj, grip, *, steps=12, atol=1.0e-5, resetter=None):
    """Check replay and a masked pure-scene reset against a next-step control."""
    checks = ["all_world_reset", "reset_replay"]
    sim.reset()
    initial = _snapshot(sim, obj, grip)
    poses = grip.initial_poses.numpy().copy()
    commands = poses.copy()
    commands[:, :, 2] += 0.02
    commands[:, :, 0] += np.where(np.arange(sim.num_worlds) % 2 == 0, 0.012, -0.012)[:, None]
    # Exercise aperture inputs too, without changing the shared carrier pose.
    separation = poses[:, 1, :3] - poses[:, 0, :3]
    closing = separation / np.linalg.norm(separation, axis=1)[:, None]
    commands[:, 0, :3] += 0.004 * closing
    commands[:, 1, :3] -= 0.004 * closing

    def replay():
        for world, targets in enumerate(commands):
            grip.set_targets(targets, world=world)
        for _ in range(steps):
            sim.step()
        return _snapshot(sim, obj, grip)

    first = replay()
    actuator_caps_checked = _actuator_check(grip, first, atol=atol)
    displacement = (first["body_q"][:, :, :3] - initial["body_q"][:, :, :3]).mean(axis=1)
    if sim.num_worlds >= 2:
        signs = np.where(np.arange(sim.num_worlds) % 2 == 0, 1.0, -1.0)
        if not np.all(displacement[:, 0] * signs > 1.0e-4):
            raise AssertionError(f"different world commands did not give opposite x responses: {displacement}")
        checks.append("different_world_actions")
    sim.step()
    unreset_next = _snapshot(sim, obj, grip)
    sim.reset()
    _compare(_snapshot(sim, obj, grip), initial, atol=atol)
    second = replay()
    _compare(second, first, atol=atol)
    _actuator_check(grip, second, atol=atol)

    # The coupled mechanism needs a full scene reset; do not extend the pure
    # scene's targeted-reset claim to coupled solver history or proxy state.
    if not sim.pipeline.program.coupled:
        if resetter is None:
            from reset_pick_place import prepare_reset

            resetter = prepare_reset(sim, obj, grip)
        before = _snapshot(sim, obj, grip)
        clock = (sim.pipeline.sim_time, sim.step_count)
        resetter.reset(0)
        after = _snapshot(sim, obj, grip)
        if (sim.pipeline.sim_time, sim.step_count) != clock:
            raise AssertionError("masked reset changed the global simulation clock")
        for world in range(1, sim.num_worlds):
            _compare(after, before, atol=0.0, world=world)
        _compare(after, initial, atol=atol, world=0)
        sim.step()
        partial_next = _snapshot(sim, obj, grip)
        for world in range(1, sim.num_worlds):
            _compare(partial_next, unreset_next, atol=atol, world=world)
        checks.extend(("toy_mechanism_masked_reset", "neighbor_unchanged_immediately"))
        if sim.num_worlds >= 2:
            checks.append("neighbor_next_step_counterfactual")
    sim.reset()
    _compare(_snapshot(sim, obj, grip), initial, atol=atol)
    if not sim.pipeline.program.coupled:
        sim.step()
        _compare(partial_next, _snapshot(sim, obj, grip), atol=atol, world=0)
        checks.append("reset_world_next_step_matches_fresh_reset")
        sim.reset()
        _compare(_snapshot(sim, obj, grip), initial, atol=atol)
    return {
        "passed": True,
        "steps": steps,
        "worlds": sim.num_worlds,
        "absolute_tolerance": atol,
        "world_mean_displacement_m": displacement.tolist(),
        "checks": checks,
        "masked_reset_skipped_for_coupled_scene": bool(sim.pipeline.program.coupled),
        "partial_object_or_solver_reset_tested": not sim.pipeline.program.coupled,
        "actuator_effort_caps_validated": actuator_caps_checked,
        "neighbor_next_step_tolerance_note": "absolute tolerance accommodates unordered native floating-point reductions",
        "owned_state_observation": _owned_observation(grip, first),
    }


def benchmark(config, *, worlds, steps, warm_steps=2, check_core=False, core_steps=12, record_contacts=False):
    # Both paths use identical fixed targets and physics. Reissuing those targets
    # every control step measures the public policy-input cost without changing
    # contact workload between paths. This intentionally disables the demo script.
    effective = {"graph": True, "substeps": SUBSTEPS, "iterations": 12, "mode": "dynamic", **config, "scripted": False}
    start = time.perf_counter()
    sim, obj, grip, _, _, labels = build_scene(effective, num_worlds=worlds)
    sim.sync()
    build_s = time.perf_counter() - start
    obj, grip = obj.entity, grip.entity
    try:
        resetter = None
        if check_core and not sim.pipeline.program.coupled:
            from reset_pick_place import prepare_reset

            resetter = prepare_reset(sim, obj, grip)
        recording_setup_s = 0.0
        if record_contacts:
            from contact_recording import prepare_contact_recording

            recording_setup_s, _ = _timed(sim, lambda: prepare_contact_recording(sim, obj.spec.name, grip))
        first_reset_s, _ = _timed(sim, sim.reset)
        targets = _hold_targets(grip)
        for _ in range(warm_steps):
            sim.step()
        sim.sync()
        memory_warmed = _memory(sim.model.device)
        reset_s, _ = _timed(sim, sim.reset)
        _hold_targets(grip)
        pipeline = sim.pipeline
        graph_before = _graph(pipeline)
        clock_before = float(pipeline.sim_time)
        readback = pipeline.readback
        try:
            pipeline.readback = None

            def physics():
                for _ in range(steps):
                    pipeline.step()

            physics_s, _ = _timed(sim, physics)
        finally:
            pipeline.readback = readback
            pipeline.mark_dirty()
        physics_sim_s = float(pipeline.sim_time) - clock_before
        graph_physics = _graph(pipeline)
        physics_finite = all(np.isfinite(value).all() for value in _snapshot(sim, obj, grip).values())
        sim.reset()
        _hold_targets(grip)
        clock_before = float(pipeline.sim_time)

        def action_observe():
            last = None
            for _ in range(steps):
                for world, pose in enumerate(targets):
                    grip.set_targets(pose, world=world)
                last = sim.step()
            return last

        action_s, last_step = _timed(sim, action_observe)
        action_sim_s = float(pipeline.sim_time) - clock_before
        graph_action = _graph(pipeline)
        final_snapshot = _snapshot(sim, obj, grip)
        finite = all(np.isfinite(value).all() for value in final_snapshot.values())

        def rates(elapsed, simulated):
            return {
                "wall_s": elapsed,
                "simulated_s_per_world": simulated,
                "aggregate_simulated_s_per_wall_s": worlds * simulated / elapsed,
                "aggregate_control_steps_per_wall_s": worlds * steps / elapsed,
                "batch_step_latency_s": elapsed / steps,
                "amortized_world_step_s": elapsed / (steps * worlds),
                "rollouts_per_wall_s_at_measured_horizon": worlds / elapsed,
            }

        row = {
            "worlds": worlds,
            "steps": steps,
            "warm_steps": warm_steps,
            "control_hz": CONTROL_HZ,
            "device": str(sim.model.device),
            "device_name": sim.model.device.name,
            "config": effective,
            "mesh": {"vertices_per_world": len(obj.particles), "tets_per_world": len(labels)},
            "workload": "initial targets held; action path reissues identical targets each step; cameras off",
            "task_success_validated": False,
            "observation_scope": {
                "path": "existing sim.step observation/evaluation with batched state readback",
                "toy_summary_worlds": [0],
                "proprioception": "empty: this scene has no embodiment",
                "complete_per_world_policy_observation": False,
            },
            "contact_recording_enabled": bool(record_contacts),
            "motor_effort_recording_enabled": bool(
                getattr(grip, "motor_effort_kind", None) == "applied_external_joint_effort"
                or (record_contacts and getattr(grip, "motor_observables", None) is not None)
            ),
            "motor_effort_limits_validated": False,
            "owned_state_observation": _owned_observation(grip, final_snapshot),
            "build_s_excluding_module_import": build_s,
            "contact_recording_setup_s": recording_setup_s,
            "first_reset_warmup_capture_s": first_reset_s,
            "warmed_reset_with_observation_s": reset_s,
            "physics_without_readback": rates(physics_s, physics_sim_s),
            "target_update_sim_step_observe": rates(action_s, action_sim_s),
            "physics_final_state_finite": physics_finite,
            "action_final_state_finite": finite,
            "last_step_terminated": bool(last_step[2]),
            "graph_before": graph_before,
            "physics_graph_replays": graph_physics["steps_replayed"] - graph_before["steps_replayed"],
            "physics_eager_steps": graph_physics["steps_eager"] - graph_before["steps_eager"],
            "action_graph_replays": graph_action["steps_replayed"] - graph_physics["steps_replayed"],
            "action_eager_steps": graph_action["steps_eager"] - graph_physics["steps_eager"],
            "graph_after": graph_action,
            "memory_warmed": memory_warmed,
            "memory_after": _memory(sim.model.device),
        }
        if check_core:
            try:
                row["core_checks"] = core_check(sim, obj, grip, steps=core_steps, resetter=resetter)
            except Exception as exc:
                row["core_checks"] = {"passed": False, "error": f"{type(exc).__name__}: {exc}"}
        return row
    finally:
        sim.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="JSON scene configuration")
    parser.add_argument("--out", type=Path, default=OUTPUT / "benchmark.json")
    parser.add_argument("--worlds", type=int, nargs="+", default=[1, 8, 32])
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--warm-steps", type=int, default=2)
    parser.add_argument(
        "--core-check", action="store_true", help="check reset/replay at each size and add two-world checks"
    )
    parser.add_argument("--core-steps", type=int, default=12)
    parser.add_argument(
        "--record-contacts",
        action="store_true",
        help="include matched native contact/motor recording in both timing paths",
    )
    args = parser.parse_args()
    if min([args.steps, args.core_steps, *args.worlds]) <= 0 or args.warm_steps < 0:
        parser.error("world counts and step counts must be positive; warm steps may be zero")
    config = json.loads(args.config.read_text()) if args.config else {}
    if not isinstance(config, dict):
        parser.error("config must be a JSON object")
    result = {
        "note": "Cost benchmark and focused control checks; physical/task acceptance is reported by the full demo.",
        "memory_note": "Device-used snapshots include other processes; Warp pool high is cumulative within this process.",
        "recording_note": "Bare timing is the default; this benchmark does not validate actuator effort caps.",
        "profiles": [],
    }
    counts = list(dict.fromkeys(args.worlds))
    if args.core_check and 2 not in counts:
        counts.append(2)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    failed = False
    for worlds in counts:
        try:
            row = benchmark(
                config,
                worlds=worlds,
                steps=args.steps,
                warm_steps=args.warm_steps,
                check_core=args.core_check,
                core_steps=args.core_steps,
                record_contacts=args.record_contacts,
            )
            failed |= not row["physics_final_state_finite"] or not row["action_final_state_finite"]
            failed |= not row.get("core_checks", {"passed": True})["passed"]
        except Exception as exc:
            row = {"worlds": worlds, "error": f"{type(exc).__name__}: {exc}"}
            failed = True
        result["profiles"].append(row)
        args.out.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        gc.collect()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
