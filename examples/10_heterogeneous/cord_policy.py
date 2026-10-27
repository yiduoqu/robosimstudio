# SPDX-License-Identifier: Apache-2.0
"""A small observed-cord command policy for the bounded floating gripper.

Call ``step`` before each control interval. Only commands are changed: the
existing dynamic servo supplies finite forces and may stall. Observe through
closure, then freeze the command frame before lifting the physical object.
"""

from __future__ import annotations

import numpy as np
from cord_grasp import (
    DEFAULT_PAD_HALF,
    _minimum_proxy_z,
    _quaternion,
    build_cord_pick_trajectory,
    cord_approach_displacement,
    cord_targets,
    observe_cord,
    table_safe_cord_center,
)
from trajectory import _path, _points, _poses

# Endpoint clearance reserve for quaternion-interpolated pad corners and
# float32 target storage. Every sample still faces the physical requirement.
COMMAND_CLEARANCE_RESERVE = 0.000020


def _smooth(value):
    value = np.clip(value, 0.0, 1.0)
    return value**3 * (10 + value * (-15 + 6 * value))


class CordPickupPolicy:
    """Track a moving apex at 30 Hz using short, smooth substep commands.

    Construct after physical settling, with the previous COMMAND poses, not
    measured finger poses. ``step(frame, positions, gripper)`` installs one
    21-sample segment at 600 Hz. At ``lift_start_frame`` it instead installs a
    single frozen lift/transfer path. ``pick_end_frame`` starts placement.

    Pads must match ``pad_half`` and have no ledges. The initial approach check
    covers the entire frozen tetrahedral boundary; future deformation is not
    a collision-clearance guarantee. Table clearance is checked for every
    commanded substep. Contact response and actuator saturation remain physics.
    ``post_grasp_half_gap`` optionally requests continued closure during the
    first 0.4 s of lift; carriage motion and actuator limits are unchanged.
    ``normal_approach`` uses the observed tool normal for hover/descent while
    preserving the existing hover height and full-boundary clearance check.
    """

    def __init__(
        self,
        start_poses,
        current_positions,
        source_vertices,
        tets,
        labels,
        *,
        dt,
        start_frame,
        control_hz=30,
        world=0,
        table_z=0.75,
        pad_half=DEFAULT_PAD_HALF,
        particle_radius=0.0007,
        compression_per_side=0.0007,
        approach_clearance=0.001,
        open_clearance=0.003,
        table_clearance=0.0005,
        max_center_raise=0.002,
        displacement=(0.18, -0.10),
        lift_height=0.35,
        post_grasp_half_gap=None,
        parallel=False,
        freeze_on_close=False,
        freeze_on_contact=False,
        normal_approach=False,
    ):
        self.dt, self.control_hz = float(dt), int(control_hz)
        self.parallel = bool(parallel)
        self.freeze_on_close = bool(freeze_on_close)
        self.freeze_on_contact = bool(freeze_on_contact)
        if self.freeze_on_close and self.freeze_on_contact:
            raise ValueError("freeze_on_close and freeze_on_contact are mutually exclusive")
        self._contact_history = []
        self.normal_approach = bool(normal_approach)
        self._closure_observation = None
        if not np.isfinite(dt) or dt <= 0 or control_hz != self.control_hz or self.control_hz <= 0:
            raise ValueError("dt and integer control_hz must be positive")
        self.control_dt = 1 / self.control_hz
        self.substeps = round(self.control_dt / self.dt)
        if self.substeps < 1 or not np.isclose(self.substeps * self.dt, self.control_dt, atol=1.0e-10, rtol=0):
            raise ValueError("each control interval must contain an integer number of physics steps")
        if int(start_frame) != start_frame or start_frame < 0 or int(world) != world or world < 0:
            raise ValueError("start_frame and world must be nonnegative integers")
        self.start_frame, self.world = int(start_frame), int(world)
        self.pad_half = np.asarray(pad_half, dtype=float)
        self.displacement = np.asarray(displacement, dtype=float)
        scalars = (
            table_z,
            particle_radius,
            compression_per_side,
            approach_clearance,
            open_clearance,
            table_clearance,
            max_center_raise,
            lift_height,
        )
        if not np.isfinite(scalars).all() or min(scalars[1:-1]) < 0 or lift_height <= 0:
            raise ValueError("finite geometry, nonnegative clearances and positive lift are required")
        if self.pad_half.shape != (3,) or not np.isfinite(self.pad_half).all() or np.any(self.pad_half <= 0):
            raise ValueError("pad_half must contain three finite positive dimensions")
        if self.displacement.shape != (2,) or not np.isfinite(self.displacement).all():
            raise ValueError("displacement must contain two finite coordinates")
        if post_grasp_half_gap is not None and (
            not np.isfinite(post_grasp_half_gap) or post_grasp_half_gap <= self.pad_half[0]
        ):
            raise ValueError("post_grasp_half_gap must be finite and leave a positive solid-pad gap")
        self.post_grasp_half_gap = None if post_grasp_half_gap is None else float(post_grasp_half_gap)
        self.table_z, self.table_clearance, self.max_center_raise = table_z, table_clearance, max_center_raise
        self.command_table_clearance = table_clearance + COMMAND_CLEARANCE_RESERVE
        self.particle_radius, self.compression_per_side = particle_radius, compression_per_side
        self.lift_height = lift_height
        self.source = _points(source_vertices, "source_vertices", minimum=4).copy()
        self.tets, self.labels = np.asarray(tets, dtype=int).copy(), np.asarray(labels).copy()
        self.previous_target = _poses(start_poses)
        self.initial_target = self.previous_target.copy()
        current = _points(current_positions, "current_positions", minimum=4)
        # Reuse the quantified whole-boundary SAT check, not a cord-only bound.
        _, checked, self.apex_ids = build_cord_pick_trajectory(
            self.previous_target,
            current,
            self.source,
            self.tets,
            self.labels,
            dt=self.dt,
            table_z=table_z,
            pad_half=self.pad_half,
            particle_radius=particle_radius,
            compression_per_side=compression_per_side,
            approach_clearance=approach_clearance,
            table_clearance=self.command_table_clearance,
            max_center_raise=max_center_raise,
            elapsed_before_s=start_frame / self.control_hz,
            approach_only=True,
            parallel=self.parallel,
            normal_approach=self.normal_approach,
        )
        chosen_gap = checked["approach_candidates"][-1]["half_gap_m"]
        self.open_clearance = max(
            open_clearance,
            chosen_gap - checked["observed_cord_width_m"] / 2 - self.pad_half[0] - particle_radius,
        )
        self._phase_frames = []
        phases, cursor = {}, self.start_frame
        for name, seconds in (
            ("tracked_hover", 0.7),
            ("tracked_descent", 0.7),
            ("close", 0.8),
            ("grasp_hold", 0.4),
            ("lift", 2.4),
            ("lift_hold", 0.7),
            ("transport", 1.8),
            ("transport_hold", 0.3),
        ):
            count = round(seconds * self.control_hz)
            if count < 1 or not np.isclose(count / self.control_hz, seconds):
                raise ValueError("control rate must represent all phase durations exactly")
            phases[name] = {
                "start_s": cursor / self.control_hz,
                "end_s": (cursor + count) / self.control_hz,
                "start_frame": cursor,
                "end_frame": cursor + count,
                "start_sample": (cursor - self.start_frame) * self.substeps,
                "end_sample": (cursor + count - self.start_frame) * self.substeps,
            }
            self._phase_frames.append((name, cursor, cursor + count))
            cursor += count
        self.lift_start_frame = phases["lift"]["start_frame"]
        self.pick_end_frame = cursor
        self.metadata = {
            "grasp": "single_upper_cord_tracked_then_frozen",
            "parallel_carrier_commands": self.parallel,
            "phases": phases,
            "start_frame": self.start_frame,
            "end_frame": cursor,
            "dt_s": self.dt,
            "control_hz": self.control_hz,
            "command_segment_samples": self.substeps + 1,
            "elapsed_before_s": self.start_frame / self.control_hz,
            "duration_s": (cursor - self.start_frame) / self.control_hz,
            "elapsed_end_s": cursor / self.control_hz,
            "grasp_check_s": self.lift_start_frame / self.control_hz,
            "transport_window_s": [phases["transport"]["start_s"], phases["transport_hold"]["end_s"]],
            "target_translation_xy_m": self.displacement.tolist(),
            "apex_source_ids": self.apex_ids.tolist(),
            "pad_half_extents_m": self.pad_half.tolist(),
            "support_ledges": False,
            "required_pad_table_clearance_m": float(self.table_clearance),
            "command_endpoint_clearance_reserve_m": COMMAND_CLEARANCE_RESERVE,
            "planned_endpoint_pad_table_clearance_m": float(self.command_table_clearance),
            "compression_per_side_m": float(compression_per_side),
            "post_grasp_half_gap_m": self.post_grasp_half_gap,
            "open_clearance_beyond_inflated_cord_m": float(self.open_clearance),
            "initial_approach_candidates": checked["approach_candidates"],
            "normal_approach": self.normal_approach,
            "freeze_on_contact": self.freeze_on_contact,
            "initial_approach_direction_world": checked["approach_direction_world"],
            "initial_approach_displacement_world_m": checked["approach_displacement_world_m"],
            "approach_frozen_mesh_clearance_m": float(approach_clearance),
            "approach_clearance_scope": "initial frozen whole boundary; future deformation not guaranteed",
            "observations": [],
        }
        self._last_frame = self.start_frame - 1
        self._frozen = False
        self._last_observation = None
        self._previous_rotation = observe_cord(current, self.source, self.tets, self.labels)["rotation"].copy()

    def _validate_commands(self, poses):
        minimum = min(_minimum_proxy_z(pair, self.pad_half) for pair in poses)
        if minimum < self.table_z + self.table_clearance - 1.0e-7:
            raise ValueError(
                f"commanded oriented pad clearance {minimum - self.table_z:.9f} m "
                f"is below required {self.table_clearance:.9f} m"
            )
        return minimum

    def _check_gripper(self, gripper):
        if gripper.spec.mode != "dynamic" or gripper.spec.ledge:
            raise ValueError("cord policy requires dynamic flat fingers without ledges")
        if not np.isclose(gripper.spec.pad_half_width, self.pad_half[1]):
            raise ValueError("policy pad width must match the physical fingers")
        if not np.isclose(gripper.dt, self.dt):
            raise ValueError("policy and gripper physics steps must match")
        if not np.allclose(self.pad_half[[0, 2]], [0.003, 0.006]):
            raise ValueError("physical flat fingers have fixed 3 mm thickness and 6 mm half-height")
        self.metadata["actuator_force_cap_n"] = float(gripper.spec.force_limit)
        self.metadata["actuator_torque_cap_nm"] = float(gripper.spec.torque_limit)

    def step(self, frame, current_positions, gripper, *, ring_normal_loads=None):
        """Install commands before simulation; return the current phase name.

        Call exactly once for every control frame from ``start_frame`` through
        ``lift_start_frame``. Afterwards this leaves the installed frozen path
        alone, so a caller can run physics without repeated host observations.
        """
        if frame < self.start_frame:
            return "settle"
        if frame >= self.pick_end_frame:
            return "complete"
        phase, begin, end = next(item for item in self._phase_frames if item[1] <= frame < item[2])
        if self.freeze_on_contact:
            loads = np.asarray(ring_normal_loads, dtype=float)
            if loads.shape != (2,) or not np.isfinite(loads).all() or np.any(loads < 0):
                raise ValueError("freeze_on_contact requires two actual finite nonnegative ring normal loads")
            if phase in ("close", "grasp_hold") and np.all(loads > 0.2):
                self._contact_history.append({"frame": int(frame), "ring_normal_load_N": loads.tolist()})
                self._contact_history = self._contact_history[-3:]
            else:
                self._contact_history.clear()
        if self._frozen:
            return phase
        if frame != self._last_frame + 1:
            raise ValueError("tracked policy must be called once per consecutive control frame")
        self._check_gripper(gripper)
        if frame == self.lift_start_frame:
            lifted = self.previous_target.copy()
            lifted[:, 2] += self.lift_height
            transferred = lifted.copy()
            transferred[:, :2] += self.displacement
            poses, _ = _path(
                self.previous_target,
                (
                    ("lift", lifted, 2.4),
                    ("lift_hold", lifted, 0.7),
                    ("transport", transferred, 1.8),
                    ("transport_hold", transferred, 0.3),
                ),
                self.dt,
                frame / self.control_hz,
                parallel=self.parallel,
            )
            if self.post_grasp_half_gap is not None:
                separation = self.previous_target[1, :3] - self.previous_target[0, :3]
                half_gap = float(np.linalg.norm(separation) / 2)
                if self.post_grasp_half_gap > half_gap:
                    raise ValueError("post_grasp_half_gap must request closure relative to the frozen aperture")
                closing_axis = separation.astype(float) / (2 * half_gap)
                # Equal/opposite shifts preserve every carriage midpoint and
                # quaternion. Actual fingers may stall under the existing caps.
                blend_duration = 0.4
                fraction = _smooth(np.arange(len(poses)) * self.dt / blend_duration)
                shift = ((self.post_grasp_half_gap - half_gap) * fraction)[:, None] * closing_axis
                positions = poses[:, :, :3].astype(float)
                positions[:, 0] -= shift
                positions[:, 1] += shift
                poses[:, :, :3] = positions
                self.metadata["post_grasp_aperture"] = {
                    "initial_half_gap_m": half_gap,
                    "requested_half_gap_m": self.post_grasp_half_gap,
                    "requested_solid_inner_gap_m": 2 * (self.post_grasp_half_gap - self.pad_half[0]),
                    "interpolation": "quintic smoothstep; hold after blend",
                    "blend_start_s": frame / self.control_hz,
                    "blend_end_s": frame / self.control_hz + blend_duration,
                    "blend_duration_s": blend_duration,
                    "carriage_midpoint_and_orientation": "unchanged from frozen lift/transfer path",
                    "actuation": "dynamic bounded servo; requested aperture is not prescribed physical motion",
                }
            self._validate_commands(poses)
            gripper.set_trajectory(poses, world=self.world)
            self._frozen = True
            self.metadata["frozen_at_frame"] = int(frame)
            self.metadata["frozen_start_targets"] = self.previous_target.tolist()
            self.metadata["frozen_last_observation"] = self._last_observation
            return phase
        current = _points(current_positions, "current_positions", minimum=4)
        observation = self._closure_observation
        cached_observation = observation is not None
        if observation is None:
            observation = observe_cord(current, self.source, self.tets, self.labels)
        # A fitted tangent has an equivalent 180-degree jaw-swapping frame.
        # Keep finger identities continuous if its world-axis sign branch flips.
        if observation["rotation"][:, 0] @ self._previous_rotation[:, 0] < 0:
            rotation = observation["rotation"] @ np.diag((-1.0, -1.0, 1.0))
            local = current[observation["apex_ids"]] @ rotation
            observation["rotation"] = rotation
            observation["quaternion"] = _quaternion(rotation)
            observation["section_bounds_in_gripper_m"] = (
                np.stack((local.min(axis=0), local.max(axis=0))) - observation["center"] @ rotation
            )
        self._previous_rotation = observation["rotation"].copy()
        contact_trigger = self.freeze_on_contact and len(self._contact_history) == 3
        if ((self.freeze_on_close and phase == "close") or contact_trigger) and self._closure_observation is None:
            self._closure_observation = observation
            self.metadata["grasp_frame_frozen_at_s"] = frame / self.control_hz
            self.metadata["grasp_frame_frozen_center_m"] = observation["center"].tolist()
            self.metadata["grasp_frame_frozen_quaternion_xyzw"] = observation["quaternion"].tolist()
            self.metadata["grasp_frame_frozen_width_m"] = float(observation["width_m"])
            if contact_trigger:
                self.metadata["contact_freeze_trigger"] = {
                    "frame": int(frame),
                    "time_s": frame / self.control_hz,
                    "consecutive_actual_loads": list(self._contact_history),
                }
        center, geometry = table_safe_cord_center(
            observation, self.pad_half, self.table_z, self.command_table_clearance, self.max_center_raise
        )
        base_gap = observation["width_m"] / 2 + self.pad_half[0] + self.particle_radius
        opened, closed = base_gap + self.open_clearance, base_gap - self.compression_per_side
        if closed <= self.pad_half[0] + 0.0002:
            raise ValueError("observed cord is too thin for the specified pinch compression")
        progress = (frame + 1 - begin) / (end - begin)
        approach_active = phase in ("tracked_hover", "tracked_descent")
        approach_displacement = np.zeros(3)
        if approach_active:
            approach_displacement = cord_approach_displacement(
                observation, current, self.pad_half, center, normal_approach=self.normal_approach
            )
        if phase == "tracked_hover":
            center += approach_displacement
            goal = cord_targets(observation, opened, center=center)
            # Reach the observed hover smoothly rather than jumping there in a
            # single 33 ms control update. Fit/position are refreshed throughout.
            blend, _ = _path(
                self.initial_target, (("hover", goal, 1.0),), 1.0 / (end - begin), 0, parallel=self.parallel
            )
            goal = blend[frame + 1 - begin]
        elif phase == "tracked_descent":
            center += approach_displacement * (1 - _smooth(progress))
            goal = cord_targets(observation, opened, center=center)
        else:
            gap = opened + (closed - opened) * _smooth(progress) if phase == "close" else closed
            goal = cord_targets(observation, gap, center=center)
        poses, _ = _path(
            self.previous_target,
            (("control", goal, self.control_dt),),
            self.dt,
            frame / self.control_hz,
            parallel=self.parallel,
        )
        clearance = self._validate_commands(poses) - self.table_z
        gripper.set_trajectory(poses, world=self.world)
        self.previous_target = poses[-1].copy()
        self._last_frame = int(frame)
        record = {
            **geometry,
            "frame": int(frame),
            "time_s": frame / self.control_hz,
            "phase": phase,
            "observation_kind": "frozen_grasp_reference" if cached_observation else "fresh_cord_fit",
            "observation_time_s": (
                self.metadata["grasp_frame_frozen_at_s"] if cached_observation else frame / self.control_hz
            ),
            "observed_cord_width_m": observation["width_m"],
            "frame_fit_rms_m": observation["fit_rms_m"],
            "quaternion_xyzw": observation["quaternion"].tolist(),
            "approach_active": approach_active,
            "approach_direction_world": (
                (approach_displacement / np.linalg.norm(approach_displacement)).tolist() if approach_active else None
            ),
            "approach_displacement_world_m": approach_displacement.tolist(),
            "commanded_midpoint_world_m": self.previous_target[:, :3].mean(axis=0).tolist(),
            "commanded_half_gap_m": float(np.linalg.norm(np.diff(self.previous_target[:, :3], axis=0)) / 2),
            "minimum_commanded_pad_table_clearance_m": float(clearance),
        }
        # Distinguish measured fits from the reference reused during closure.
        self.metadata["observations"].append(record)
        self._last_observation = record
        return phase


class CordPlacementPolicy:
    """Translate held commands until observed body support permits release.

    This example policy changes public gripper targets only. Clearance checks
    use the current observation; they do not guarantee future collision safety.
    """

    def __init__(
        self,
        start_poses,
        *,
        dt,
        start_frame,
        desired_body_xy,
        initial_body_z,
        weight_n,
        control_hz=30,
        world=0,
        table_z=0.75,
        pad_half=DEFAULT_PAD_HALF,
    ):
        self.target = _poses(start_poses).copy()
        if int(control_hz) != control_hz or control_hz <= 0 or int(world) != world or world < 0:
            raise ValueError("control_hz must be positive and world must be a nonnegative integer")
        if int(start_frame) != start_frame or start_frame < 0:
            raise ValueError("start_frame must be a nonnegative integer")
        self.dt, self.hz, self.world = float(dt), int(control_hz), int(world)
        self.control_dt, self.start_frame = 1 / self.hz, int(start_frame)
        self.xy = np.asarray(desired_body_xy, dtype=float)
        self.table_z, self.pad_half = float(table_z), np.asarray(pad_half, dtype=float)
        self.initial_body_z, self.weight = float(initial_body_z), float(weight_n)
        if (
            self.xy.shape != (2,)
            or not np.isfinite(self.xy).all()
            or self.weight <= 0
            or self.pad_half.shape != (3,)
            or not np.isfinite(self.pad_half).all()
            or np.any(self.pad_half <= 0)
            or not np.isfinite([dt, initial_body_z, weight_n, table_z]).all()
            or dt <= 0
            or not np.isclose(round(self.control_dt / dt) * dt, self.control_dt)
        ):
            raise ValueError("finite placement geometry and an integer physics/control cadence are required")
        self.deadline = self.start_frame + 36 * self.hz
        self.end_frame = None
        self._last_frame, self._window = self.start_frame - 1, []
        self._force_window = []
        self._contact_history = []
        self._support_start = None
        self._arc = None
        self._arc_angle = 0.0
        self._body_landed = False
        self.stage = "approach"
        self.status = "lower_align"
        self.metadata = {
            "policy": "observed_body_support_before_release",
            "start_frame": self.start_frame,
            "elapsed_before_s": self.start_frame / self.hz,
            "timeout_s": 36.0,
            "desired_body_xy_m": self.xy.tolist(),
            "weight_N": self.weight,
            "limits": {
                "peak_descent_m_s": 0.06,
                "peak_xy_m_s": 0.03,
                "peak_load_transfer_m_s": 0.01,
                "load_admittance_m_per_N_s": 0.020,
                "peak_arc_m_s": 0.020,
                "arc_angle_limit_deg": 50.0,
                "instantaneous_load_pause_weights": 1.5,
                "tracking_pause_m": 0.005,
                "tracking_pause_deg": 3.0,
                "sustained_contact_threshold_weights": 0.2,
                "sustained_contact_observations": 3,
                "guarded_hold_s": 1.0,
                "pad_floor_clearance_m": 0.005,
                "apex_underside_reserve_m": 0.001,
            },
            "gate": {
                "body_minimum_above_table_m": 0.003,
                "body_mean_above_initial_m": 0.010,
                "body_centroid_speed_m_s": 0.020,
                "xy_error_m": 0.010,
                "centroid_drift_m": 0.002,
                "stable_duration_s": 0.4,
                "mean_table_load_weight_range": [0.9, 1.1],
                "body_bearing_force_N": 0.02,
            },
            "body_contact_attribution": "barycentric vertices exclusive to label-1 tetrahedra",
            "phases": {"support_feedback": {"start_s": self.start_frame / self.hz, "end_s": None}},
            "observations": [],
            "stages": [{"name": "approach", "start_s": self.start_frame / self.hz, "end_s": None}],
            "status": self.status,
        }

    def _fail(self, reason, frame):
        self.status = "failed_hold"
        self._stage("failed_hold", frame)
        self.metadata.update(status=self.status, failure_reason=reason, failure_time_s=frame / self.hz)
        self.metadata["phases"]["support_feedback"]["end_s"] = frame / self.hz
        return self.status

    def finish_interval(self, end_frame):
        """Mark completion after the last release/settle interval actually ran."""
        if self.status == "release" and end_frame >= self.end_frame:
            self.status = "complete"
            self._stage("complete", end_frame)
            self.metadata["status"] = self.status

    def _stage(self, name, frame):
        if name != self.stage:
            self.metadata["stages"][-1]["end_s"] = frame / self.hz
            self.metadata["stages"].append({"name": name, "start_s": frame / self.hz, "end_s": None})
            self.stage = name

    def _arc_point(self, angle):
        anchor, relative, direction = self._arc
        horizontal, vertical = relative @ direction, relative[2]
        point = anchor + relative + direction * (horizontal * (np.cos(angle) - 1) + vertical * np.sin(angle))
        point[2] += vertical * (np.cos(angle) - 1) - horizontal * np.sin(angle)
        return point

    def step(self, frame, body_points, body_velocities, apex_points, actual_poses, observation, gripper):
        """Install one command segment, or the gated three-second release path."""
        if frame != self._last_frame + 1:
            raise ValueError("placement step must be called once per consecutive control frame")
        self._last_frame = frame
        if self.status == "release":
            self.finish_interval(frame)
            return self.status
        if self.status in ("complete", "failed_hold"):
            return self.status
        body, apex = _points(body_points, "body_points"), _points(apex_points, "apex_points")
        center = body.mean(axis=0)
        speed = float(np.linalg.norm(np.asarray(body_velocities).mean(axis=0)))
        xy_error = float(np.linalg.norm(self.xy - center[:2]))
        actual_clearance = _minimum_proxy_z(_poses(actual_poses), self.pad_half) - self.table_z
        patch_clearance = float(apex[:, 2].min() - 0.0007 - self.table_z)
        floor_fz = float(observation["static_support_force_on_object_N"][2])
        body_fz = float(observation["body_support_force_on_object_N"][2])
        if not np.isfinite([floor_fz, body_fz]).all():
            return self._fail("non-finite table/body support force observation", frame)
        window_frames = round(0.4 * self.hz)
        self._force_window.append((frame, floor_fz))
        self._force_window = [item for item in self._force_window if frame - item[0] <= window_frames]
        force_ready = frame - self._force_window[0][0] >= window_frames
        transfer_load = float(np.mean([item[1] for item in self._force_window]))
        supported_load = force_ready and 0.9 * self.weight <= transfer_load <= 1.1 * self.weight
        low = body[:, 2].min() <= self.table_z + 0.003 and center[2] <= self.initial_body_z + 0.010
        body_bearing = low and body_fz > 0.02
        actual = _poses(actual_poses)
        # Jaw closure deliberately yields to the held cord under finite force.
        # Carriage tracking uses the midpoint, independent of that aperture error.
        tracking_m = float(np.linalg.norm(actual[:, :3].mean(axis=0) - self.target[:, :3].mean(axis=0)))
        pad_tracking_m = float(np.max(np.linalg.norm(actual[:, :3] - self.target[:, :3], axis=1)))
        tracking_deg = float(
            np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(actual[:, 3:] * self.target[:, 3:], axis=1)), 0, 1))).max()
        )
        instant_high, mean_high = floor_fz > 1.5 * self.weight, force_ready and transfer_load > 1.1 * self.weight
        tracking_pause = tracking_m > 0.005 or tracking_deg > 3.0
        contact_probe = floor_fz > 0.2 * self.weight
        self._contact_history = [*self._contact_history, frame][-3:] if contact_probe else []
        if self._support_start is None and (len(self._contact_history) >= 3 or body_bearing):
            self._support_start = frame
            self.metadata["sustained_table_contact_frame"] = int(frame)
        self._body_landed = self._body_landed or bool(body_bearing)
        if self._support_start is not None:
            if frame - self._support_start < self.hz:
                self._stage("guarded_hold", frame)
            elif self._body_landed:
                self._stage("body_align", frame)
            elif self._arc is None and not (instant_high or mean_high or tracking_pause):
                foot = observation.get("static_contact_center_m")
                if foot is not None:
                    anchor = np.asarray(foot, dtype=float)
                    com = np.asarray(observation["object_mass_weighted_center_m"], dtype=float)
                    direction = com - anchor
                    direction[2] = 0
                    if not np.isfinite([anchor, com]).all() or np.linalg.norm(direction) < 0.001:
                        return self._fail("cannot define a finite foot-to-COM arc direction", frame)
                    direction /= np.linalg.norm(direction)
                    relative = self.target[:, :3].mean(axis=0) - anchor
                    self._arc = anchor, relative, direction
                    self.metadata["arc"] = {
                        "anchor_m": anchor.tolist(),
                        "direction_world": direction.tolist(),
                        "start_midpoint_m": (anchor + relative).tolist(),
                        "start_s": frame / self.hz,
                        "maximum_angle_deg": 50.0,
                        "quaternion_and_aperture": "fixed",
                    }
                    self._stage("pivot_arc", frame)
            elif self._arc is not None:
                self._stage("pivot_arc" if self._arc_angle < np.deg2rad(50) else "arc_hold", frame)
        candidate = bool(self.stage == "body_align" and body_bearing and speed <= 0.020 and xy_error <= 0.010)
        if candidate:
            self._window.append((frame, center.copy(), floor_fz))
        else:
            self._window.clear()
        self._window = [item for item in self._window if frame - item[0] <= window_frames]
        mean_load = float(np.mean([item[2] for item in self._window])) if self._window else None
        drift = (
            float(max(np.linalg.norm(item[1] - self._window[0][1]) for item in self._window)) if self._window else None
        )
        gate = bool(
            self._window
            and frame - self._window[0][0] >= window_frames
            and drift <= 0.002
            and supported_load
            and not instant_high
            and not tracking_pause
            and 0.9 * self.weight <= mean_load <= 1.1 * self.weight
        )
        record = {
            "frame": int(frame),
            "time_s": frame / self.hz,
            "body_center_m": center.tolist(),
            "body_minimum_z_m": float(body[:, 2].min()),
            "body_centroid_speed_m_s": speed,
            "xy_error_m": xy_error,
            "actual_pad_floor_clearance_m": float(actual_clearance),
            "apex_underside_floor_clearance_m": patch_clearance,
            "table_Fz_N": floor_fz,
            "body_bearing_Fz_N": body_fz,
            "candidate": candidate,
            "mean_table_Fz_N": mean_load,
            "transfer_mean_table_Fz_N": transfer_load,
            "transfer_force_window_ready": bool(force_ready),
            "centroid_drift_m": drift,
            "release_gate": gate,
            "phase": "support_hold" if candidate else "lower_align",
            "stage": self.stage,
            "tracking_position_error_m": tracking_m,
            "maximum_pad_position_error_m": pad_tracking_m,
            "tracking_orientation_error_deg": tracking_deg,
            "instantaneous_load_pause": bool(instant_high),
            "mean_load_pause": bool(mean_high),
            "tracking_pause": tracking_pause,
            "static_contact_center_m": observation.get("static_contact_center_m"),
            "whole_mass_center_m": observation.get("object_mass_weighted_center_m"),
            "arc_angle_deg": float(np.degrees(self._arc_angle)),
            "nominal_arc_midpoint_m": None if self._arc is None else self._arc_point(self._arc_angle).tolist(),
            "commanded_midpoint_m": self.target[:, :3].mean(axis=0).tolist(),
            "pause_reasons": [],
            "vertical_admittance_peak_m_s": 0.0,
        }
        self.metadata["observations"].append(record)
        if observation["inverted_tetrahedra"]:
            return self._fail("inverted tetrahedra in observed physical state", frame)
        if observation["maximum_contact_plane_penetration_m"] > 0.0005:
            return self._fail("recorded contact penetration above 0.5 mm", frame)
        if observation["carrier_normal_load_N"] > 0.02:
            return self._fail("carrier/object contact load above 0.02 N", frame)
        if actual_clearance < 0.005:
            return self._fail("actual oriented pad floor clearance below 5 mm", frame)
        if sum(observation["ring_normal_load_N"]) <= 0.02 and body_fz <= 0.02:
            return self._fail("held ring contact lost before body-bearing support", frame)
        if gate:
            opened = self.target.copy()
            axis = self.target[1, :3] - self.target[0, :3]
            axis /= np.linalg.norm(axis)
            center_command = self.target[:, :3].mean(axis=0)
            if np.linalg.norm(self.target[1, :3] - self.target[0, :3]) / 2 >= 0.025:
                return self._fail("release aperture would not open the held command", frame)
            opened[0, :3], opened[1, :3] = center_command - 0.025 * axis, center_command + 0.025 * axis
            retreat = opened.copy()
            retreat[:, 2] += 0.100
            path, phases = _path(
                self.target,
                (("release", opened, 0.8), ("retreat", retreat, 0.7), ("settle", retreat, 1.5)),
                self.dt,
                frame / self.hz,
                parallel=True,
            )
            if min(_minimum_proxy_z(pair, self.pad_half) for pair in path) < self.table_z + 0.005:
                return self._fail("opening would violate oriented pad clearance", frame)
            gripper.set_trajectory(path, world=self.world)
            self.end_frame = frame + round(phases["duration_s"] * self.hz)
            self.status = "release"
            self._stage("release", frame)
            self.metadata["phases"]["support_feedback"]["end_s"] = frame / self.hz
            self.metadata.update(
                phases={**self.metadata["phases"], **phases["phases"]},
                status=self.status,
                release_gate_time_s=frame / self.hz,
                duration_s=phases["elapsed_end_s"] - self.start_frame / self.hz,
                elapsed_end_s=phases["elapsed_end_s"],
                end_frame=self.end_frame,
            )
            return self.status
        if frame >= self.deadline:
            return self._fail("body support/placement gate timed out; grasp held", frame)
        goal = self.target.copy()
        # All short segments use quintic interpolation (peak/mean speed=1.875).
        command_dt = self.control_dt / 1.875
        delta, angle_step = np.zeros(3), 0.0
        pause = []
        if instant_high:
            pause.append("instantaneous_floor_overload")
        if mean_high:
            pause.append("mean_floor_overload")
        if tracking_pause:
            pause.append("tracking_lag")
        nominal_allowed = not pause
        align = self.stage == "body_align" or (self.stage == "approach" and not contact_probe)
        if align and nominal_allowed and xy_error > 0.010:
            delta_xy = self.xy - center[:2]
            delta[:2] = delta_xy * min(1.0, 0.030 * command_dt / max(np.linalg.norm(delta_xy), 1e-12))
        if self.stage == "approach" and not contact_probe and nominal_allowed:
            delta[2] = -0.060 * command_dt
        if self.stage == "pivot_arc" and nominal_allowed:
            if floor_fz <= 0.02:
                pause.append("no_current_floor_support")
            else:
                _, relative, direction = self._arc
                radius = np.hypot(relative @ direction, relative[2])
                angle_step = min(np.deg2rad(50) - self._arc_angle, 0.020 * command_dt / max(radius, 1e-6))
                delta = self._arc_point(self._arc_angle + angle_step) - self._arc_point(self._arc_angle)
        relief_vz = 0.0
        if force_ready and (self._support_start is not None or contact_probe or instant_high or mean_high):
            error = transfer_load - np.clip(transfer_load, 0.9 * self.weight, 1.1 * self.weight)
            relief_vz = float(np.clip(0.020 * error, -0.010, 0.010))
            # A paused arc may need gentle descent to regain its floor contact.
            # Overload and servo lag still veto downward load transfer.
            if relief_vz < 0 and (instant_high or mean_high or tracking_pause):
                relief_vz = 0.0
            delta[2] += relief_vz * command_dt
        command_clearance = _minimum_proxy_z(self.target, self.pad_half) - self.table_z
        if command_clearance < 0.005:
            return self._fail("commanded oriented pad floor clearance below 5 mm", frame)
        downward_budget = max(0.0, min(actual_clearance - 0.0052, command_clearance - 0.0052, patch_clearance - 0.001))
        if delta[2] < -downward_budget:
            # Reject the nominal increment as a whole: angle never advances
            # while a safety guard withholds the corresponding arc motion.
            delta[:] = 0
            delta[2] = max(relief_vz * command_dt, -downward_budget)
            angle_step = 0.0
            pause.append("floor_clearance")
        goal[:, :3] += delta
        self._arc_angle += angle_step
        corrected = bool(np.any(goal[:, :3] != self.target[:, :3]))
        if corrected:
            self._window.clear()
        phase = self.stage + ("_paused" if pause else "")
        path, _ = _path(
            self.target,
            ((phase, goal, self.control_dt),),
            self.dt,
            frame / self.hz,
            parallel=True,
        )
        gripper.set_trajectory(path, world=self.world)
        self.target = goal
        self.status = phase
        record["phase"] = phase
        record["stability_window_reset_by_command"] = corrected
        record["pause_reasons"] = pause
        record["vertical_admittance_peak_m_s"] = relief_vz
        record["arc_angle_deg"] = float(np.degrees(self._arc_angle))
        record["nominal_arc_midpoint_m"] = None if self._arc is None else self._arc_point(self._arc_angle).tolist()
        record["commanded_midpoint_m"] = goal[:, :3].mean(axis=0).tolist()
        self.metadata["status"] = self.status
        return self.status
