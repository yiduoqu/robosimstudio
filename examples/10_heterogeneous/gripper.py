# SPDX-License-Identifier: Apache-2.0
"""Dynamic contact fingers driven by bounded wrenches and reusable target buffers.

The toy is never attached. ``mode="kinematic"`` explicitly retains the original
unlimited baseline; the normal mode has finite authority on every motion axis.
"""

from __future__ import annotations

from dataclasses import dataclass

import newton
import numpy as np
import warp as wp

import robosimstudio as rss


def smooth(time: float, start: float, end: float) -> float:
    t = float(np.clip((time - start) / (end - start), 0.0, 1.0))
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def trajectory_velocities(poses: np.ndarray, dt: float) -> np.ndarray:
    """World-frame linear/angular rates for a substep-sampled [N, 2, 7] path."""
    velocities = np.zeros((*poses.shape[:2], 6), dtype=np.float32)
    if len(poses) < 2:
        return velocities
    velocities[:-1, :, :3] = np.diff(poses[:, :, :3], axis=0) / dt
    a, b = poses[1:, :, 3:], poses[:-1, :, 3:]
    # q_next * inverse(q_current): rotation increments in world coordinates.
    xyz = -a[..., 3, None] * b[..., :3] + b[..., 3, None] * a[..., :3] - np.cross(a[..., :3], b[..., :3])
    scalar = np.sum(a * b, axis=-1)
    sign = np.where(scalar < 0, -1.0, 1.0)
    xyz *= sign[..., None]
    scalar *= sign
    magnitude = np.linalg.norm(xyz, axis=-1)
    angle = 2.0 * np.arctan2(magnitude, np.clip(scalar, 0.0, 1.0))
    scale = np.divide(angle, magnitude * dt, out=np.zeros_like(angle), where=magnitude > 1.0e-10)
    velocities[:-1, :, 3:] = xyz * scale[..., None]
    return velocities


@wp.kernel
def _sample_targets(
    tick: wp.array[int],
    enabled: wp.array[int],
    lengths: wp.array[int],
    poses: wp.array3d[wp.transform],
    velocities: wp.array3d[wp.spatial_vector],
    targets: wp.array2d[wp.transform],
    rates: wp.array2d[wp.spatial_vector],
):
    world, finger = wp.tid()
    if enabled[world] != 0:
        frame = wp.min(tick[world], lengths[world] - 1)
        targets[world, finger] = poses[world, frame, finger]
        rates[world, finger] = velocities[world, frame, finger]


@wp.kernel
def _drive_dynamic(
    bodies: wp.array2d[int],
    targets: wp.array2d[wp.transform],
    rates: wp.array2d[wp.spatial_vector],
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    body_mass: wp.array[float],
    body_world: wp.array[int],
    gravity: wp.array[wp.vec3],
    linear_ke: float,
    linear_kd: float,
    angular_ke: float,
    angular_kd: float,
    force_limit: float,
    torque_limit: float,
    compensate_gravity: bool,
    body_f: wp.array[wp.spatial_vector],
    applied: wp.array2d[wp.spatial_vector],
    peak_force: wp.array2d[float],
    peak_torque: wp.array2d[float],
):
    world, finger = wp.tid()
    body = bodies[world, finger]
    target = targets[world, finger]
    current = body_q[body]
    rotation = wp.transform_get_rotation(current)
    target_rotation = wp.transform_get_rotation(target)
    com = wp.transform_point(current, body_com[body])
    target_com = wp.transform_point(target, body_com[body])
    target_rate = rates[world, finger]
    velocity = body_qd[body]
    target_angular = wp.spatial_bottom(target_rate)
    target_linear = wp.spatial_top(target_rate) + wp.cross(
        target_angular, wp.quat_rotate(target_rotation, body_com[body])
    )
    force = linear_ke * (target_com - com) + linear_kd * (target_linear - wp.spatial_top(velocity))
    if compensate_gravity:
        force -= body_mass[body] * gravity[body_world[body]]
    # Clamp after all actuator contributions, including only this finger's own
    # gravity compensation. Payload weight consumes the same finite budget.
    force_norm = wp.length(force)
    if force_norm > force_limit:
        force *= force_limit / force_norm
    error = target_rotation * wp.quat_inverse(rotation)
    if error[3] < 0.0:
        error = wp.quat(-error[0], -error[1], -error[2], -error[3])
    axis = wp.vec3(error[0], error[1], error[2])
    sine = wp.length(axis)
    rotation_vector = wp.vec3(0.0)
    if sine > 1.0e-8:
        rotation_vector = axis * (2.0 * wp.atan2(sine, error[3]) / sine)
    torque = angular_ke * rotation_vector + angular_kd * (target_angular - wp.spatial_bottom(velocity))
    torque_norm = wp.length(torque)
    if torque_norm > torque_limit:
        torque *= torque_limit / torque_norm
    wrench = wp.spatial_vector(force, torque)
    wp.atomic_add(body_f, body, wrench)
    applied[world, finger] = wrench
    peak_force[world, finger] = wp.max(peak_force[world, finger], wp.length(force))
    peak_torque[world, finger] = wp.max(peak_torque[world, finger], wp.length(torque))


@wp.kernel
def _drive_kinematic(
    bodies: wp.array2d[int],
    targets: wp.array2d[wp.transform],
    rates: wp.array2d[wp.spatial_vector],
    q0: wp.array[wp.transform],
    q1: wp.array[wp.transform],
    qd0: wp.array[wp.spatial_vector],
    qd1: wp.array[wp.spatial_vector],
):
    world, finger = wp.tid()
    body = bodies[world, finger]
    q0[body] = targets[world, finger]
    q1[body] = targets[world, finger]
    qd0[body] = rates[world, finger]
    qd1[body] = rates[world, finger]


@wp.kernel
def _advance_tick(tick: wp.array[int], enabled: wp.array[int], lengths: wp.array[int]):
    world = wp.tid()
    if enabled[world] != 0:
        tick[world] = wp.min(tick[world] + 1, lengths[world] - 1)


@wp.kernel
def _reset_world(
    world: int,
    bodies: wp.array2d[int],
    coords: wp.array2d[int],
    dofs: wp.array2d[int],
    initial: wp.array2d[wp.transform],
    targets: wp.array2d[wp.transform],
    rates: wp.array2d[wp.spatial_vector],
    applied: wp.array2d[wp.spatial_vector],
    peak_force: wp.array2d[float],
    peak_torque: wp.array2d[float],
    tick: wp.array[int],
    q0: wp.array[wp.transform],
    q1: wp.array[wp.transform],
    qd0: wp.array[wp.spatial_vector],
    qd1: wp.array[wp.spatial_vector],
    f0: wp.array[wp.spatial_vector],
    f1: wp.array[wp.spatial_vector],
    jq0: wp.array[float],
    jq1: wp.array[float],
    jqd0: wp.array[float],
    jqd1: wp.array[float],
):
    finger = wp.tid()
    body = bodies[world, finger]
    pose = initial[world, finger]
    q0[body] = pose
    q1[body] = pose
    qd0[body] = wp.spatial_vector()
    qd1[body] = wp.spatial_vector()
    f0[body] = wp.spatial_vector()
    f1[body] = wp.spatial_vector()
    xyz = wp.transform_get_translation(pose)
    quat = wp.transform_get_rotation(pose)
    for i in range(3):
        jq0[coords[world, finger] + i] = xyz[i]
        jq1[coords[world, finger] + i] = xyz[i]
    for i in range(4):
        jq0[coords[world, finger] + 3 + i] = quat[i]
        jq1[coords[world, finger] + 3 + i] = quat[i]
    for i in range(6):
        jqd0[dofs[world, finger] + i] = 0.0
        jqd1[dofs[world, finger] + i] = 0.0
    targets[world, finger] = pose
    rates[world, finger] = wp.spatial_vector()
    applied[world, finger] = wp.spatial_vector()
    peak_force[world, finger] = 0.0
    peak_torque[world, finger] = 0.0
    if finger == 0:
        tick[world] = 0


@dataclass(frozen=True, kw_only=True)
class ScriptedGripperSpec:
    entity: str = "scripted_loop_gripper"
    name: str = "loop_gripper"
    mode: str = "dynamic"
    scripted: bool = True
    finger_mass: float = 0.05
    force_limit: float = 5.0
    torque_limit: float = 0.03
    linear_ke: float = 1500.0
    linear_kd: float = 15.0
    angular_ke: float = 0.03
    angular_kd: float = 0.0005
    gravity_compensation: bool = True
    pad_half_width: float = 0.020
    ledge: bool = True
    center: tuple[float, float, float] = (0.0, 0.0, 0.02)
    half_gap: float = 0.015
    closed_half_gap: float = 0.005
    release_half_gap: float = 0.070
    approach_z: float = -0.030
    travel_x: float = 0.18
    lift_z: float = 0.40
    lower_offset_z: float = 0.295
    release_lower_z: float = 0.17
    control_hz: int = 30
    substeps: int = 20
    duration: float = 13.0

    def __post_init__(self):
        if self.mode not in ("dynamic", "kinematic"):
            raise ValueError("gripper mode must be dynamic or kinematic")
        positive = (
            self.finger_mass,
            self.force_limit,
            self.torque_limit,
            self.control_hz,
            self.substeps,
            self.duration,
            self.pad_half_width,
        )
        gains = (self.linear_ke, self.linear_kd, self.angular_ke, self.angular_kd)
        if not all(np.isfinite(v) and v > 0 for v in positive):
            raise ValueError("gripper mass, limits, rates and duration must be finite and positive")
        if not all(np.isfinite(v) and v >= 0 for v in gains):
            raise ValueError("gripper gains must be finite and nonnegative")


@rss.register_entity("scripted_loop_gripper")
class ScriptedGripper(rss.Entity):
    domain = "rigid"
    world_aware = True

    def build(self, builder: newton.ModelBuilder, episode) -> None:
        del episode
        spec = self.spec
        self.center = np.asarray(self.placed((*spec.center, 0.0, 0.0, 0.0, 1.0))[:3], dtype=np.float64)
        cfg = newton.ModelBuilder.ShapeConfig(mu=2.0, ke=1.0e5, kd=25.0, gap=0.001, density=0.0)
        self.bodies, self.finger_shapes, self.shapes_by_side, self._joints = [], [], [], []
        # Explicit solid-finger envelope inertia, independent of proxy density.
        hx, hy, hz = (0.006 if spec.ledge else 0.003), spec.pad_half_width, (0.008 if spec.ledge else 0.006)
        inertia = np.diag(np.array((hy * hy + hz * hz, hx * hx + hz * hz, hx * hx + hy * hy)) * spec.finger_mass / 3)
        for side in (-1, 1):
            pos = self.center + np.array([side * spec.half_gap, 0.0, 0.0])
            self._joints.append(int(builder.joint_count))
            body = builder.add_body(
                xform=wp.transform(pos, wp.quat_identity()),
                mass=spec.finger_mass,
                inertia=wp.mat33(inertia),
                lock_inertia=True,
                is_kinematic=spec.mode == "kinematic",
                label=f"{spec.name}_{side:+d}",
            )
            shape = builder.add_shape_box(
                body,
                hx=0.003,
                hy=spec.pad_half_width,
                hz=0.006,
                cfg=cfg,
                color=(0.16, 0.30, 0.39),
                label=f"{spec.name}_{side:+d}_pad",
            )
            self.bodies.append(body)
            self.finger_shapes.append(shape)
            side_shapes = [shape]
            if spec.ledge:
                ledge = builder.add_shape_box(
                    body,
                    xform=wp.transform(wp.vec3(0.0, 0.0, -0.0068), wp.quat_identity()),
                    hx=0.006,
                    hy=spec.pad_half_width,
                    hz=0.0012,
                    cfg=cfg,
                    color=(0.16, 0.30, 0.39),
                    label=f"{spec.name}_{side:+d}_ledge",
                )
                self.finger_shapes.append(ledge)
                side_shapes.append(ledge)
            self.shapes_by_side.append(tuple(side_shapes))

    def bind(self, model) -> None:
        super().bind(model)
        self._model = self.stepping_model if self.stepping_model is not None else model
        self.num_worlds = int(getattr(self.worlds, "num_worlds", 1))
        bodies = self.worlds.spread_bodies(self.bodies) if self.worlds is not None else np.asarray([self.bodies])
        coords = np.asarray(model.joint_q_start.numpy()[self._joints], dtype=np.int32)
        dofs = np.asarray(model.joint_qd_start.numpy()[self._joints], dtype=np.int32)
        coords = self.worlds.spread_coords(coords) if self.worlds is not None else coords[None]
        dofs = self.worlds.spread_dofs(dofs) if self.worlds is not None else dofs[None]
        self.bodies_by_world = bodies
        device = self._model.device
        self._body_indices = wp.array(bodies, dtype=int, device=device)
        self._coords, self._dofs = wp.array(coords, dtype=int, device=device), wp.array(dofs, dtype=int, device=device)
        initial = self._model.body_q.numpy()[bodies].astype(np.float32)
        self.initial_poses = wp.array(initial, dtype=wp.transform, device=device)
        self.target_poses = wp.clone(self.initial_poses)
        self.target_velocities = wp.zeros((self.num_worlds, 2), dtype=wp.spatial_vector, device=device)
        self.applied_wrenches = wp.zeros_like(self.target_velocities)
        self.peak_force = wp.zeros((self.num_worlds, 2), dtype=float, device=device)
        self.peak_torque = wp.zeros_like(self.peak_force)
        self._tick = wp.zeros(self.num_worlds, dtype=int, device=device)

    def on_scene_ready(self, pipeline) -> None:
        self.pipeline = pipeline
        self.dt = float(pipeline.sim_dt)
        self.capacity = int(np.ceil(self.spec.duration / self.dt)) + 2
        device = self._model.device
        self._poses = wp.zeros((self.num_worlds, self.capacity, 2), dtype=wp.transform, device=device)
        self._velocities = wp.zeros((self.num_worlds, self.capacity, 2), dtype=wp.spatial_vector, device=device)
        self._lengths = wp.ones(self.num_worlds, dtype=int, device=device)
        self._enabled = wp.zeros(self.num_worlds, dtype=int, device=device)
        for world in range(self.num_worlds):
            if self.spec.scripted:
                self.set_trajectory(self.legacy_trajectory(), world=world)

    def legacy_trajectory(self) -> np.ndarray:
        """Original path, sampled at physics cadence; callers can replace it."""
        spec = self.spec
        poses = np.zeros((self.capacity, 2, 7), dtype=np.float32)
        poses[:, :, 6] = 1.0
        for index in range(self.capacity):
            time = index * self.dt
            close = smooth(time, 0.40, 0.85)
            grasp_gap = spec.half_gap + (spec.closed_half_gap - spec.half_gap) * close
            left_gap = grasp_gap + (spec.release_half_gap - grasp_gap) * smooth(time, 8.0, 8.5)
            right_gap = grasp_gap + (spec.release_half_gap - grasp_gap) * smooth(time, 10.2, 10.7)
            approach = spec.approach_z * smooth(time, 0.05, 0.35)
            lower = smooth(time, 5.6, 7.2)
            lift = spec.lift_z * smooth(time, 1.2, 2.8) * (1.0 - lower)
            move = spec.travel_x * smooth(time, 3.0, 4.8)
            retreat = smooth(time, 11.3, 12.1)
            release_lower = spec.release_lower_z * smooth(time, 8.5, 9.4)
            center = self.center + np.array(
                [move, -0.12 * retreat, approach + lift + spec.lower_offset_z * lower - release_lower + 0.08 * retreat]
            )
            poses[index, 0, :3] = center + np.array([-left_gap, 0.0, 0.0])
            poses[index, 1, :3] = center + np.array([right_gap, 0.0, 0.0])
            tip = smooth(time, 9.4, 10.2) * (1.0 - smooth(time, 10.7, 11.2))
            angle = -0.5 * np.pi * tip
            poses[index, 1, 4], poses[index, 1, 6] = np.sin(angle / 2.0), np.cos(angle / 2.0)
        return poses

    @staticmethod
    def _validate_poses(poses, shape):
        poses = np.asarray(poses, dtype=np.float32).copy()
        if poses.shape != shape or not np.isfinite(poses).all():
            raise ValueError(f"expected finite gripper poses with shape {shape}")
        norms = np.linalg.norm(poses[..., 3:], axis=-1, keepdims=True)
        if np.any(norms < 1.0e-8):
            raise ValueError("target quaternions cannot be zero")
        poses[..., 3:] /= norms
        return poses

    def _world(self, world):
        world = int(world)
        if not 0 <= world < self.num_worlds:
            raise IndexError(f"world {world} outside {self.num_worlds} gripper worlds")
        return world

    def _write_world(self, array, values, world, *, dtype, stride):
        source = wp.array(values, dtype=dtype, device=self._model.device)
        wp.copy(array, source, dest_offset=world * stride, count=source.size)

    def set_targets(self, poses, velocities=None, *, world=0) -> None:
        """Set two world-frame poses [2,7] and optional linear/angular rates [2,6].

        Disables scripted playback only for this world. With scripted=False,
        policies may also write public [world,finger] target_poses and
        target_velocities device buffers directly. Physical state is untouched.
        """
        world = self._world(world)
        poses = self._validate_poses(poses, (2, 7))
        rates = np.zeros((2, 6), dtype=np.float32) if velocities is None else np.asarray(velocities, dtype=np.float32)
        if rates.shape != (2, 6) or not np.isfinite(rates).all():
            raise ValueError("expected finite target velocities with shape (2, 6)")
        self._write_world(self.target_poses, poses, world, dtype=wp.transform, stride=2)
        self._write_world(self.target_velocities, rates, world, dtype=wp.spatial_vector, stride=2)
        self._write_world(self._enabled, [0], world, dtype=int, stride=1)

    def set_trajectory(self, poses, *, world=0) -> None:
        """Replace [N,2,7] targets at self.dt cadence; hold the final pose at rest.

        Reuses fixed buffers without rebuilding the scene or CUDA graph. N must
        fit self.capacity, determined by spec.duration and the physics timestep.
        """
        world = self._world(world)
        poses = np.asarray(poses, dtype=np.float32)
        if poses.ndim != 3 or not 1 <= len(poses) <= self.capacity:
            raise ValueError(f"trajectory needs 1 to {self.capacity} samples")
        poses = self._validate_poses(poses, (len(poses), 2, 7))
        rates = trajectory_velocities(poses, self.dt)
        self._write_world(self._poses, poses, world, dtype=wp.transform, stride=self.capacity * 2)
        self._write_world(self._velocities, rates, world, dtype=wp.spatial_vector, stride=self.capacity * 2)
        self._write_world(self._lengths, [len(poses)], world, dtype=int, stride=1)
        self._write_world(self._tick, [0], world, dtype=int, stride=1)
        self._write_world(self._enabled, [1], world, dtype=int, stride=1)

    def contribute_hooks(self, hooks) -> None:
        phase = "rigid_force" if self.spec.mode == "dynamic" else "pre_solve"
        hooks.add(phase, self._drive, name="drive-loop-fingers")
        hooks.add("post_solve", self._advance, name="advance-loop-fingers")

    def _drive(self, state, dt) -> None:
        del dt
        wp.launch(
            _sample_targets,
            dim=(self.num_worlds, 2),
            inputs=[
                self._tick,
                self._enabled,
                self._lengths,
                self._poses,
                self._velocities,
                self.target_poses,
                self.target_velocities,
            ],
            device=self._model.device,
        )
        if self.spec.mode == "kinematic":
            wp.launch(
                _drive_kinematic,
                dim=(self.num_worlds, 2),
                inputs=[
                    self._body_indices,
                    self.target_poses,
                    self.target_velocities,
                    state.body_q,
                    self.pipeline.state_1.body_q,
                    state.body_qd,
                    self.pipeline.state_1.body_qd,
                ],
                device=self._model.device,
            )
        else:
            spec, model = self.spec, self._model
            wp.launch(
                _drive_dynamic,
                dim=(self.num_worlds, 2),
                inputs=[
                    self._body_indices,
                    self.target_poses,
                    self.target_velocities,
                    state.body_q,
                    state.body_qd,
                    model.body_com,
                    model.body_mass,
                    model.body_world,
                    model.gravity,
                    spec.linear_ke,
                    spec.linear_kd,
                    spec.angular_ke,
                    spec.angular_kd,
                    spec.force_limit,
                    spec.torque_limit,
                    spec.gravity_compensation,
                    state.body_f,
                    self.applied_wrenches,
                    self.peak_force,
                    self.peak_torque,
                ],
                device=model.device,
            )

    def _advance(self, state, dt) -> None:
        del state, dt
        wp.launch(
            _advance_tick,
            dim=self.num_worlds,
            inputs=[
                self._tick,
                self._enabled,
                self._lengths,
            ],
            device=self._model.device,
        )

    def reset(self, pipeline, episode, *, world: int | None = None) -> None:
        del episode
        # Sim.reset_scene calls once with no world; wave resets pass one world.
        # Both cases must restore controls/body state without touching neighbors.
        selected = range(self.num_worlds) if world is None else (self._world(world),)
        a, b = pipeline.state_0, pipeline.state_1
        for index in selected:
            wp.launch(
                _reset_world,
                dim=2,
                inputs=[
                    index,
                    self._body_indices,
                    self._coords,
                    self._dofs,
                    self.initial_poses,
                    self.target_poses,
                    self.target_velocities,
                    self.applied_wrenches,
                    self.peak_force,
                    self.peak_torque,
                    self._tick,
                    a.body_q,
                    b.body_q,
                    a.body_qd,
                    b.body_qd,
                    a.body_f,
                    b.body_f,
                    a.joint_q,
                    b.joint_q,
                    a.joint_qd,
                    b.joint_qd,
                ],
                device=self._model.device,
            )
            if not self.spec.scripted:
                # A policy reset holds the initial targets until a new command.
                self._write_world(self._enabled, [0], index, dtype=int, stride=1)
