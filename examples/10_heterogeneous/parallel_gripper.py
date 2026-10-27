# SPDX-License-Identifier: Apache-2.0
"""Experimental finite-authority parallel jaws with a shared carrier.

Only the carrier receives a world wrench. Two native prismatic joints constrain
finger height/attitude. MuJoCo caps its native motors; the VBD route clamps
explicit joint feedforward effort with native equal/opposite carrier reactions.
The object is never attached. Target poses remain two world transforms, as in
gripper.py, but incompatible independent-finger commands are rejected.
"""

from __future__ import annotations

from dataclasses import dataclass

import newton
import numpy as np
import warp as wp
from gripper import ScriptedGripper, _advance_tick, _sample_targets

import robosimstudio as rss


@dataclass(frozen=True, kw_only=True)
class ParallelGripperSpec:
    entity: str = "parallel_loop_gripper"
    name: str = "parallel_loop_gripper"
    mode: str = "dynamic"
    motor_backend: str = "mujoco"
    scripted: bool = False
    center: tuple[float, float, float] = (0.0, 0.0, 0.02)
    half_gap: float = 0.015
    minimum_half_gap: float = 0.0032
    maximum_half_gap: float = 0.060
    pad_half_width: float = 0.006
    ledge: bool = False
    carrier_mass: float = 0.2
    finger_mass: float = 0.05
    carrier_half_extents: tuple[float, float, float] = (0.060, 0.015, 0.009)
    carrier_offset_z: float = 0.045
    carriage_force_limit: float = 10.0
    carriage_torque_limit: float = 0.1
    jaw_force_limit: float = 5.0
    carriage_linear_ke: float = 1500.0
    carriage_linear_kd: float = 42.0
    carriage_angular_ke: float = 1.0
    carriage_angular_kd: float = 0.01
    jaw_ke: float = 1500.0
    jaw_kd: float = 15.0
    gravity_compensation: bool = True
    control_hz: int = 30
    substeps: int = 20
    duration: float = 13.0

    @property
    def force_limit(self):
        """Policy compatibility: carriage cap, not a finger world-force cap."""
        return self.carriage_force_limit

    @property
    def torque_limit(self):
        return self.carriage_torque_limit

    def __post_init__(self):
        if self.motor_backend not in ("mujoco", "vbd"):
            raise ValueError("motor_backend must be mujoco or vbd")
        if self.mode != "dynamic" or self.ledge:
            raise ValueError("mechanical parallel gripper requires dynamic flat fingers without ledges")
        dimensions = np.asarray(self.carrier_half_extents, dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("carrier_half_extents must contain three finite positive dimensions")
        positive = (
            self.carrier_mass,
            self.finger_mass,
            self.carriage_force_limit,
            self.carriage_torque_limit,
            self.jaw_force_limit,
            self.pad_half_width,
            self.carrier_offset_z,
            self.control_hz,
            self.substeps,
            self.duration,
        )
        gains = (
            self.carriage_linear_ke,
            self.carriage_linear_kd,
            self.carriage_angular_ke,
            self.carriage_angular_kd,
            self.jaw_ke,
            self.jaw_kd,
        )
        if not np.isfinite(positive).all() or min(positive) <= 0 or not np.isfinite(gains).all() or min(gains) < 0:
            raise ValueError("masses, limits, dimensions and rates must be positive; gains nonnegative")
        if not np.isclose(self.pad_half_width, 0.006):
            raise ValueError("this experiment retains the 12 mm-wide flat fingertips")
        if not 0.003 < self.minimum_half_gap <= self.half_gap <= self.maximum_half_gap:
            raise ValueError("initial half-gap must lie within positive nonintersecting slide limits")
        if not np.isfinite(self.maximum_half_gap) or not np.isfinite(self.center).all():
            raise ValueError("initial geometry must be finite")


@wp.kernel
def _drive_carrier_and_slides(
    bodies: wp.array2d[int],
    slide_targets: wp.array2d[int],
    slide_dofs: wp.array2d[int],
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
    target_q: wp.array[float],
    target_qd: wp.array[float],
    applied: wp.array2d[wp.spatial_vector],
    peak_force: wp.array2d[float],
    peak_torque: wp.array2d[float],
):
    world = wp.tid()
    carrier = bodies[world, 0]
    left, right = targets[world, 0], targets[world, 1]
    rotation = wp.transform_get_rotation(left)
    midpoint = (wp.transform_get_translation(left) + wp.transform_get_translation(right)) * 0.5
    axis = wp.quat_rotate(rotation, wp.vec3(1.0, 0.0, 0.0))
    separation = wp.transform_get_translation(right) - wp.transform_get_translation(left)
    half_gap = wp.dot(separation, axis) * 0.5
    target_linear = (wp.spatial_top(rates[world, 0]) + wp.spatial_top(rates[world, 1])) * 0.5
    target_angular = (wp.spatial_bottom(rates[world, 0]) + wp.spatial_bottom(rates[world, 1])) * 0.5
    gap_rate = wp.dot(wp.spatial_top(rates[world, 1]) - wp.spatial_top(rates[world, 0]), axis) * 0.5
    for finger in range(2):
        target_q[slide_targets[world, finger]] = half_gap
        target_qd[slide_dofs[world, finger]] = gap_rate

    current = body_q[carrier]
    com = wp.transform_point(current, body_com[carrier])
    target_com = midpoint + wp.quat_rotate(rotation, body_com[carrier])
    target_com_rate = target_linear + wp.cross(target_angular, wp.quat_rotate(rotation, body_com[carrier]))
    velocity = body_qd[carrier]
    force = linear_ke * (target_com - com) + linear_kd * (target_com_rate - wp.spatial_top(velocity))
    error = rotation * wp.quat_inverse(wp.transform_get_rotation(current))
    if error[3] < 0.0:
        error = wp.quat(-error[0], -error[1], -error[2], -error[3])
    axis_error = wp.vec3(error[0], error[1], error[2])
    sine = wp.length(axis_error)
    rotation_vector = wp.vec3()
    if sine > 1.0e-8:
        rotation_vector = axis_error * (2.0 * wp.atan2(sine, error[3]) / sine)
    torque = angular_ke * rotation_vector + angular_kd * (target_angular - wp.spatial_bottom(velocity))
    if compensate_gravity:
        g = gravity[body_world[carrier]]
        for link in range(3):
            body = bodies[world, link]
            own_weight = body_mass[body] * g
            force -= own_weight
            link_com = wp.transform_point(body_q[body], body_com[body])
            torque -= wp.cross(link_com - com, own_weight)
    # Both own-weight compensation and payload tracking use these same caps.
    norm_f, norm_t = wp.length(force), wp.length(torque)
    if norm_f > force_limit:
        force *= force_limit / norm_f
    if norm_t > torque_limit:
        torque *= torque_limit / norm_t
    wrench = wp.spatial_vector(force, torque)
    wp.atomic_add(body_f, carrier, wrench)
    applied[world, 0] = wrench
    peak_force[world, 0] = wp.max(peak_force[world, 0], wp.length(force))
    peak_torque[world, 0] = wp.max(peak_torque[world, 0], wp.length(torque))


@wp.kernel
def _measure_motors(
    local_dofs: wp.array2d[int],
    qfrc_actuator: wp.array[float],
    measured: wp.array2d[float],
    peaks: wp.array2d[float],
    available: wp.array[int],
):
    world, finger = wp.tid()
    effort = qfrc_actuator[local_dofs[world, finger]]
    measured[world, finger] = effort
    if wp.isfinite(effort):
        peaks[world, finger] = wp.max(peaks[world, finger], wp.abs(effort))
    if finger == 0:
        available[world] = int(
            wp.isfinite(qfrc_actuator[local_dofs[world, 0]]) and wp.isfinite(qfrc_actuator[local_dofs[world, 1]])
        )


@wp.func
def _slide_state(
    carrier: int,
    child: int,
    side: float,
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
):
    # Anchors are identity transforms in this mechanism. Velocities are at COM.
    parent_pose, child_pose = body_q[carrier], body_q[child]
    parent_origin = wp.transform_get_translation(parent_pose)
    child_origin = wp.transform_get_translation(child_pose)
    axis = wp.quat_rotate(wp.transform_get_rotation(parent_pose), wp.vec3(side, 0.0, 0.0))
    displacement = child_origin - parent_origin
    coordinate = wp.dot(displacement, axis)
    parent_omega = wp.spatial_bottom(body_qd[carrier])
    parent_velocity = wp.spatial_top(body_qd[carrier]) + wp.cross(
        parent_omega, parent_origin - wp.transform_point(parent_pose, body_com[carrier])
    )
    child_velocity = wp.spatial_top(body_qd[child]) + wp.cross(
        wp.spatial_bottom(body_qd[child]), child_origin - wp.transform_point(child_pose, body_com[child])
    )
    rate = wp.dot(child_velocity - parent_velocity - wp.cross(parent_omega, displacement), axis)
    lateral = wp.length(displacement - coordinate * axis)
    cosine = wp.abs(wp.dot(wp.transform_get_rotation(parent_pose), wp.transform_get_rotation(child_pose)))
    angle = 2.0 * wp.acos(wp.clamp(cosine, 0.0, 1.0))
    return coordinate, rate, lateral, angle


@wp.kernel
def _drive_vbd_slides(
    bodies: wp.array2d[int],
    slide_targets: wp.array2d[int],
    slide_dofs: wp.array2d[int],
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    target_q: wp.array[float],
    target_qd: wp.array[float],
    ke: float,
    kd: float,
    limit: float,
    joint_f: wp.array[float],
    applied: wp.array2d[float],
    peaks: wp.array2d[float],
    available: wp.array[int],
):
    world, finger = wp.tid()
    coordinate, rate, _lateral, _angle = _slide_state(
        bodies[world, 0], bodies[world, finger + 1], float(2 * finger - 1), body_q, body_qd, body_com
    )
    dof = slide_dofs[world, finger]
    effort = wp.clamp(
        ke * (target_q[slide_targets[world, finger]] - coordinate) + kd * (target_qd[dof] - rate), -limit, limit
    )
    # SolverVBD's apply_joint_forces applies this at the joint anchors with the
    # equal/opposite parent wrench, including the carrier COM moment arm.
    joint_f[dof] = effort
    applied[world, finger] = effort
    peaks[world, finger] = wp.max(peaks[world, finger], wp.abs(effort))
    if finger == 0:
        available[world] = 1


@wp.kernel
def _measure_mechanism(
    bodies: wp.array2d[int],
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    coordinates: wp.array2d[float],
    rates: wp.array2d[float],
    lateral_error: wp.array2d[float],
    angular_error: wp.array2d[float],
):
    world, finger = wp.tid()
    coordinate, rate, lateral, angle = _slide_state(
        bodies[world, 0], bodies[world, finger + 1], float(2 * finger - 1), body_q, body_qd, body_com
    )
    coordinates[world, finger] = coordinate
    rates[world, finger] = rate
    lateral_error[world, finger] = lateral
    angular_error[world, finger] = angle


@wp.kernel
def _reset_mechanism(
    world: int,
    bodies: wp.array2d[int],
    coords: wp.array2d[int],
    dofs: wp.array2d[int],
    slide_targets: wp.array2d[int],
    slide_dofs: wp.array2d[int],
    motor_dofs: wp.array2d[int],
    initial_bodies: wp.array2d[wp.transform],
    initial_joints: wp.array2d[float],
    initial_fingers: wp.array2d[wp.transform],
    targets: wp.array2d[wp.transform],
    rates: wp.array2d[wp.spatial_vector],
    path: wp.array3d[wp.transform],
    path_rates: wp.array3d[wp.spatial_vector],
    tick: wp.array[int],
    lengths: wp.array[int],
    enabled: wp.array[int],
    scripted: bool,
    applied: wp.array2d[wp.spatial_vector],
    peak_force: wp.array2d[float],
    peak_torque: wp.array2d[float],
    motor_efforts: wp.array2d[float],
    peak_motor_efforts: wp.array2d[float],
    motor_available: wp.array[int],
    observed_efforts: wp.array[float],
    unavailable: float,
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
    target_q: wp.array[float],
    target_qd: wp.array[float],
    joint_f: wp.array[float],
):
    link = wp.tid()
    body = bodies[world, link]
    q0[body] = initial_bodies[world, link]
    q1[body] = initial_bodies[world, link]
    qd0[body] = wp.spatial_vector()
    qd1[body] = wp.spatial_vector()
    f0[body] = wp.spatial_vector()
    f1[body] = wp.spatial_vector()
    if link < 2:
        finger = link
        targets[world, finger] = initial_fingers[world, finger]
        rates[world, finger] = wp.spatial_vector()
        path[world, 0, finger] = initial_fingers[world, finger]
        path_rates[world, 0, finger] = wp.spatial_vector()
        target_q[slide_targets[world, finger]] = initial_joints[world, 7 + finger]
        target_qd[slide_dofs[world, finger]] = 0.0
        motor_efforts[world, finger] = unavailable
        peak_motor_efforts[world, finger] = 0.0
        if observed_efforts:
            observed_efforts[motor_dofs[world, finger]] = unavailable
    if link == 0:
        for i in range(9):
            jq0[coords[world, i]] = initial_joints[world, i]
            jq1[coords[world, i]] = initial_joints[world, i]
        for i in range(8):
            jqd0[dofs[world, i]] = 0.0
            jqd1[dofs[world, i]] = 0.0
            joint_f[dofs[world, i]] = 0.0
        tick[world] = 0
        lengths[world] = 1
        enabled[world] = int(scripted)
        applied[world, 0] = wp.spatial_vector()
        peak_force[world, 0] = 0.0
        peak_torque[world, 0] = 0.0
        motor_available[world] = 0


@rss.register_entity("parallel_loop_gripper")
class ParallelGripper(ScriptedGripper):
    """Mechanical jaws with explicit source motor semantics.

    For MuJoCo, pass ``motor_observables`` to the source ``step`` before capture.
    Without that routing, native motor readings are NaN and unavailable. VBD
    reports its explicitly applied bounded feedforward effort, not an observable.
    ``bodies`` always contains the two fingers; ``carrier_body`` is separate.
    """

    domain = "rigid"
    world_aware = True

    def build(self, builder, episode):
        del episode
        spec = self.spec
        self.center = np.asarray(self.placed((*spec.center, 0.0, 0.0, 0.0, 1.0))[:3], dtype=float)
        half = np.asarray(spec.carrier_half_extents)
        self.carrier_inertia = (
            spec.carrier_mass
            / 3
            * np.array((half[1] ** 2 + half[2] ** 2, half[0] ** 2 + half[2] ** 2, half[0] ** 2 + half[1] ** 2))
        )
        self.carrier_body = builder.add_link(
            xform=wp.transform(self.center, wp.quat_identity()),
            com=wp.vec3(0.0, 0.0, spec.carrier_offset_z),
            mass=spec.carrier_mass,
            inertia=wp.mat33(np.diag(self.carrier_inertia)),
            lock_inertia=True,
            label=f"{spec.name}_carrier",
        )
        cfg = newton.ModelBuilder.ShapeConfig(mu=2.0, ke=1.0e5, kd=25.0, gap=0.001, density=0.0)
        self.carrier_shape = builder.add_shape_box(
            self.carrier_body,
            xform=wp.transform(wp.vec3(0.0, 0.0, spec.carrier_offset_z), wp.quat_identity()),
            hx=float(half[0]),
            hy=float(half[1]),
            hz=float(half[2]),
            cfg=cfg,
            color=(0.16, 0.30, 0.39),
            label=f"{spec.name}_carrier_bar",
        )
        self.joints = [builder.add_joint_free(self.carrier_body, label=f"{spec.name}_free")]
        self.bodies, self.finger_shapes, self.shapes_by_side = [], [], []
        finger_inertia = (
            spec.finger_mass
            / 3
            * np.array((spec.pad_half_width**2 + 0.006**2, 0.003**2 + 0.006**2, 0.003**2 + spec.pad_half_width**2))
        )
        for sign in (-1, 1):
            body = builder.add_link(
                xform=wp.transform(self.center + np.array((sign * spec.half_gap, 0.0, 0.0)), wp.quat_identity()),
                mass=spec.finger_mass,
                inertia=wp.mat33(np.diag(finger_inertia)),
                lock_inertia=True,
                label=f"{spec.name}_finger_{sign:+d}",
            )
            joint = builder.add_joint_prismatic(
                self.carrier_body,
                body,
                axis=wp.vec3(float(sign), 0.0, 0.0),
                target_pos=spec.half_gap,
                target_ke=spec.jaw_ke if spec.motor_backend == "mujoco" else 0.0,
                target_kd=spec.jaw_kd if spec.motor_backend == "mujoco" else 0.0,
                effort_limit=spec.jaw_force_limit,
                actuator_mode=newton.JointTargetMode.POSITION_VELOCITY,
                limit_lower=spec.minimum_half_gap,
                limit_upper=spec.maximum_half_gap,
                damping=0.0,
                friction=0.0,
                label=f"{spec.name}_slide_{sign:+d}",
            )
            builder.joint_q[builder.joint_q_start[joint]] = spec.half_gap
            self.joints.append(joint)
            shape = builder.add_shape_box(
                body,
                hx=0.003,
                hy=spec.pad_half_width,
                hz=0.006,
                cfg=cfg,
                color=(0.16, 0.30, 0.39),
                label=f"{spec.name}_pad_{sign:+d}",
            )
            self.bodies.append(body)
            self.finger_shapes.append(shape)
            self.shapes_by_side.append((shape,))
        builder.add_articulation(self.joints, label=f"{spec.name}_mechanism")
        self.all_bodies = [self.carrier_body, *self.bodies]

    def bind(self, model):
        rss.Entity.bind(self, model)
        self._model = self.stepping_model if self.stepping_model is not None else model
        self.num_worlds = int(getattr(self.worlds, "num_worlds", 1))
        spread = (
            self.worlds.spread_bodies(self.all_bodies) if self.worlds is not None else np.asarray([self.all_bodies])
        )
        joints = self.worlds.spread_joints(self.joints) if self.worlds is not None else np.asarray([self.joints])
        qstarts = self._model.joint_q_start.numpy()[joints]
        dstarts = self._model.joint_qd_start.numpy()[joints]
        self.finger_target_coords_by_world = self._model.joint_target_q_start.numpy()[joints[:, 1:]]
        self.finger_dofs_by_world = dstarts[:, 1:]
        coords = np.concatenate((qstarts[:, :1] + np.arange(7)[None], qstarts[:, 1:]), axis=1)
        dofs = np.concatenate((dstarts[:, :1] + np.arange(6)[None], dstarts[:, 1:]), axis=1)
        device = self._model.device
        self._all_body_indices = wp.array(spread, dtype=int, device=device)
        self.bodies_by_world, self.carrier_bodies_by_world = spread[:, 1:], spread[:, 0]
        self._joint_indices_by_world = joints
        self._coords, self._dofs = wp.array(coords, dtype=int, device=device), wp.array(dofs, dtype=int, device=device)
        self._slide_targets = wp.array(self.finger_target_coords_by_world, dtype=int, device=device)
        self._slide_dofs = wp.array(self.finger_dofs_by_world, dtype=int, device=device)
        initial = self._model.body_q.numpy()[spread].astype(np.float32)
        self.initial_body_poses = wp.array(initial, dtype=wp.transform, device=device)
        self.initial_poses = wp.array(initial[:, 1:], dtype=wp.transform, device=device)
        self.initial_joint_values = wp.array(self._model.joint_q.numpy()[coords], dtype=float, device=device)
        self.target_poses = wp.clone(self.initial_poses)
        self.target_velocities = wp.zeros((self.num_worlds, 2), dtype=wp.spatial_vector, device=device)
        self.applied_carriage_wrench = wp.zeros((self.num_worlds, 1), dtype=wp.spatial_vector, device=device)
        self.peak_carriage_force = wp.zeros((self.num_worlds, 1), dtype=float, device=device)
        self.peak_carriage_torque = wp.zeros_like(self.peak_carriage_force)
        self.motor_efforts = wp.full((self.num_worlds, 2), float("nan"), dtype=float, device=device)
        self.peak_motor_effort = wp.zeros_like(self.motor_efforts)
        self.motor_efforts_available = wp.zeros(self.num_worlds, dtype=int, device=device)
        self._tick = wp.zeros(self.num_worlds, dtype=int, device=device)
        self.actual_half_gaps = wp.zeros((self.num_worlds, 2), dtype=float, device=device)
        self.actual_gap_rates = wp.zeros_like(self.actual_half_gaps)
        self.slide_lateral_error = wp.zeros_like(self.actual_half_gaps)
        self.slide_angular_error = wp.zeros_like(self.actual_half_gaps)

    def on_scene_ready(self, pipeline):
        self.pipeline, self.dt = pipeline, float(pipeline.sim_dt)
        spec, device = self.spec, self._model.device
        if (
            spec.carriage_angular_ke * self.dt**2 + 2 * spec.carriage_angular_kd * self.dt
            >= 4 * self.carrier_inertia.min()
        ):
            raise ValueError("external carrier angular PD exceeds its conservative isolated explicit stability bound")
        self.capacity = int(np.ceil(spec.duration / self.dt)) + 2
        self._poses = wp.zeros((self.num_worlds, self.capacity, 2), dtype=wp.transform, device=device)
        self._velocities = wp.zeros((self.num_worlds, self.capacity, 2), dtype=wp.spatial_vector, device=device)
        self._lengths = wp.ones(self.num_worlds, dtype=int, device=device)
        self._enabled = wp.zeros(self.num_worlds, dtype=int, device=device)
        solver = pipeline.program.solver_for(self.name)
        self.motor_observables = None
        if spec.motor_backend == "vbd":
            if not isinstance(solver, newton.solvers.SolverVBD):
                raise ValueError("VBD parallel gripper requires a SolverVBD backend (including subclasses)")
            if not pipeline.program.coupled:
                if self._model is not pipeline.model:
                    raise ValueError("uncoupled gripper reset indices must address the main pipeline model")
                # Compiler _start_pose has evaluated FK before ready hooks.
                # Full Sim.reset evaluates it again; retain the same rounded
                # transforms for masked reset without running FK on neighbors.
                # Authored command targets intentionally remain unchanged.
                indices = self._all_body_indices.numpy()
                self.initial_body_poses.assign(pipeline.state_0.body_q.numpy()[indices])
            self.motor_effort_kind = "applied_external_joint_effort"
            self.motor_dofs_by_world = self.finger_dofs_by_world.copy()
            self._motor_dofs = self._slide_dofs
        else:
            if not isinstance(solver, newton.solvers.SolverMuJoCo):
                raise ValueError("parallel_loop_gripper requires a native MuJoCo source backend")
            view = pipeline.program.view_for(self.name)
            self.motor_observables = solver.observables({solver.ObservableKind.QFRC_ACTUATOR})
            self.motor_observables.qfrc_actuator.fill_(float("nan"))
            # Model views compact DOFs. Labels plus world disambiguate replicas.
            local_keys = list(zip(view.joint_world.numpy().tolist(), view.joint_label, strict=True))
            global_worlds = self._model.joint_world.numpy()
            local_dofs = np.empty((self.num_worlds, 2), dtype=np.int32)
            for world in range(self.num_worlds):
                for finger, joint in enumerate(self._joint_indices_by_world[world, 1:]):
                    key = (int(global_worlds[joint]), self._model.joint_label[joint])
                    matches = [index for index, candidate in enumerate(local_keys) if candidate == key]
                    if len(matches) != 1:
                        raise ValueError(f"cannot uniquely resolve native slide observable for {key}")
                    local_dofs[world, finger] = view.joint_qd_start.numpy()[matches[0]]
            self.motor_dofs_by_world = local_dofs
            self._motor_dofs = wp.array(local_dofs, dtype=int, device=device)
            self.motor_effort_kind = "native_QFRC_ACTUATOR"
        for world in range(self.num_worlds):
            if spec.scripted:
                self.set_trajectory(self.initial_poses.numpy()[world][None], world=world)

    def _validate_poses(self, poses, shape):
        poses = ScriptedGripper._validate_poses(poses, shape)
        pairs = poses.reshape((-1, 2, 7))
        qa, qb = pairs[:, 0, 3:], pairs[:, 1, 3:]
        sign = np.where(np.sum(qa * qb, axis=1) < 0, -1.0, 1.0)
        if np.any(np.linalg.norm(qa - qb * sign[:, None], axis=1) > 1.0e-4):
            raise ValueError("parallel targets must have one common orientation")
        x, y, z, w = qa.T
        axes = np.stack((1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)), axis=1)
        delta = pairs[:, 1, :3] - pairs[:, 0, :3]
        gap = np.sum(delta * axes, axis=1) / 2
        if np.any(np.linalg.norm(delta - 2 * gap[:, None] * axes, axis=1) > 1.0e-5):
            raise ValueError("finger separation must be parallel to common local x")
        if np.any(gap < self.spec.minimum_half_gap - 1.0e-6) or np.any(gap > self.spec.maximum_half_gap + 1.0e-6):
            raise ValueError("target half-gap lies outside mechanical slide limits")
        return poses

    def legacy_trajectory(self):
        raise ValueError("independent-finger legacy release is incompatible with mechanical parallel jaws")

    def contribute_hooks(self, hooks):
        hooks.add("rigid_force", self._drive, name="drive-parallel-carrier-and-slides")
        hooks.add("post_solve", self._advance, name="measure-and-advance-parallel-jaws")

    def _drive(self, state, dt):
        del dt
        model, spec = self._model, self.spec
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
            device=model.device,
        )
        wp.launch(
            _drive_carrier_and_slides,
            dim=self.num_worlds,
            inputs=[
                self._all_body_indices,
                self._slide_targets,
                self._slide_dofs,
                self.target_poses,
                self.target_velocities,
                state.body_q,
                state.body_qd,
                model.body_com,
                model.body_mass,
                model.body_world,
                model.gravity,
                spec.carriage_linear_ke,
                spec.carriage_linear_kd,
                spec.carriage_angular_ke,
                spec.carriage_angular_kd,
                spec.carriage_force_limit,
                spec.carriage_torque_limit,
                spec.gravity_compensation,
                state.body_f,
                self.pipeline.control.joint_target_q,
                self.pipeline.control.joint_target_qd,
                self.applied_carriage_wrench,
                self.peak_carriage_force,
                self.peak_carriage_torque,
            ],
            device=model.device,
        )
        if spec.motor_backend == "vbd":
            wp.launch(
                _drive_vbd_slides,
                dim=(self.num_worlds, 2),
                inputs=[
                    self._all_body_indices,
                    self._slide_targets,
                    self._slide_dofs,
                    state.body_q,
                    state.body_qd,
                    model.body_com,
                    self.pipeline.control.joint_target_q,
                    self.pipeline.control.joint_target_qd,
                    spec.jaw_ke,
                    spec.jaw_kd,
                    spec.jaw_force_limit,
                    self.pipeline.control.joint_f,
                    self.motor_efforts,
                    self.peak_motor_effort,
                    self.motor_efforts_available,
                ],
                device=model.device,
            )

    def _advance(self, state, dt):
        del dt
        wp.launch(
            _measure_mechanism,
            dim=(self.num_worlds, 2),
            inputs=[
                self._all_body_indices,
                state.body_q,
                state.body_qd,
                self._model.body_com,
                self.actual_half_gaps,
                self.actual_gap_rates,
                self.slide_lateral_error,
                self.slide_angular_error,
            ],
            device=self._model.device,
        )
        if self.motor_observables is not None:
            wp.launch(
                _measure_motors,
                dim=(self.num_worlds, 2),
                inputs=[
                    self._motor_dofs,
                    self.motor_observables.qfrc_actuator,
                    self.motor_efforts,
                    self.peak_motor_effort,
                    self.motor_efforts_available,
                ],
                device=self._model.device,
            )
        wp.launch(
            _advance_tick,
            dim=self.num_worlds,
            inputs=[self._tick, self._enabled, self._lengths],
            device=self._model.device,
        )

    def reset(self, pipeline, episode, *, world=None):
        """Restore this mechanism only; coupled-object partial reset is unverified."""
        del episode
        selected = range(self.num_worlds) if world is None else (self._world(world),)
        a, b = pipeline.state_0, pipeline.state_1
        for index in selected:
            wp.launch(
                _reset_mechanism,
                dim=3,
                inputs=[
                    index,
                    self._all_body_indices,
                    self._coords,
                    self._dofs,
                    self._slide_targets,
                    self._slide_dofs,
                    self._motor_dofs,
                    self.initial_body_poses,
                    self.initial_joint_values,
                    self.initial_poses,
                    self.target_poses,
                    self.target_velocities,
                    self._poses,
                    self._velocities,
                    self._tick,
                    self._lengths,
                    self._enabled,
                    self.spec.scripted,
                    self.applied_carriage_wrench,
                    self.peak_carriage_force,
                    self.peak_carriage_torque,
                    self.motor_efforts,
                    self.peak_motor_effort,
                    self.motor_efforts_available,
                    self.motor_observables.qfrc_actuator if self.motor_observables is not None else None,
                    float("nan"),
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
                    pipeline.control.joint_target_q,
                    pipeline.control.joint_target_qd,
                    pipeline.control.joint_f,
                ],
                device=self._model.device,
            )
            for array, value in (
                (self.actual_half_gaps, self.spec.half_gap),
                (self.actual_gap_rates, 0.0),
                (self.slide_lateral_error, 0.0),
                (self.slide_angular_error, 0.0),
            ):
                self._write_world(array, [value, value], index, dtype=float, stride=2)
