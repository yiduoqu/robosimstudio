# SPDX-License-Identifier: Apache-2.0
"""Pick up the existing labelled Clawhauser volume by its loop and place it on a table.

Run on a CUDA node with Newton 1.8 development and RoboSimStudio on PYTHONPATH.
The two bounded dynamic fingers use only collision contact; no particle attachment or
direct object motion is added. Outputs are written under ``outputs/clawhauser``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

import newton
import numpy as np
import warp as wp
from gripper import ScriptedGripperSpec

import robosimstudio as rss
from robosimstudio.specs.physics import CouplerSpec
from robosimstudio_materials.heterogeneous import HeterogeneousSoftSpec, load_labelled_tets

ASSET = Path(__file__).resolve().parent / "assets" / "clawhauser"
OUTPUT = Path(__file__).resolve().parents[2] / "outputs" / "clawhauser"
CONTROL_HZ = 30
SUBSTEPS = 20
DURATION = 15.0
TABLE_Z = 0.75
# RSS positions are relative to the bench's work-surface frame, whose origin
# is world x=0.6 m. These values place the toy at world x=0.47 and 0.65 m.
SOURCE_X = -0.13
TARGET_X = 0.05
SOURCE_HASHES = {
    "final_mesh.mesh": "df3d1505d96d36fbfa76291e7ab94269a30500ba5e4b87516d0bf8f57f0c6458",
    "heterogeneous_params.npz": "7cfc8ca3cc7c08d261aa90b08832230801fec1ac62848d0e9239d9e2f318e8c6",
    "inferred_params.json": "45aacd698f2b38a6fd8bf13b46cc9c33497fc90c6c1bdf088ed12d915ec3d992",
    "appearance/visual_to_physics.npz": "4b015d1141794211e4e2c6e2b39fadef6762a73ffe2d76b921e47d5676f3d6c4",
}


def build_scene(config: dict | None = None, *, num_worlds: int = 1):
    config = config or {}
    mechanical = bool(config.get("mechanical_gripper", False))
    mechanical_backend = str(config.get("mechanical_backend", "mujoco"))
    if mechanical_backend not in ("mujoco", "vbd"):
        raise ValueError("mechanical_backend must be mujoco or vbd")
    proxy = mechanical and mechanical_backend == "mujoco"
    substeps = int(config.get("substeps", SUBSTEPS))
    if config.get("deterministic", False):
        from contact_determinism import configure_soft_contacts

        configure_soft_contacts()
    if config.get("full_surface_contact", False):
        # Newton 1.8's generic face kernel can spend minutes in NVRTC's default
        # optimizer on CUDA 13.4. This is a nondifferentiable example; preserve
        # the collision equations while using Warp's supported compile control.
        from newton._src.geometry.soft_contacts_sdf import create_soft_face_contacts

        wp.set_module_options(
            {"mode": "release", "optimization_level": 1, "enable_backward": False},
            module=create_soft_face_contacts.module,
        )
    for relative, expected in SOURCE_HASHES.items():
        actual = hashlib.sha256((ASSET / relative).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"Clawhauser source asset changed: {relative}")
    vertices, tets, labels, elastic, density = load_labelled_tets(
        ASSET / "final_mesh.mesh", ASSET / "heterogeneous_params.npz"
    )
    if (len(vertices), len(tets)) != (1840, 6850) or set(np.unique(labels)) != {0, 1, 2}:
        raise ValueError("Clawhauser source geometry or labels changed")
    if not np.array_equal(np.unique(elastic[:, 0]), [5e5, 5e6]) or not np.array_equal(np.unique(density), [300, 1100]):
        raise ValueError("Clawhauser source material estimates changed")

    # A flat pose makes the loop accessible and the plush body stable on the
    # table. The asset's local +y points toward its loop; rotate 180° about x.
    pose = (SOURCE_X, 0.0, float(vertices[:, 2].max() + 0.002))
    rotation = (1.0, 0.0, 0.0, 0.0)
    with np.load(ASSET / "appearance" / "visual_to_physics.npz") as binding:
        faces = binding["visual_faces"][binding["boundary_face_part_ids"] == 2]
        ring_surface = np.unique(binding["physics_vertex_indices"][faces])
    grasp_ids = ring_surface[vertices[ring_surface, 1] > 0.095]
    if len(grasp_ids) < 8:
        raise ValueError("Clawhauser loop grasp patch is missing")
    local_center = vertices[grasp_ids].mean(axis=0)
    grasp_center = (SOURCE_X + local_center[0], -local_center[1], pose[2] - local_center[2])
    if config.get("trajectory") in ("observed", "cord", "cord_feedback"):
        grasp_center = (*grasp_center[:2], grasp_center[2] + 0.04)
    half_span = np.max(np.abs(vertices[grasp_ids, 0] - local_center[0]))

    coarse = bool(config.get("coarse_translation", False))
    contact_gap = float(config.get("contact_gap_m", 0.002))
    solver_options = (
        {
            "coarse_translation": True,
            "coarse_contact_gap": contact_gap,
            "coarse_budget": float(config.get("coarse_budget_m", 0.00025)),
            "coarse_rigid_translation": bool(config.get("coarse_rigid_translation", False)),
            "coarse_deterministic": bool(config.get("coarse_deterministic", True)),
            "coarse_relative_budget": float(config.get("coarse_relative_budget_m", 0.00025)),
        }
        if coarse
        else {}
    )
    if coarse and config.get("coarse_object_rotation", False):
        solver_options["coarse_object_rotation"] = True
    solver_options["deterministic"] = (
        wp.DeterministicMode.RUN_TO_RUN
        if config.get("solver_deterministic", config.get("deterministic", False))
        else wp.DeterministicMode.NOT_GUARANTEED
    )
    vbd = rss.backend(
        "robosimstudio_materials.vbd_coarse.SolverVBDCoarse" if coarse else "vbd",
        iterations=int(config.get("iterations", 12)),
        friction_epsilon=float(config.get("friction_epsilon_m_s", 0.01)),
        rigid_compliant_alm=bool(config.get("rigid_compliant_alm", False)),
        rigid_joint_linear_ke=float(config.get("rigid_joint_linear_ke", 1.0e5)),
        rigid_joint_angular_ke=float(config.get("rigid_joint_angular_ke", 1.0e5)),
        particle_enable_self_contact=False,
        particle_enable_tile_solve=bool(config.get("tile_solve", False)),
        rigid_body_particle_contact_buffer_size=4096,
        **solver_options,
    )
    rigid = rss.backend("mujoco", use_mujoco_contacts=True, integrator="implicitfast", iterations=20) if proxy else vbd
    coupling = (
        rss.coupler(
            "proxy",
            mode="lagged",
            iterations=int(config.get("proxy_iterations", 1)),
            mass_scale=1.0,
            proxy_relaxation=float(config.get("proxy_relaxation", 0.5)),
            proxy_joints=True,
            collide_interval=1,
        )
        if proxy
        else CouplerSpec()
    )
    physics = rss.ArrangementSpec(
        control_hz=CONTROL_HZ,
        substeps=substeps,
        use_cuda_graph=bool(config.get("graph", True)),
        backends=(("soft", vbd), ("rigid", rigid), ("*", rigid)),
        coupling=coupling,
        collision=rss.CollisionSpec(
            soft_contact_margin=contact_gap,
            full_surface_contact=bool(config.get("full_surface_contact", False)),
            contact_ke=float(config.get("contact_ke", 100000.0)),
            contact_kd=25.0,
            broad_phase="nxn",
            deterministic=bool(config.get("deterministic", False)),
        ),
    )
    scene = rss.Scene(cell="bench_075", physics=physics, name="clawhauser_loop_pick_place")
    scene.add(
        rss.slab(
            name="source_zone",
            half=(0.06, 0.06, 0.00025),
            at=(SOURCE_X, 0.0, 0.001),
            collide=False,
            color=(0.13, 0.28, 0.43),
        )
    )
    scene.add(
        rss.slab(
            name="target_zone",
            half=(0.06, 0.06, 0.00025),
            at=(TARGET_X, -0.10, 0.001),
            collide=False,
            color=(0.10, 0.38, 0.28),
        )
    )
    obj = scene.add(
        HeterogeneousSoftSpec(
            name="clawhauser",
            mesh_path=str(ASSET / "final_mesh.mesh"),
            params_path=str(ASSET / "heterogeneous_params.npz"),
            materials_path=str(ASSET / "inferred_params.json"),
            pose=pose,
            quat=rotation,
            damping_time=0.0005,
            material_overrides={int(k): v for k, v in config.get("material_overrides", {}).items()},
            contact_ke=float(config.get("contact_ke", 100000.0)),
        )
    )
    if mechanical:
        from parallel_gripper import ParallelGripperSpec

        gripper_spec = ParallelGripperSpec(
            motor_backend=mechanical_backend,
            center=grasp_center,
            half_gap=float(half_span + 0.012),
            control_hz=CONTROL_HZ,
            substeps=substeps,
            duration=DURATION,
            pad_half_width=float(config.get("pad_half_width", 0.006)),
        )
    else:
        gripper_spec = ScriptedGripperSpec(
            center=grasp_center,
            mode=config.get("mode", "dynamic"),
            scripted=config.get("scripted", config.get("trajectory") not in ("observed", "cord", "cord_feedback")),
            force_limit=float(config.get("force_limit", 5.0)),
            torque_limit=float(config.get("torque_limit", 0.03)),
            angular_ke=float(config.get("angular_ke", 0.03)),
            angular_kd=float(config.get("angular_kd", 0.0005)),
            pad_half_width=float(config.get("pad_half_width", 0.020)),
            ledge=bool(config.get("ledge", True)),
            half_gap=float(half_span + 0.012),
            closed_half_gap=float(config.get("closed_half_gap", 0.010)),
            travel_x=TARGET_X - SOURCE_X,
            control_hz=CONTROL_HZ,
            substeps=substeps,
            duration=DURATION,
        )
    grip = scene.add(gripper_spec)
    camera = rss.CameraSpec(
        name="overview",
        width=640,
        height=480,
        eye=(0.90, -0.50, 1.30),
        look_at=(0.57, -0.04, 0.95),
        fov_y=50.0,
    )
    scene.add(camera)
    sim = scene.build(
        device="cuda:0", num_worlds=num_worlds, warmup=False, use_cuda_graph=bool(config.get("graph", True))
    )
    if config.get("deterministic", False):
        from contact_determinism import configure_soft_record_capacity

        # Configure the actual main pipeline before first launch. Proxy-owned
        # compacted pipelines are outside this example's validated batch path.
        configure_soft_record_capacity(sim.pipeline.program.collision)
    if "contact_ke" in config:
        sim.model.shape_material_ke.fill_(float(config["contact_ke"]))
    entity = obj.entity
    soft_solver = sim.pipeline.program.solver_for(entity.spec.name)
    if not isinstance(soft_solver, newton.solvers.SolverVBD):
        raise RuntimeError(f"expected Newton VBD for Clawhauser, got {type(soft_solver).__name__}")
    if proxy and not config.get("proxy_joint_drives", True):
        # Lagged proxies already receive the source's motor-driven end velocity.
        # Keep slide constraints/limits, but do not apply a second, uncapped VBD
        # motor. ModelView replacement arrays must not modify the source model.
        program = sim.pipeline.program
        view = program.solver.view(program.island_of(entity.spec.name).name)
        source_ke = sim.model.joint_target_ke.numpy().copy()
        source_kd = sim.model.joint_target_kd.numpy().copy()
        view.joint_target_ke = wp.zeros_like(view.joint_target_ke)
        view.joint_target_kd = wp.zeros_like(view.joint_target_kd)
        soft_solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)
        if not np.array_equal(source_ke, sim.model.joint_target_ke.numpy()) or not np.array_equal(
            source_kd, sim.model.joint_target_kd.numpy()
        ):
            raise RuntimeError("passive proxy drive configuration modified source motor coefficients")
    if sim.model.tet_count != len(tets) * num_worlds or len(entity.tet_part_labels) != len(tets):
        raise RuntimeError("RSS lost Clawhauser tetrahedra or part labels")
    if sim.model.attachment_particle_body_count:
        raise RuntimeError("the contact grasp scene must not contain particle attachments")
    model_tets = sim.model.tet_indices.numpy()[entity.tet_range[0] : entity.tet_range[1]]
    if not np.array_equal(model_tets - entity.particles[0], tets):
        raise RuntimeError("Newton changed the source tetrahedron order")
    young, nu = entity.effective_elastic.T
    source_mu = young / (2.0 * (1.0 + nu))
    source_lambda = young * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    model_elastic = sim.model.tet_materials.numpy()[entity.tet_range[0] : entity.tet_range[1], :2]
    if not np.allclose(model_elastic, np.column_stack((source_mu, source_lambda)), rtol=1e-5):
        raise RuntimeError("Newton FEM elasticity differs from the selected effective profile")
    if not np.allclose(sim.model.particle_mass.numpy()[entity.particles], entity.source_mass, rtol=1e-5):
        raise RuntimeError("Newton nodal mass differs from source per-tet densities")
    return sim, obj, grip, camera, grasp_ids, labels


def _effort_summary(gripper):
    if hasattr(gripper, "applied_carriage_wrench"):
        return {
            "actuation_model": "shared_carriage_and_two_prismatic_jaws",
            "motor_effort_kind": gripper.motor_effort_kind,
            "carriage_force_limit_N": float(gripper.spec.carriage_force_limit),
            "carriage_torque_limit_Nm": float(gripper.spec.carriage_torque_limit),
            "jaw_effort_limit_N": float(gripper.spec.jaw_force_limit),
            "peak_carriage_force_N": gripper.peak_carriage_force.numpy().tolist(),
            "peak_carriage_torque_Nm": gripper.peak_carriage_torque.numpy().tolist(),
            "peak_jaw_motor_effort_N": gripper.peak_motor_effort.numpy().tolist(),
        }
    return {
        "actuation_model": "independent_floating_fingers",
        "peak_applied_force_N": gripper.peak_force.numpy().tolist(),
        "peak_applied_torque_Nm": gripper.peak_torque.numpy().tolist(),
    }


def _mechanism_errors(gripper, body_q):
    if not hasattr(gripper, "carrier_body"):
        return {}
    from cord_grasp import _rotation

    carrier = body_q[gripper.carrier_body]
    fingers = body_q[gripper.bodies]
    axis = _rotation(carrier[3:])[:, 0]
    delta = fingers[:, :3] - carrier[:3]
    axial = delta @ axis
    lateral = np.linalg.norm(delta - axial[:, None] * axis, axis=1)
    a = carrier[3:] / np.linalg.norm(carrier[3:])
    b = fingers[:, 3:] / np.linalg.norm(fingers[:, 3:], axis=1)[:, None]
    angles = np.rad2deg(2 * np.arccos(np.clip(np.abs(b @ a), 0, 1)))
    q = axial * np.array([-1, 1])
    limit_error = np.maximum(np.maximum(gripper.spec.minimum_half_gap - q, q - gripper.spec.maximum_half_gap), 0)
    return {
        "mechanical_joint_lateral_error_m": lateral.tolist(),
        "mechanical_joint_orientation_error_deg": angles.tolist(),
        "mechanical_joint_limit_error_m": limit_error.tolist(),
        "observed_jaw_half_gap_m": q.tolist(),
    }


def _coarse_diagnostics(solver):
    block = getattr(solver, "coarse_block", None)
    if block is None:
        return {}
    # These are snapshots of the last substep, not whole-episode maxima or
    # fresh post-correction force-balance measurements.
    names = ["path_length", "applications", "invalid_world"]
    if block.rigid_translation:
        names += ["rigid_path_length", "relative_path_length", "rigid_applications", "rigid_fallback"]
    if getattr(block, "object_rotation", False):
        names += [
            "rotation_applications",
            "rotation_path_angle",
            "rotation_budget_scale",
            "rotation_motion_bound",
            "rotation_fallback",
        ]
    return {"coarse_last_substep": {name: getattr(block, name).numpy().tolist() for name in names}}


def run(out: Path, *, steps: int, render_every: int = 0, config: dict | None = None):
    config = config or {}
    example_root = Path(__file__).resolve().parent
    repo_root = example_root.parents[1]
    code_files = [
        *(
            example_root / name
            for name in (
                "clawhauser_pick_place.py",
                "gripper.py",
                "cord_grasp.py",
                "cord_policy.py",
                "quality.py",
                "trajectory.py",
                "contact_recording.py",
                "contact_determinism.py",
                "parallel_gripper.py",
            )
        ),
        repo_root / "robosimstudio_materials/heterogeneous.py",
        repo_root / "robosimstudio_materials/vbd_coarse.py",
        repo_root / "robosimstudio/scene/physics/couplers.py",
        repo_root / "robosimstudio/scene/physics/compose.py",
        repo_root / "robosimstudio/specs/arrangement.py",
        repo_root / "robosimstudio/scene/entity.py",
    ]
    code_hashes = {
        str(path.relative_to(repo_root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in code_files
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "run_identity.json").write_text(
        json.dumps({"code_sha256": code_hashes, "requested_config": config}, indent=2) + "\n"
    )
    sim, obj, grip, camera, ring_ids, labels = build_scene(config)
    from contact_recording import prepare_contact_recording
    from quality import ContactQuality, MeshQuality, contact_depths, finger_table_depth, stretches

    quality = MeshQuality(sim, obj.entity)
    recording = prepare_contact_recording(sim, obj.entity.spec.name, grip.entity)
    contact_quality = ContactQuality(sim, recording, grip.entity, TABLE_Z)
    solver, observables = recording.solver, recording.observables
    sim.reset()
    quality.clear()
    contact_quality.clear()
    (out / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    started = time.perf_counter()
    entity = obj.entity
    particle_ids = entity.particles
    patch_ids = particle_ids[ring_ids]
    ring_ids = particle_ids[np.unique(entity.source_tets[labels == 2])]
    body_ids = particle_ids[np.unique(entity.source_tets[labels == 1])]
    body_only_ids = np.setdiff1d(body_ids, particle_ids[np.unique(entity.source_tets[labels != 1])])
    q0 = sim.pipeline.state_0.particle_q.numpy()[particle_ids]
    rest_det = np.linalg.det(q0[entity.source_tets[:, 1:]] - q0[entity.source_tets[:, :1]])
    if np.any(rest_det <= 0):
        raise RuntimeError("source tetrahedra were not positively oriented after build")
    initial_body = q0[np.unique(entity.source_tets[labels == 1])].mean(axis=0)
    positions, velocities, body_poses, body_velocities, diagnostics = [], [], [], [], []
    phases = {}
    pick_end_frame = None
    close_frame = None
    cord_policy = None
    placement_policy = None
    if config.get("placement_feedback", False) and not (
        config.get("trajectory") == "cord_feedback" and config.get("mechanical_gripper", False)
    ):
        raise ValueError("placement_feedback requires the mechanical cord_feedback path")

    videos = None
    video_error = None
    if render_every:
        from robosimstudio.core.render.video import VideoSet

        videos = VideoSet(str(out), fps=max(1, CONTROL_HZ // render_every))
        videos.__enter__()
    try:
        for frame in range(steps):
            if config.get("trajectory") == "observed":
                from trajectory import build_pick_trajectory, build_place_trajectory

                phase_changed = False
                if frame == CONTROL_HZ:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    poses = sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies]
                    path, metadata = build_pick_trajectory(
                        poses,
                        q[patch_ids],
                        dt=grip.entity.dt,
                        object_points=q[particle_ids],
                        ring_points=q[ring_ids],
                        closed_half_gap=config.get("observed_closed_half_gap"),
                        elapsed_before_s=frame / CONTROL_HZ,
                    )
                    grip.entity.set_trajectory(path)
                    phases["pick"] = metadata
                    config["transit_window_s"] = metadata["transport_window_s"]
                    config["target_translation_y_m"] = metadata["target_translation_xy_m"][1]
                    pick_end_frame = frame + round(metadata["duration_s"] * CONTROL_HZ)
                    phase_changed = True
                elif pick_end_frame is not None and frame == pick_end_frame:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    # Parallel waypoints describe ideal commands; compliant
                    # measured fingers need not lie exactly on that manifold.
                    poses = (
                        grip.entity.target_poses.numpy()[0]
                        if config.get("mechanical_gripper", False)
                        else sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies]
                    )
                    try:
                        path, metadata = build_place_trajectory(
                            poses, q[particle_ids], dt=grip.entity.dt, elapsed_before_s=frame / CONTROL_HZ
                        )
                    except ValueError as exc:
                        phases["placement_skipped"] = {"reason": str(exc), "time_s": frame / CONTROL_HZ}
                    else:
                        grip.entity.set_trajectory(path)
                        phases["place"] = metadata
                        config["duration_s"] = metadata["elapsed_end_s"]
                    phase_changed = True
                if phase_changed:
                    (out / "phases.json").write_text(json.dumps(phases, indent=2) + "\n")
            elif config.get("trajectory") == "cord_feedback":
                from cord_grasp import build_cord_place_trajectory
                from cord_policy import CordPickupPolicy, CordPlacementPolicy

                if frame == CONTROL_HZ:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    cord_policy = CordPickupPolicy(
                        grip.entity.target_poses.numpy()[0],
                        q[particle_ids],
                        entity.source_vertices * entity.spec.scale,
                        entity.source_tets,
                        labels,
                        dt=grip.entity.dt,
                        start_frame=frame,
                        post_grasp_half_gap=config.get("post_grasp_half_gap_m"),
                        parallel=bool(config.get("mechanical_gripper", False)),
                        freeze_on_close=bool(config.get("freeze_on_close", False)),
                        freeze_on_contact=bool(config.get("freeze_on_contact", False)),
                        normal_approach=bool(config.get("normal_approach", False)),
                    )
                    patch_ids = particle_ids[cord_policy.apex_ids]
                    phases["pick"] = cord_policy.metadata
                    config["transit_window_s"] = cord_policy.metadata["transport_window_s"]
                    config["target_translation_y_m"] = cord_policy.metadata["target_translation_xy_m"][1]
                    pick_end_frame = cord_policy.pick_end_frame
                if cord_policy is not None and frame < pick_end_frame:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    cord_policy.step(
                        frame, q[particle_ids], grip.entity, ring_normal_loads=diagnostics[-1]["ring_normal_load_N"]
                    )
                elif pick_end_frame is not None and frame == pick_end_frame:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    # Parallel waypoints describe ideal commands; compliant
                    # measured fingers need not lie exactly on that manifold.
                    poses = (
                        grip.entity.target_poses.numpy()[0]
                        if config.get("mechanical_gripper", False)
                        else sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies]
                    )
                    if config.get("placement_feedback", False):
                        gravity = sim.model.gravity.numpy()[0]
                        placement_policy = CordPlacementPolicy(
                            poses,
                            dt=grip.entity.dt,
                            start_frame=frame,
                            desired_body_xy=np.add(initial_body[:2], [TARGET_X - SOURCE_X, -0.10]),
                            initial_body_z=initial_body[2],
                            weight_n=float(entity.source_mass.sum() * np.linalg.norm(gravity)),
                        )
                        phases["place"] = placement_policy.metadata
                    else:
                        try:
                            path, metadata = build_cord_place_trajectory(
                                poses,
                                q[particle_ids],
                                dt=grip.entity.dt,
                                elapsed_before_s=frame / CONTROL_HZ,
                                desired_body_xy=np.add(initial_body[:2], [TARGET_X - SOURCE_X, -0.10]),
                                observed_body_points=q[body_ids],
                                parallel=bool(config.get("mechanical_gripper", False)),
                            )
                        except ValueError as exc:
                            phases["placement_skipped"] = {"reason": str(exc), "time_s": frame / CONTROL_HZ}
                        else:
                            grip.entity.set_trajectory(path)
                            phases["place"] = metadata
                            config["duration_s"] = metadata["elapsed_end_s"]
                if placement_policy is not None:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    placement_policy.step(
                        frame,
                        q[body_ids],
                        sim.pipeline.state_0.particle_qd.numpy()[body_ids],
                        q[particle_ids[cord_policy.apex_ids]],
                        sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies],
                        diagnostics[-1],
                        grip.entity,
                    )
                    if placement_policy.end_frame is not None:
                        config["duration_s"] = placement_policy.end_frame / CONTROL_HZ
            elif config.get("trajectory") == "cord":
                from cord_grasp import (
                    build_cord_close_transfer,
                    build_cord_pick_trajectory,
                    build_cord_place_trajectory,
                )

                phase_changed = False
                if frame == CONTROL_HZ or frame == close_frame:
                    q = sim.pipeline.state_0.particle_q.numpy()[particle_ids]
                    poses = sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies]
                    if frame == CONTROL_HZ:
                        path, metadata, apex = build_cord_pick_trajectory(
                            poses,
                            q,
                            entity.source_vertices * entity.spec.scale,
                            entity.source_tets,
                            labels,
                            dt=grip.entity.dt,
                            elapsed_before_s=frame / CONTROL_HZ,
                            approach_only=True,
                        )
                        close_frame = frame + round(metadata["duration_s"] * CONTROL_HZ)
                        phases["approach"] = metadata
                    else:
                        path, metadata, apex = build_cord_close_transfer(
                            poses,
                            q,
                            entity.source_vertices * entity.spec.scale,
                            entity.source_tets,
                            labels,
                            dt=grip.entity.dt,
                            elapsed_before_s=frame / CONTROL_HZ,
                        )
                        phases["pick"] = metadata
                        config["transit_window_s"] = metadata["transport_window_s"]
                        config["target_translation_y_m"] = metadata["target_translation_xy_m"][1]
                        pick_end_frame = frame + round(metadata["duration_s"] * CONTROL_HZ)
                    patch_ids = particle_ids[apex]
                    grip.entity.set_trajectory(path)
                    phase_changed = True
                elif pick_end_frame is not None and frame == pick_end_frame:
                    q = sim.pipeline.state_0.particle_q.numpy()
                    # Parallel waypoints describe ideal commands; compliant
                    # measured fingers need not lie exactly on that manifold.
                    poses = (
                        grip.entity.target_poses.numpy()[0]
                        if config.get("mechanical_gripper", False)
                        else sim.pipeline.state_0.body_q.numpy()[grip.entity.bodies]
                    )
                    try:
                        path, metadata = build_cord_place_trajectory(
                            poses,
                            q[particle_ids],
                            dt=grip.entity.dt,
                            elapsed_before_s=frame / CONTROL_HZ,
                            desired_body_xy=np.add(initial_body[:2], [TARGET_X - SOURCE_X, -0.10]),
                            observed_body_points=q[body_ids],
                        )
                    except ValueError as exc:
                        phases["placement_skipped"] = {"reason": str(exc), "time_s": frame / CONTROL_HZ}
                    else:
                        grip.entity.set_trajectory(path)
                        phases["place"] = metadata
                        config["duration_s"] = metadata["elapsed_end_s"]
                    phase_changed = True
                if phase_changed:
                    (out / "phases.json").write_text(json.dumps(phases, indent=2) + "\n")
            sim.run(1).sync()
            q = sim.pipeline.state_0.particle_q.numpy()
            body_q = sim.pipeline.state_0.body_q.numpy()
            owned = q[particle_ids]
            owned_bodies = getattr(grip.entity, "all_bodies", grip.entity.bodies)
            if not (
                np.isfinite(owned).all()
                and np.isfinite(sim.pipeline.state_0.particle_qd.numpy()[particle_ids]).all()
                and np.isfinite(body_q[owned_bodies]).all()
                and np.isfinite(sim.pipeline.state_0.body_qd.numpy()[owned_bodies]).all()
            ):
                raise RuntimeError(f"non-finite object or gripper state at frame {frame + 1}")
            det = np.linalg.det(owned[entity.source_tets[:, 1:]] - owned[entity.source_tets[:, :1]])
            contacts = recording.contacts
            count = int(contacts.soft_contact_count.numpy()[0])
            contact_shapes = contacts.soft_contact_shape.numpy()[: min(count, contacts.soft_contact_max)]
            finger_mask = np.isin(contact_shapes, grip.entity.finger_shapes)
            finger_contacts = int(np.count_nonzero(finger_mask))
            contact_indices = contacts.soft_contact_indices.numpy()[: len(contact_shapes)]
            contact_bary = contacts.soft_contact_barycentric.numpy()[: len(contact_shapes)]
            ring_weight = np.sum(contact_bary * np.isin(contact_indices, ring_ids), axis=1)
            body_weight = np.sum(contact_bary * np.isin(contact_indices, body_only_ids), axis=1)
            ring_mask = ring_weight > 0
            patch_mask = np.any(np.isin(contact_indices, patch_ids), axis=1)
            contact_state = recording.state()
            depths = contact_depths(
                contact_state.particle_q.numpy(), contact_state.body_q.numpy(), recording.model, contacts
            )
            forces = observables.contact_f[
                contacts.rigid_contact_max : contacts.rigid_contact_max + len(contact_shapes)
            ].numpy()[:, :3]
            normals = contacts.soft_contact_normal.numpy()[: len(contact_shapes)]
            side_masks = [np.isin(contact_shapes, shapes) for shapes in grip.entity.shapes_by_side]
            carrier_mask = contact_shapes == getattr(grip.entity, "carrier_shape", -1)
            static_mask = recording.model.shape_body.numpy()[contact_shapes] < 0
            static_weights = np.where(static_mask, np.maximum(-forces[:, 2], 0), 0)
            contact_points = (
                contact_bary[:, :, None] * contact_state.particle_q.numpy()[np.maximum(contact_indices, 0)]
            ).sum(axis=1)
            static_center = (
                np.average(contact_points, axis=0, weights=static_weights).tolist()
                if static_weights.sum() > 0
                else None
            )
            if config.get("dump_contacts") and frame in (20, 35, 60):
                dump = {
                    "indices": contact_indices.tolist(),
                    "shapes": contact_shapes.tolist(),
                    "normals": normals.tolist(),
                    "force_on_shape_N": forces.tolist(),
                    "points_on_shape_local": contacts.soft_contact_body_pos.numpy()[: len(contact_shapes)].tolist(),
                    "barycentric": contacts.soft_contact_barycentric.numpy()[: len(contact_shapes)].tolist(),
                    "body_q": body_q.tolist(),
                    "particle_q": q.tolist(),
                    "shape_body": sim.model.shape_body.numpy().tolist(),
                }
                (out / f"contacts_{frame:04d}.json").write_text(json.dumps(dump))
            row = {
                "frame": frame + 1,
                "time_s": (frame + 1) / CONTROL_HZ,
                "ring_center_m": q[ring_ids].mean(axis=0).tolist(),
                "body_center_m": q[body_ids].mean(axis=0).tolist(),
                "body_minimum_z_m": float(q[body_ids, 2].min()),
                "body_centroid_velocity_m_s": sim.pipeline.state_0.particle_qd.numpy()[body_ids].mean(axis=0).tolist(),
                "body_translation_x_m": float(q[body_ids, 0].mean() - initial_body[0]),
                "minimum_z_m": float(owned[:, 2].min()),
                "minimum_tet_volume_ratio": float((det / rest_det).min()),
                "inverted_tetrahedra": int(np.count_nonzero(det <= 0)),
                "soft_contacts": count,
                "contact_capacity": int(contacts.soft_contact_max),
                "maximum_contact_plane_penetration_m": float(depths.max(initial=0)),
                "finger_table_penetration_m": finger_table_depth(body_q[grip.entity.bodies], grip.entity.spec, TABLE_Z),
                "principal_stretch_by_part": stretches(owned, entity),
                **(
                    {
                        "carriage_servo_wrench": grip.entity.applied_carriage_wrench.numpy()[0, 0].tolist(),
                        "jaw_motor_effort_N": grip.entity.motor_efforts.numpy()[0].tolist(),
                        "jaw_motor_effort_available": bool(grip.entity.motor_efforts_available.numpy()[0]),
                    }
                    if config.get("mechanical_gripper", False)
                    else {"finger_servo_wrench": grip.entity.applied_wrenches.numpy()[0].tolist()}
                ),
                "finger_contacts": finger_contacts,
                "all_contact_force_on_object_N": (-forces.sum(axis=0)).tolist(),
                "object_mass_weighted_center_m": np.average(owned, axis=0, weights=entity.source_mass).tolist(),
                "object_momentum_kg_m_s": (
                    sim.pipeline.state_0.particle_qd.numpy()[particle_ids] * entity.source_mass[:, None]
                )
                .sum(axis=0)
                .tolist(),
                "finger_force_resultant_N": [forces[m].sum(axis=0).tolist() for m in side_masks],
                "finger_normal_load_N": [
                    float(np.abs(np.sum(forces[m] * normals[m], axis=1)).sum()) for m in side_masks
                ],
                "ring_normal_load_N": [
                    float((np.abs(np.sum(forces[m] * normals[m], axis=1)) * ring_weight[m]).sum()) for m in side_masks
                ],
                "static_support_force_on_object_N": (-forces[static_mask].sum(axis=0)).tolist(),
                "static_contact_center_m": static_center,
                "body_support_force_on_object_N": (
                    -(forces[static_mask] * body_weight[static_mask, None]).sum(axis=0)
                ).tolist(),
                "carrier_normal_load_N": float(
                    np.abs(np.sum(forces[carrier_mask] * normals[carrier_mask], axis=1)).sum()
                ),
                **recording.proxy_alignment(grip.entity, body_q),
                **_mechanism_errors(grip.entity, body_q),
                **_coarse_diagnostics(solver),
                "finger_contacts_by_side": [
                    int(np.count_nonzero(np.isin(contact_shapes, shapes))) for shapes in grip.entity.shapes_by_side
                ],
                "ring_contacts_by_side": [
                    int(np.count_nonzero(np.isin(contact_shapes, shapes) & ring_mask))
                    for shapes in grip.entity.shapes_by_side
                ],
                "grasp_patch_contacts": int(np.count_nonzero(finger_mask & patch_mask)),
                "gripper_body_x_m": body_q[grip.entity.bodies, 0].tolist(),
                "gripper_body_y_m": body_q[grip.entity.bodies, 1].tolist(),
                "gripper_body_z_m": body_q[grip.entity.bodies, 2].tolist(),
            }
            diagnostics.append(row)
            if placement_policy is not None:
                placement_policy.finish_interval(frame + 1)
            positions.append(owned.astype(np.float32))
            velocities.append(sim.pipeline.state_0.particle_qd.numpy()[particle_ids].copy())
            body_poses.append(body_q.astype(np.float32))
            body_velocities.append(sim.pipeline.state_0.body_qd.numpy().copy())
            if videos is not None and frame % render_every == 0:
                try:
                    videos.append_frames({camera.name: sim.render(camera)})
                except Exception as exc:
                    video_error = repr(exc)
                    try:
                        videos.__exit__(type(exc), exc, exc.__traceback__)
                    except Exception:
                        pass
                    videos = None
            if frame % 15 == 0 or frame == steps - 1:
                print(json.dumps(row), flush=True)
            if placement_policy is not None and placement_policy.status == "complete":
                break
    finally:
        if videos is not None:
            try:
                videos.__exit__(None, None, None)
            except Exception as exc:
                video_error = repr(exc)

        # Preserve valid measurements even if a later planning/physics/render
        # stage fails. The raised exception still marks that run unsuccessful.
        np.savez_compressed(
            out / "states.npz",
            particle_q=np.asarray(positions),
            particle_qd=np.asarray(velocities),
            body_q=np.asarray(body_poses),
            body_qd=np.asarray(body_velocities),
        )
        (out / "diagnostics.json").write_text(json.dumps(diagnostics, indent=2) + "\n")
        (out / "config.json").write_text(json.dumps(config, indent=2) + "\n")
        (out / "quality_substeps.json").write_text(json.dumps(quality.read(), indent=2) + "\n")
        (out / "contact_quality_substeps.json").write_text(json.dumps(contact_quality.read(), indent=2) + "\n")
        if phases:
            (out / "phases.json").write_text(json.dumps(phases, indent=2) + "\n")
    newton_root = Path(newton.__file__).resolve().parents[1]
    newton_commit = subprocess.check_output(["git", "-C", str(newton_root), "rev-parse", "HEAD"], text=True).strip()
    manifest = {
        "asset": "clawhauser",
        "code_sha256": code_hashes,
        "code_changed_during_run": any(
            hashlib.sha256(path.read_bytes()).hexdigest() != code_hashes[str(path.relative_to(repo_root))]
            for path in code_files
        ),
        "newton_version": newton.__version__,
        "newton_source_commit": newton_commit,
        "solver": type(solver).__name__,
        "coupled": bool(sim.pipeline.program.coupled),
        "finger_body_ids": [int(i) for i in grip.entity.bodies],
        "finger_shape_ids": [int(i) for i in grip.entity.finger_shapes],
        "tetrahedra": int(sim.model.tet_count),
        "part_tet_counts": {str(i): int(np.count_nonzero(labels == i)) for i in np.unique(labels)},
        "source_young_pa": sorted(float(v) for v in np.unique(entity.source_elastic[:, 0])),
        "source_poisson": sorted(float(v) for v in np.unique(entity.source_elastic[:, 1])),
        "source_density_kg_m3": sorted(float(v) for v in np.unique(entity.source_density)),
        "global_soft_contact_mu": float(sim.model.soft_contact_mu),
        "grasp_attachments": int(sim.model.attachment_particle_body_count),
        "completed_steps": len(diagnostics),
        "effective_elastic": {str(i): entity.effective_elastic[labels == i][0].tolist() for i in np.unique(labels)},
        "effective_damping_pa_s": {str(i): float(entity.effective_damping[labels == i][0]) for i in np.unique(labels)},
        "graph_captured": sim.pipeline.captured,
        "face_contact_compile_options": (
            {"mode": "release", "optimization_level": 1, "enable_backward": False}
            if config.get("full_surface_contact", False)
            else None
        ),
        "recording_wall_s": time.perf_counter() - started,
        "quality_substeps": quality.read(),
        "contact_quality_substeps": contact_quality.read(),
        **_effort_summary(grip.entity),
        "video_error": video_error,
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if diagnostics:
        final = diagnostics[-1]
        transit_start, transit_end = config.get("transit_window_s", [3.5, 5.5])
        transit = [row for row in diagnostics if transit_start <= row["time_s"] <= transit_end]
        duration = float(config.get("duration_s", DURATION))
        settled = [row for row in diagnostics if row["time_s"] >= duration - 0.9]
        lifted = float(max((row["body_center_m"][2] - initial_body[2] for row in transit), default=0.0))
        clearance = float(min((row["minimum_z_m"] - TABLE_Z for row in transit), default=0.0))
        peak_finger_contacts = max(row["finger_contacts"] for row in diagnostics)
        target_error_x = float(abs(final["body_center_m"][0] - (initial_body[0] + TARGET_X - SOURCE_X)))
        target_xy = np.add(initial_body[:2], [TARGET_X - SOURCE_X, float(config.get("target_translation_y_m", 0))])
        target_error_xy = float(np.linalg.norm(np.asarray(final["body_center_m"][:2]) - target_xy))
        max_penetration = max(row["maximum_contact_plane_penetration_m"] for row in diagnostics)
        max_finger_penetration = max(row["finger_table_penetration_m"] for row in diagnostics)
        contact_overflow = any(row["soft_contacts"] > row["contact_capacity"] for row in diagnostics)
        proxy_corner_error = max(max(row["proxy_finger_corner_error_m"]) for row in diagnostics)
        carrier_transport_load = max((row["carrier_normal_load_N"] for row in transit), default=0.0)
        body_drift = (
            float(np.linalg.norm(np.asarray(final["body_center_m"]) - settled[0]["body_center_m"])) if settled else None
        )
        finger_distance = min(
            float(
                np.linalg.norm(
                    np.asarray(final["ring_center_m"])
                    - np.asarray(
                        [final["gripper_body_x_m"][i], final["gripper_body_y_m"][i], final["gripper_body_z_m"][i]]
                    )
                )
            )
            for i in range(2)
        )
        bilateral_ring_grasp = bool(
            transit
            and all(min(row["ring_contacts_by_side"]) > 0 and min(row["ring_normal_load_N"]) > 0.02 for row in transit)
        )
        released = bool(settled and max(row["finger_contacts"] for row in settled) == 0)
        table_supported = bool(
            TABLE_Z - 0.002 <= final["minimum_z_m"] <= TABLE_Z + 0.004
            and final["static_support_force_on_object_N"][2] > 0.02
            and settled
            and max(row["carrier_normal_load_N"] for row in settled) <= 0.02
        )
        max_inverted = max(row["inverted_tetrahedra"] for row in diagnostics)
        settled_inverted = max((row["inverted_tetrahedra"] for row in settled), default=None)
        max_inverted_fraction = max_inverted / len(labels)
        actuator_limits_valid = True
        mechanism_constraints_valid = True
        mechanism_errors = {}
        if config.get("mechanical_gripper", False):
            mechanism = grip.entity
            actuator_limits_valid = bool(
                all(row["jaw_motor_effort_available"] for row in diagnostics)
                and np.all(mechanism.peak_motor_effort.numpy() <= mechanism.spec.jaw_force_limit + 1.0e-5)
                and np.all(mechanism.peak_carriage_force.numpy() <= mechanism.spec.carriage_force_limit + 1.0e-5)
                and np.all(mechanism.peak_carriage_torque.numpy() <= mechanism.spec.carriage_torque_limit + 1.0e-5)
            )
            mechanism_errors = {
                key: max(max(row[key]) for row in diagnostics)
                for key in (
                    "mechanical_joint_lateral_error_m",
                    "mechanical_joint_orientation_error_deg",
                    "mechanical_joint_limit_error_m",
                )
            }
            mechanism_constraints_valid = bool(
                mechanism_errors["mechanical_joint_lateral_error_m"] <= 0.0005
                and mechanism_errors["mechanical_joint_orientation_error_deg"] <= 1.0
                and mechanism_errors["mechanical_joint_limit_error_m"] <= 0.0001
            )
        success = (
            steps >= round(duration * CONTROL_HZ)
            and (
                not config.get("placement_feedback", False)
                or (placement_policy is not None and placement_policy.status == "complete")
            )
            and lifted > 0.08
            and clearance > 0.05
            and bilateral_ring_grasp
            and target_error_xy <= 0.02
            and released
            and table_supported
            and body_drift is not None
            and body_drift <= 0.005
            and finger_distance > 0.08
            and settled_inverted == 0
            and max_inverted == 0
            and np.min(quality.minimum.numpy()) > 0
            and not np.any(quality.nonfinite.numpy())
            and max_penetration <= 0.0005
            and max_finger_penetration <= 0.0005
            and contact_quality.read()["maximum_recorded_plane_penetration_m"] <= 0.0005
            and contact_quality.read()["maximum_gripper_box_table_penetration_m"] <= 0.0005
            and not contact_quality.read()["invalid_or_overflow"]
            and not contact_overflow
            and proxy_corner_error <= 0.0005
            and carrier_transport_load <= 0.02
            and actuator_limits_valid
            and mechanism_constraints_valid
        )
        result = {
            "transit_body_lift_m": lifted,
            "transit_clearance_m": clearance,
            "body_translation_x_m": final["body_translation_x_m"],
            "target_error_x_m": target_error_x,
            "target_error_xy_m": target_error_xy,
            "maximum_contact_plane_penetration_m": max_penetration,
            "maximum_finger_table_penetration_m": max_finger_penetration,
            "contact_overflow": contact_overflow,
            "maximum_proxy_finger_corner_error_m": proxy_corner_error,
            "maximum_carrier_transport_load_N": carrier_transport_load,
            "actuator_limits_valid": actuator_limits_valid,
            "mechanism_constraints_valid": mechanism_constraints_valid,
            "maximum_mechanism_errors": mechanism_errors,
            "peak_finger_contacts": peak_finger_contacts,
            "bilateral_ring_grasp": bilateral_ring_grasp,
            "placement_feedback_status": None if placement_policy is None else placement_policy.status,
            "released": released,
            "table_supported": table_supported,
            "post_release_body_drift_m": body_drift,
            "final_finger_ring_distance_m": finger_distance,
            "max_inverted_tetrahedra": max_inverted,
            "max_inverted_fraction": max_inverted_fraction,
            "settled_inverted_tetrahedra": settled_inverted,
            "quality_substeps": quality.read(),
            "contact_quality_substeps": contact_quality.read(),
            **_effort_summary(grip.entity),
            "success": bool(success),
        }
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    return diagnostics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUTPUT / "run")
    parser.add_argument("--steps", type=int, default=round(DURATION * CONTROL_HZ))
    parser.add_argument("--render-every", type=int, default=0)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    run(
        args.out,
        steps=args.steps,
        render_every=args.render_every,
        config=json.loads(args.config.read_text()) if args.config else None,
    )


if __name__ == "__main__":
    main()
