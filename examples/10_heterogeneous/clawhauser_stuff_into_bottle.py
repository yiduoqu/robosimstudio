# SPDX-License-Identifier: Apache-2.0
"""Lower the labelled Clawhauser volume into a faceted glass cylinder, tail first.

The cylinder's bore is deliberately too small for the toy: at the default 60 mm
inner diameter the tail (47 by 36 mm) enters and the body (110 by 72 mm) does
not, so the descent ends on the rim with the tail inside. That is the point of
the scene, and it is measured rather than asserted -- how deep the toy got, how
much of it is inside, whether the rim holds it after the jaws open, and what the
jaws spent to get it there.

The claw is the force-bounded one, and this scene is what that bound is for. In
dynamic mode the fingers carry their own mass and are driven by a clamped PD
wrench, so the push the toy receives is at most the clamp and ``peak_force_n`` is
a measurement. The scene reports the effort against the achieved depth: a jaw
pressing at its cap while the toy has stopped moving is a jam, and the descent
stops there rather than grinding -- a guarded move again, this time on the
object's own depth rate instead of on a contact force, because the force signal
is exactly the one that stays at the cap when something is stuck.

Geometry is measured from the mesh and reported: the loop's hole centre is where
the pads close, its outer reach in the closing direction is the width the squeeze
is stated from, and the container is built from its bore radius, panel count and
depth, so a sweep over bores is an argument rather than a different file.

Controls, for contrast rather than for the result:

    --claw kinematic   prescribed poses: unlimited jaw authority
    --no-coarse        stock VBD: no common-translation solve under load

Knobs, each reported with its measured effect:

    --bore   inner diameter of the container [m]
    --mu     the pipeline's single global soft-contact friction coefficient
    --cap    the claw's force clamp [N]

Run on a CUDA node with Newton 1.8 development and RoboSimStudio on PYTHONPATH.
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
from gripper import ScriptedGripperSpec
from loop_geometry import ring_geometry, smooth
from mesh_quality import MeshQuality, penetration

import robosimstudio as rss
from robosimstudio_materials.heterogeneous import HeterogeneousSoftSpec, load_labelled_tets

ASSET = Path(__file__).resolve().parent / "assets" / "clawhauser"
OUTPUT = Path(__file__).resolve().parents[2] / "outputs" / "clawhauser_stuff_into_bottle"
CONTROL_HZ = 30
SUBSTEPS = 20
DURATION = 10.5
"""Long enough for the guarded descent to stop early or late and still leave the
last half second still, so the window the outcome is judged on is a quiet one.
"""
BORE_RADIUS = 0.030
"""Inner radius of the container: 60 mm across by default. The tail is 47 by 36 mm
and the body 110 by 72 mm, so this bore takes the tail and stops on the body.
"""
WALL = 0.004
"""Panel thickness. The container is 24 flat panels, not a cylinder: a faceted
wall is what a slab scene can build, and the facet's chord is what the toy rides
over on its way in.
"""
PANELS = 24
RIM_Z = 0.090
"""Height of the rim above the work surface: the depth the toy can reach."""
FLOOR_Z = 0.004
"""Height of the container's floor above the work surface."""
LIFT_CLEARANCE = 0.020
"""How far the toy's lowest point clears the rim while it is carried across."""
APPROACH = 0.004
"""How far it clears the rim when the descent starts."""
DESCEND_RATE = 0.035
"""Planned descent speed [m/s]. Slower than a drop and faster than a creep: the
tail has to find the bore, and the jam has to be a jam rather than an impact.
"""
JAM_STEP = 0.0005
JAM_FRAMES = 8
"""A stall: 0.5 mm of depth is the least progress that counts, and eight frames
without it (0.27 s) is long enough that the swing of the carried toy, which moves
its lowest point by tens of millimetres, cannot look like one.
"""
DROP_CLEARANCE = 0.006
"""Above this the toy is not resting on the bench."""
TABLE_Z = 0.75
BENCH_X = 0.6
"""Where the work-surface frame's origin sits in world x."""
SOURCE_X = -0.07
TARGET_X = 0.07
CONTACT_GAP = 0.02
"""The collision pipeline's detection gap. The coarse block's own gap must be
this: a correction can only reach contacts the pipeline stored."""
CONTACT_KE = 100000.0
"""The arrangement's rigid-soft penalty stiffness: support load is this times
the summed penetration a contact patch reports."""
COARSE_BUDGET = 0.005
"""Cumulative coarse translation per substep, at most a quarter of the gap."""
GRIP_SQUEEZE = 0.006
"""How far past the loop's measured width the closed pads aim [m]. The same
measured value the rod scene uses: at 1 mm the round cord slides straight out of
the pinch, at 6 mm the drive sits at its clamp and the grasp holds.
"""
PAD_HALF_THICKNESS = 0.003
"""The pad box's half-extent along the closing axis, as ``gripper.py`` builds it."""
OPENED_HALF_GAP = 0.045
"""How far apart the jaws open once the toy is in place."""
JAM_HOLD = 1.0
JAM_OPEN = 0.6
JAM_WITHDRAW = 0.5
"""After the guarded stop: hold the toy still, open, and rise clear."""
WITHDRAW_Z = 0.060
"""How far the opened jaws rise to clear the toy: straight up, and clear of the
rim, so the withdrawal cannot drag the toy out of the container it just entered.
"""
PHASES = {
    "close": (0.6, 1.4),
    "lift": (1.4, 3.0),
    "hang": (3.0, 4.0),
    "carry": (4.0, 6.0),
    "lower": (6.0, 6.5),
    "descend": (6.5, 9.0),
    "open": (9.5, 10.0),
    "withdraw": (10.0, 10.5),
}
"""Planned phase windows. ``hang`` is a phase with no motion: the toy is lifted
out of its standing pose and has to settle into hanging from the pinch before it
is carried anywhere. The descend's own end is not a boundary, only the ramp's
bound; the stop is the jam guard, and the open and withdraw windows are the
fallback's, for a run whose guard never fires.
"""
SOURCE_HASHES = {
    "final_mesh.mesh": "df3d1505d96d36fbfa76291e7ab94269a30500ba5e4b87516d0bf8f57f0c6458",
    "heterogeneous_params.npz": "7cfc8ca3cc7c08d261aa90b08832230801fec1ac62848d0e9239d9e2f318e8c6",
    "inferred_params.json": "45aacd698f2b38a6fd8bf13b46cc9c33497fc90c6c1bdf088ed12d915ec3d992",
    "appearance/visual_to_physics.npz": "4b015d1141794211e4e2c6e2b39fadef6762a73ffe2d76b921e47d5676f3d6c4",
}


def build_scene(*, coarse: bool, claw: str, bore: float, cap: float, mu: float | None):
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
    ring = ring_geometry(vertices, tets, labels)

    # Standing pose, tail down: the same quarter turn about x the rod scene uses,
    # which puts the mesh's +y (the loop) up and its +z (the hole's axis) along
    # world y. Under that rotation a vertex's world z is its local y, so the
    # toy's own lowest local y is what stands it on the bench.
    quat = (float(np.sin(np.pi / 4.0)), 0.0, 0.0, float(np.cos(np.pi / 4.0)))
    hole = ring["hole_centre_local"]
    base = ring["base_local_y"]
    pose = (SOURCE_X, 0.0, -base + 0.001)
    hole_height = float(hole[1] - base)
    # The tail is what goes in first, so it is the tail's own centre that gets
    # carried over the container's axis. Centring the hole instead leaves the
    # tail 3.5 mm off-axis, and the tail's 47 by 36 mm section has 0.8 mm of
    # diametral clearance in a 60 mm bore: it would be riding on the facet edge.
    tail_tets = np.where(labels == 0)[0]
    tail_vertices = vertices[np.unique(tets[tail_tets])]
    tail_centre = vertices[tets[tail_tets]].mean(axis=1).mean(axis=0)
    geometry = {
        "hole_centre_local": np.round(hole, 4).tolist(),
        "hole_inradius_m": ring["hole_inradius"],
        "hole_inradius_median_m": ring["hole_inradius_median"],
        "loop_reach_m": ring["side_reach"],
        "pad_half_thickness_m": PAD_HALF_THICKNESS,
        "grip_squeeze_m": GRIP_SQUEEZE,
        "toy_height_m": float(vertices[:, 1].max() - base),
        "hole_height_m": hole_height,
        # Where the jaws close and how wide the thing they close on is: the loop's
        # own centre and its outer reach, as in Scene A. See the gripper's note.
        "grip_z_m": hole_height + 0.001,
        "grip_reach_m": ring["side_reach"],
        "pad_half_width_m": 0.006,
        # The ledges gripper.py builds under each pad (2.4 mm tall, 3 mm inboard
        # of the pad's face): the grasp rests on them, so the run says so.
        "ledge": True,
        "bore_radius_m": bore,
        "rim_z_m": RIM_Z,
        "floor_z_m": FLOOR_Z,
        "lift_m": RIM_Z + LIFT_CLEARANCE - 0.001,
        "carry_m": TARGET_X - SOURCE_X - float(tail_centre[0]),
        "tail_centre_local": np.round(tail_centre, 4).tolist(),
        "tail_extent_m": np.round(np.ptp(tail_vertices, axis=0), 4).tolist(),
        "lower_m": APPROACH - LIFT_CLEARANCE,
        "descend_m": FLOOR_Z - RIM_Z - APPROACH,
        "descend_rate_m_s": DESCEND_RATE,
        "withdraw_z_m": WITHDRAW_Z,
        "force_cap_n": cap,
    }
    print(json.dumps({"geometry": geometry}), flush=True)

    common = {
        "iterations": 12,
        "particle_enable_self_contact": False,
        "particle_enable_tile_solve": False,
        "rigid_body_particle_contact_buffer_size": 4096,
    }
    vbd = (
        rss.backend(
            "robosimstudio_materials.coarse_translation.SolverVBDCoarse",
            coarse_translation=True,
            coarse_contact_gap=CONTACT_GAP,
            coarse_budget=COARSE_BUDGET,
            **common,
        )
        if coarse
        else rss.backend("vbd", **common)
    )
    physics = rss.ArrangementSpec(
        control_hz=CONTROL_HZ,
        substeps=SUBSTEPS,
        use_cuda_graph=False,
        backends=(("soft", vbd), ("rigid", vbd), ("*", vbd)),
        collision=rss.CollisionSpec(
            soft_contact_margin=CONTACT_GAP,
            full_surface_contact=False,
            contact_ke=CONTACT_KE,
            contact_kd=25.0,
            broad_phase="nxn",
        ),
    )
    scene = rss.Scene(cell="bench_075", physics=physics, name="clawhauser_stuff_into_bottle")

    # The container: PANELS flat panels whose corners meet on the bore's outer
    # radius, so the inner surface is a faceted bore that never intrudes past the
    # radius it claims, plus a floor under all of them.
    glass = [
        scene.add(
            rss.slab(
                name="container_floor",
                half=(bore + WALL, bore + WALL, FLOOR_Z / 2.0),
                at=(TARGET_X, 0.0, FLOOR_Z / 2.0),
                color=(0.55, 0.68, 0.74),
            )
        )
    ]
    panel_radius = bore + WALL / 2.0
    half_height = (RIM_Z - FLOOR_Z) / 2.0
    for index in range(PANELS):
        angle = 2.0 * np.pi * index / PANELS
        # The panel's own x is the chord, so the yaw that lays it along the
        # tangent is the radial angle plus a quarter turn.
        yaw = angle + 0.5 * np.pi
        glass.append(
            scene.add(
                rss.slab(
                    name=f"container_panel_{index}",
                    half=(float(np.tan(np.pi / PANELS) * panel_radius), WALL / 2.0, half_height),
                    at=(
                        TARGET_X + panel_radius * float(np.cos(angle)),
                        panel_radius * float(np.sin(angle)),
                        FLOOR_Z + half_height,
                        0.0,
                        0.0,
                        float(np.sin(yaw / 2.0)),
                        float(np.cos(yaw / 2.0)),
                    ),
                    color=(0.62, 0.74, 0.80),
                )
            )
        )

    obj = scene.add(
        HeterogeneousSoftSpec(
            name="clawhauser",
            mesh_path=str(ASSET / "final_mesh.mesh"),
            params_path=str(ASSET / "heterogeneous_params.npz"),
            materials_path=str(ASSET / "inferred_params.json"),
            pose=pose,
            quat=quat,
            damping_time=0.0005,
            contact_ke=CONTACT_KE,
        )
    )
    grip = scene.add(
        ScriptedGripperSpec(
            # The loop's sides, with the ledges these pads carry -- the difference
            # from Scene A, and the two attempts that put it here.
            #
            # The oracle asks for the body. It cannot be gripped: at the body's
            # measured half-width the pads clamp 104 mm apart, wider than the bore
            # they would have to reach into. The tail fits the bore and is what the
            # jaws were first given, but it is the *wrong side of the weight*: the
            # tail is the toy's lowest part, so gripping it hangs the body above
            # the pinch and the toy rotates about it -- measured, tipped 60 degrees
            # and on the bench by t = 3.6 s, with the pads still closed and later
            # 50 mm below the rim.
            #
            # The loop puts the weight below the grip and fits the bore. Squeezed
            # without ledges it creeps off that round cord in about two seconds of
            # hanging (finger contacts 6, 4, 3, 1, 0 over 2.6-4.6 s of b4-dyn,
            # toy on the bench before the carry began), which Scene A never sees
            # because the rod takes the toy's weight within 2 s. The 2.4 mm ledge
            # under each pad closes that off the way it does for the baseline: the
            # toy rests on the ledges, so what holds it up is form closure and not
            # a 5 N squeeze on a round cord.
            name="bottle_loop_gripper",
            mode=claw,
            center=(SOURCE_X + float(hole[0]), 0.0, geometry["grip_z_m"]),
            half_gap=ring["side_reach"] + PAD_HALF_THICKNESS + 0.012,
            closed_half_gap=ring["side_reach"] + PAD_HALF_THICKNESS - GRIP_SQUEEZE,
            pad_half_thickness=PAD_HALF_THICKNESS,
            pad_half_width=0.006,
            ledge=True,
            force_limit=cap,
            duration=DURATION,
            control_hz=CONTROL_HZ,
            substeps=SUBSTEPS,
        )
    )
    camera = rss.CameraSpec(
        name="overview",
        width=960,
        height=540,
        eye=(0.72, -0.40, 1.06),
        look_at=(0.60, 0.0, 0.90),
        fov_y=45.0,
    )
    scene.add(camera)
    sim = scene.build(device="cuda:0", warmup=False, use_cuda_graph=False)
    entity = obj.entity
    if not type(sim.pipeline.program.solver).__name__.startswith("SolverVBD"):
        raise RuntimeError(f"expected a VBD solver, got {type(sim.pipeline.program.solver).__name__}")
    if sim.model.tet_count != len(tets) or len(entity.tet_part_labels) != len(tets):
        raise RuntimeError("RSS lost Clawhauser tetrahedra or part labels")
    if sim.model.attachment_particle_body_count:
        raise RuntimeError("the contact grasp scene must not contain particle attachments")
    if mu is not None:
        # Newton VBD exposes one global soft-contact friction coefficient; this is
        # it, and it is the knob the sweep turns.
        sim.model.soft_contact_mu = float(mu)
    model_tets = sim.model.tet_indices.numpy()[entity.tet_range[0] : entity.tet_range[1]]
    if not np.array_equal(model_tets - entity.particles[0], tets):
        raise RuntimeError("Newton changed the source tetrahedron order")
    young, nu = entity.source_elastic.T
    source_mu = young / (2.0 * (1.0 + nu))
    source_lambda = young * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    model_elastic = sim.model.tet_materials.numpy()[entity.tet_range[0] : entity.tet_range[1], :2]
    if not np.allclose(model_elastic, np.column_stack((source_mu, source_lambda)), rtol=1e-5):
        raise RuntimeError("Newton FEM elasticity differs from the source estimates")
    if not np.allclose(sim.model.particle_mass.numpy()[entity.particles], entity.source_mass, rtol=1e-5):
        raise RuntimeError("Newton nodal mass differs from source per-tet densities")
    glass_shapes = np.concatenate([np.asarray(part.shapes, dtype=np.int32) for part in glass])
    return sim, obj, grip, camera, ring, glass_shapes, geometry


def waypoints(grip, capacity: int, dt: float, geometry: dict) -> np.ndarray:
    """The claw's own path: settle, close, lift, hang, carry, lower, descend.

    The descent runs to the container's floor and is bounded there, not scheduled:
    a run whose guard never fires must not drive the jaws through the bench. What
    ends it in every working run is the jam guard, and ``after_jam`` is the path
    that follows.
    """
    centre = np.asarray(grip.entity.center, dtype=np.float64)
    opened = float(grip.spec.half_gap)
    closed = float(grip.spec.closed_half_gap)
    lift = geometry["lift_m"]
    carry = geometry["carry_m"]
    lower = geometry["lower_m"]
    descend = geometry["descend_m"]
    poses = np.zeros((capacity, 2, 7), dtype=np.float32)
    poses[:, :, 6] = 1.0
    for index in range(capacity):
        time = index * dt
        gap = opened + (closed - opened) * smooth(time, *PHASES["close"])
        gap += (OPENED_HALF_GAP - gap) * smooth(time, *PHASES["open"])
        away = smooth(time, *PHASES["withdraw"])
        place = centre + np.array(
            [
                carry * smooth(time, *PHASES["carry"]),
                0.0,
                lift * smooth(time, *PHASES["lift"])
                + lower * smooth(time, *PHASES["lower"])
                # max(0, ...) is what keeps this zero before the descent's window
                # opens: a ramp read at a negative time is a *rise*, and this one
                # held the jaws 210 mm over the toy's head for the whole run.
                + max(-DESCEND_RATE * max(0.0, time - PHASES["descend"][0]), descend)
                + WITHDRAW_Z * away,
            ]
        )
        poses[index, 0, :3] = place + np.array([-gap, 0.0, 0.0])
        poses[index, 1, :3] = place + np.array([gap, 0.0, 0.0])
    return poses


def after_jam(positions: np.ndarray, dt: float, closed_half_gap: float) -> np.ndarray:
    """The path the jam guard hands over to: hold, open, and rise clear.

    Built from where the fingers actually are when the object stops moving, so the
    hold is a hold and not a resumed descent -- but the gap it holds is the one the
    trajectory was aiming at, not the gap the fingers are at. Commanding the
    measured gap lets the squeeze go, and a released loop slides out of the pinch.
    """
    positions = np.asarray(positions, dtype=np.float64)
    count = int(np.ceil((JAM_HOLD + JAM_OPEN + JAM_WITHDRAW) / dt)) + 2
    centre = 0.5 * (positions[0] + positions[1])
    poses = np.zeros((count, 2, 7), dtype=np.float32)
    poses[:, :, 3:] = (0.0, 0.0, 0.0, 1.0)
    for index in range(count):
        time = index * dt
        gap = closed_half_gap + (OPENED_HALF_GAP - closed_half_gap) * smooth(time, JAM_HOLD, JAM_HOLD + JAM_OPEN)
        rise = WITHDRAW_Z * smooth(time, JAM_HOLD + JAM_OPEN, JAM_HOLD + JAM_OPEN + JAM_WITHDRAW)
        poses[index, 0, :3] = centre + np.array([-gap, 0.0, rise])
        poses[index, 1, :3] = centre + np.array([gap, 0.0, rise])
    return poses


def newton_commit() -> str:
    """The Newton revision this run used; a staged source tree has no .git."""
    root = Path(newton.__file__).resolve().parents[1]
    try:
        return subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return f"unknown ({root} is not a git checkout)"


def run(
    out: Path,
    *,
    steps: int,
    render_every: int = 0,
    coarse: bool = True,
    claw: str = "dynamic",
    bore: float = BORE_RADIUS,
    cap: float = 5.0,
    mu: float | None = None,
) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    sim, obj, grip, camera, ring, glass_shapes, geometry = build_scene(
        coarse=coarse, claw=claw, bore=bore, cap=cap, mu=mu
    )
    entity = obj.entity
    particles = entity.particles
    tets = entity.source_tets
    quality = MeshQuality(sim, entity)
    dt = 1.0 / (CONTROL_HZ * SUBSTEPS)
    plan = waypoints(grip, grip.entity.capacity, dt, geometry)
    # The plan's own first sample, printed because a plan is a thing that can be
    # wrong on its own: a ramp read at a negative time once put the jaws 210 mm
    # above the toy for a whole run, and no diagnostic downstream could see it.
    print(
        json.dumps(
            {
                "trajectory": {
                    "samples": int(len(plan)),
                    "start_m": [float(0.5 * (plan[0, 0, i] + plan[0, 1, i])) for i in range(3)],
                    "start_gap_m": float(plan[0, 1, 0] - plan[0, 0, 0]),
                    "closed_gap_m": float(grip.spec.closed_half_gap * 2.0),
                    "lowest_z_m": float(0.5 * (plan[:, 0, 2] + plan[:, 1, 2]).min()),
                }
            }
        ),
        flush=True,
    )
    grip.entity.set_trajectory(plan)
    q0 = sim.pipeline.state_0.particle_q.numpy()[particles]
    rest_det = np.linalg.det(q0[tets[:, 1:]] - q0[tets[:, :1]])
    if np.any(rest_det <= 0):
        raise RuntimeError("source tetrahedra were not positively oriented after build")
    mass = float(entity.source_mass.sum())
    weight = mass * 9.81
    rim = TABLE_Z + RIM_Z
    # The container is placed in the work-surface frame, the particles are read in
    # world coordinates, and the frame's origin is 0.6 m along x.
    axis = np.array([BENCH_X + TARGET_X, 0.0])
    print(json.dumps({"mass_kg": mass, "weight_n": weight, "rim_z_m": rim}), flush=True)

    diagnostics = []
    jam_frame = None
    jam_streak = 0
    depth_best = 0.0
    progress_frame = 0
    started = time.perf_counter()
    videos = None
    video_error = None
    if render_every:
        from robosimstudio.core.render.video import VideoSet

        videos = VideoSet(str(out), fps=max(1, CONTROL_HZ // render_every))
        videos.__enter__()
    try:
        for frame in range(steps):
            sim.run(1).sync()
            q = sim.pipeline.state_0.particle_q.numpy()
            body_q = sim.pipeline.state_0.body_q.numpy()
            owned = q[particles]
            if not np.isfinite(owned).all():
                raise RuntimeError(f"non-finite Clawhauser state at frame {frame}")
            det = np.linalg.det(owned[tets[:, 1:]] - owned[tets[:, :1]])
            contacts = sim.pipeline.contacts
            count = int(contacts.soft_contact_count.numpy()[0])
            contact_shapes = contacts.soft_contact_shape.numpy()[: min(count, contacts.soft_contact_max)]
            depth = penetration(q, body_q, sim.model, contacts)
            touching = depth > 0.0
            finger = np.isin(contact_shapes, grip.entity.finger_shapes)
            in_glass = np.isin(contact_shapes, glass_shapes)
            rigid_count = int(contacts.rigid_contact_count.numpy()[0])
            rigid0 = contacts.rigid_contact_shape0.numpy()[: min(rigid_count, contacts.rigid_contact_max)]
            rigid1 = contacts.rigid_contact_shape1.numpy()[: min(rigid_count, contacts.rigid_contact_max)]
            pads_in_glass = int(
                np.count_nonzero(
                    np.isin(rigid0, grip.entity.finger_shapes) & np.isin(rigid1, glass_shapes)
                    | np.isin(rigid1, grip.entity.finger_shapes) & np.isin(rigid0, glass_shapes)
                )
            )
            # What is inside: below the rim, and within the container's own bore
            # radius of its axis. The two are reported separately because a toy
            # resting on the rim is below the rim at its tail and beside the bore
            # at its body.
            within = np.linalg.norm(owned[:, :2] - axis, axis=1) < bore + 2.0 * WALL
            below_rim = float(np.mean(owned[:, 2] < rim))
            inside = float(np.mean(within & (owned[:, 2] < rim)))
            # How deep the part of the toy that is over the bore went: a plush
            # lying on the bench beside the container is 90 mm below the rim plane
            # and has inserted nothing.
            over_bore = float(rim - owned[within, 2].min()) if np.any(within) else 0.0
            row = {
                "frame": frame + 1,
                "time_s": (frame + 1) / CONTROL_HZ,
                "minimum_z_m": float(owned[:, 2].min()),
                "insertion_depth_m": max(0.0, over_bore),
                "below_rim_fraction": below_rim,
                "inside_fraction": inside,
                "ring_centre_m": owned[ring["loop_ids"]].mean(axis=0).tolist(),
                "minimum_tet_volume_ratio": float((det / rest_det).min()),
                "inverted_tetrahedra": int(np.count_nonzero(det <= 0)),
                "soft_contacts": count,
                "finger_contacts": int(np.count_nonzero(finger & touching)),
                "container_contacts": int(np.count_nonzero(in_glass & touching)),
                "finger_container_contacts": pads_in_glass,
                "max_penetration_m": float(depth.max()) if depth.size else 0.0,
                "finger_positions": [[float(v) for v in body_q[b][:3]] for b in grip.entity.bodies],
            }
            diagnostics.append(row)
            if row["insertion_depth_m"] > depth_best + JAM_STEP:
                depth_best = row["insertion_depth_m"]
                progress_frame = frame
            elif depth_best <= 0.0:
                # The toy is still above the rim, so there is nothing to stall yet
                # and the progress clock has to move with the descent. Left at
                # frame zero it is already 8 frames stale the moment the first
                # millimetre appears, and the guard fires on that millimetre:
                # measured, the dynamic run's jam at frame 196 -- the descent's
                # second frame, jaw contacts 0, depth 0.0 -- after which the pads
                # held in mid-air, the toy fell when they opened, and its body was
                # crushed against the bench (J -1.002, 20 frames inverted).
                progress_frame = frame
            # Armed once the toy's own lowest point is under the rim: before that
            # the depth is the claw's descent and cannot stall.
            armed = depth_best > 0.0 and frame >= round(PHASES["descend"][0] * CONTROL_HZ)
            jam_streak = jam_streak + 1 if armed and frame - progress_frame >= JAM_FRAMES else 0
            if jam_frame is None and jam_streak > 0:
                jam_frame = frame + 1
                fingers = np.array([[body_q[b][0], body_q[b][1], body_q[b][2]] for b in grip.entity.bodies])
                grip.entity.set_trajectory(after_jam(fingers, dt, grip.spec.closed_half_gap))
                print(json.dumps({"jam_frame": jam_frame, "depth_m": depth_best}), flush=True)
            if claw == "dynamic":
                row["applied"] = grip.entity.applied_wrenches.numpy().tolist()
                row["peak_force_n"] = grip.entity.peak_force.numpy().tolist()
                row["peak_torque_nm"] = grip.entity.peak_torque.numpy().tolist()
            if frame % 15 == 0 or frame == steps - 1:
                print(json.dumps(row), flush=True)
            if videos is not None and frame % render_every == 0:
                try:
                    videos.append_frames({camera.name: sim.render(camera)})
                except Exception as exc:  # recorded, not swallowed
                    video_error = repr(exc)
                    videos = None
    finally:
        if videos is not None:
            try:
                videos.__exit__(None, None, None)
            except Exception as exc:  # recorded, not swallowed
                video_error = repr(exc)

    (out / "diagnostics.json").write_text(json.dumps(diagnostics, indent=2) + "\n")
    (out / "manifest.json").write_text(
        json.dumps(
            {
                "scene": "clawhauser_stuff_into_bottle",
                "asset": "clawhauser",
                "newton_version": newton.__version__,
                "newton_source_commit": newton_commit(),
                "solver": type(sim.pipeline.program.solver).__name__,
                "coarse": bool(coarse),
                "coarse_contact_gap_m": CONTACT_GAP if coarse else None,
                "coarse_budget_m": COARSE_BUDGET if coarse else None,
                "claw": claw,
                "geometry": geometry,
                "glass_panels": PANELS,
                "wall_m": WALL,
                "bore_radius_m": bore,
                "rim_z_m": TABLE_Z + RIM_Z,
                "floor_z_m": TABLE_Z + FLOOR_Z,
                "mass_kg": mass,
                "global_soft_contact_mu": float(sim.model.soft_contact_mu),
                "attachment_particles": int(sim.model.attachment_particle_body_count),
                "completed_steps": len(diagnostics),
                "mean_frame_ms": 1000.0 * (time.perf_counter() - started) / max(1, len(diagnostics)),
            },
            indent=2,
        )
        + "\n"
    )
    return judge(out, diagnostics, quality, geometry, entity, steps, jam_frame=jam_frame, video_error=video_error)


def judge(out: Path, diagnostics, quality, geometry, entity, steps, *, jam_frame, video_error) -> dict:
    """What the run did, stated as measurements rather than as a verdict.

    The depth is the point: the container is smaller than the toy on purpose, so a
    run that stops on the rim and one that goes to the floor are both outcomes and
    the number says which happened. What is gated is what the *simulation* has to
    get right -- bounded jaw effort, bounded penetration, an unmangled mesh, and a
    toy that is not still in the jaws at the end.
    """
    weight = float(entity.source_mass.sum() * 9.81)
    settled = diagnostics[-max(2, int(0.5 * CONTROL_HZ)) :]
    deepest = max(diagnostics, key=lambda row: row["insertion_depth_m"])
    reach = deepest["insertion_depth_m"]
    inside = max(row["inside_fraction"] for row in diagnostics)
    released = max(row["finger_contacts"] for row in settled) == 0
    settled_clearance = min(row["minimum_z_m"] for row in settled) - TABLE_Z
    # The pads' own clearance over the rim, which is what the container can block:
    # this bore lets them in, a deeper one need not.
    pad_z = min(min(row["finger_positions"][side][2] for side in (0, 1)) for row in diagnostics) - (TABLE_Z + RIM_Z)
    peak_force = max((max(row["peak_force_n"]) for row in diagnostics if "peak_force_n" in row), default=0.0)
    peak_torque = max((max(row["peak_torque_nm"]) for row in diagnostics if "peak_torque_nm" in row), default=0.0)
    quality_read = quality.read()
    inverted_substeps = sum(quality_read["inverted_tet_substeps_by_part"])
    nonfinite_substeps = sum(quality_read["nonfinite_tet_substeps_by_part"])
    on_bench = settled_clearance <= DROP_CLEARANCE
    wedged = bool(released and not on_bench and settled[-1]["below_rim_fraction"] > 0.02)
    inserted = bool(released and reach >= RIM_Z - FLOOR_Z - 0.004)
    if nonfinite_substeps or inverted_substeps or min(quality_read["minimum_J_by_part"]) <= 0.0:
        stop = "solver-artifact"
    elif on_bench:
        stop = "object-dropped"
    elif pad_z < 0.0:
        stop = "rim-blocks-pads"
    elif reach >= RIM_Z - FLOOR_Z - 0.004:
        stop = "inserted"
    else:
        # The descent ended with the toy held above the floor: whatever stopped it
        # was the object meeting the container rather than the claw letting go.
        stop = "object-jammed"
    result = {
        "geometry": geometry,
        "weight_n": weight,
        "jammed_frame": jam_frame,
        "max_insertion_depth_m": reach,
        "max_insertion_depth_frame": deepest["frame"],
        "max_below_rim_fraction": max(row["below_rim_fraction"] for row in diagnostics),
        "max_inside_fraction": inside,
        "settled_clearance_m": settled_clearance,
        "settled_inside_fraction": settled[-1]["inside_fraction"],
        "settled_below_rim_fraction": settled[-1]["below_rim_fraction"],
        "released": released,
        "wedged_after_release": wedged,
        "inserted_to_floor": inserted,
        "pad_clearance_over_rim_m": pad_z,
        "stop_reason": stop,
        "peak_force_n": peak_force,
        "peak_torque_nm": peak_torque,
        "max_penetration_m": max(row["max_penetration_m"] for row in diagnostics),
        "max_finger_contacts": max(row["finger_contacts"] for row in diagnostics),
        "max_container_contacts": max(row["container_contacts"] for row in diagnostics),
        "max_finger_container_contacts": max(row["finger_container_contacts"] for row in diagnostics),
        "max_inverted_tetrahedra": max(row["inverted_tetrahedra"] for row in diagnostics),
        "settled_inverted_tetrahedra": max(row["inverted_tetrahedra"] for row in settled),
        "quality": quality_read,
        "steps": len(diagnostics),
        "video_error": video_error,
    }
    result["success"] = bool(
        steps >= round(DURATION * CONTROL_HZ)
        and reach > 0.020
        and released
        and not on_bench
        and stop != "solver-artifact"
        and result["max_penetration_m"] <= 0.0005
        and nonfinite_substeps == 0
        and min(quality_read["minimum_J_by_part"]) > 0.0
        # The clamp is applied in float32, so a run that sits on it reports the
        # cap's own representation -- 5.0000005 against a float64 5.0. One part in
        # a million is the arithmetic, not the physics: measured, the b6-dyn run
        # was refused its success by 4.8e-7 N.
        and peak_force <= float(geometry["force_cap_n"]) * (1.0 + 1.0e-6)
        and peak_torque <= 0.03
    )
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUTPUT / "run")
    parser.add_argument("--steps", type=int, default=round(DURATION * CONTROL_HZ))
    parser.add_argument("--render-every", type=int, default=0)
    parser.add_argument("--claw", choices=("dynamic", "kinematic"), default="dynamic")
    parser.add_argument("--no-coarse", dest="coarse", action="store_false")
    parser.add_argument("--bore", type=float, default=BORE_RADIUS, help="container inner radius [m]")
    parser.add_argument("--cap", type=float, default=5.0, help="jaw force clamp [N]")
    parser.add_argument("--mu", type=float, default=None, help="global soft-contact friction coefficient")
    args = parser.parse_args()
    run(
        args.out,
        steps=args.steps,
        render_every=args.render_every,
        coarse=args.coarse,
        claw=args.claw,
        bore=args.bore,
        cap=args.cap,
        mu=args.mu,
    )


if __name__ == "__main__":
    main()
