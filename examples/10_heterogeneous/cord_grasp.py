# SPDX-License-Identifier: Apache-2.0
"""Commands for pinching one observed upper-cord section with narrow flat pads.

The default assumes 12 mm-wide flat fingertips, with no support ledges. These
are manipulator dimensions, not changes to the labelled tetrahedral object.
The caller must build matching proxies and retain explicit actuator caps.
"""

from __future__ import annotations

import numpy as np
from trajectory import _path, _points, _poses

DEFAULT_PAD_HALF = (0.003, 0.006, 0.006)


def select_apex(source_vertices, tets, labels):
    """Source vertex IDs for one cord section and a wider rigid-frame fit.

    Clawhauser's labelled loop is part 2 and its apex is along source +y.
    A narrow x slab through that apex selects one cord, leaving the opening
    and the two distant side strands out of the pinch-width measurement.
    """
    vertices = _points(source_vertices, "source_vertices", minimum=4)
    tets, labels = np.asarray(tets, dtype=int), np.asarray(labels)
    if tets.ndim != 2 or tets.shape[1] != 4 or labels.shape != (len(tets),) or not np.any(labels == 2):
        raise ValueError("apex grasp requires tetrahedra with a part-2 loop")
    if tets.min() < 0 or tets.max() >= len(vertices):
        raise ValueError("tetrahedron index outside source vertices")
    ring = np.unique(tets[labels == 2])
    points = vertices[ring]
    tip_y = float(points[:, 1].max())
    top = points[points[:, 1] > tip_y - 0.002]
    tip_x = float(top[:, 0].mean())
    apex = ring[(np.abs(points[:, 0] - tip_x) < 0.004) & (points[:, 1] > tip_y - 0.008)]
    fit = ring[(np.abs(points[:, 0] - tip_x) < 0.012) & (points[:, 1] > tip_y - 0.012)]
    if len(apex) < 8 or len(fit) < 12:
        raise ValueError("source mesh has too few vertices in the intended apex cord")
    return apex, fit, ring


def _quaternion(rotation):
    """A proper rotation matrix to Newton/Warp's xyzw quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        q = (
            np.array(
                (
                    rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1],
                    s * s / 4,
                )
            )
            / s
        )
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(0.0, 1 + rotation[i, i] - rotation[j, j] - rotation[k, k])) * 2
        q = np.zeros(4)
        q[i] = s / 4
        q[j], q[k] = (rotation[j, i] + rotation[i, j]) / s, (rotation[k, i] + rotation[i, k]) / s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def _rotation(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
    )


def observe_cord(current_positions, source_vertices, tets, labels):
    """Measure the current section/frame; reusable through approach and close.

    Source correspondences fit a local rigid rotation to the upper arc; they do
    not constrain its physical motion. Local gripper y follows the fitted cord
    tangent (source x). Closing x is horizontal and perpendicular to that
    tangent, approximately world y, which keeps both jaw centres at the same
    height above the table. The round cord does not require closing in its
    tilted loop plane. Local z faces upward.
    """
    source = _points(source_vertices, "source_vertices", minimum=4)
    current = _points(current_positions, "current_positions", minimum=4)
    if current.shape != source.shape:
        raise ValueError("current positions must follow the source vertex order")
    apex, fit, ring = select_apex(source, tets, labels)
    a, b = source[fit], current[fit]
    ac, bc = a - a.mean(axis=0), b - b.mean(axis=0)
    u, _, vt = np.linalg.svd(ac.T @ bc)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    fitted = vt.T @ correction @ u.T
    tangent = fitted[:, 0].copy()
    closing = np.array((-tangent[1], tangent[0], 0.0))
    if np.linalg.norm(closing) < 1.0e-6:
        raise ValueError("cord tangent is vertical; a stable horizontal grasp frame is undefined")
    closing /= np.linalg.norm(closing)
    if closing[1] < 0:
        closing *= -1
    up = np.cross(closing, tangent)
    if up[2] < 0:
        tangent *= -1
        up *= -1
    rotation = np.column_stack((closing, tangent, up))
    local = current[apex] @ rotation
    lower, upper = local.min(axis=0), local.max(axis=0)
    center = rotation @ ((lower + upper) / 2)
    residual = ac @ fitted.T - bc
    return {
        "center": center,
        "rotation": rotation,
        "quaternion": _quaternion(rotation),
        "width_m": float(upper[0] - lower[0]),
        "section_bounds_in_gripper_m": np.stack((lower - center @ rotation, upper - center @ rotation)),
        "fit_rms_m": float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))),
        "apex_ids": apex,
        "fit_ids": fit,
        "ring_ids": ring,
    }


def cord_targets(observation, half_gap, *, center=None):
    """Two targets from a fresh observation, for a bounded-servo close policy."""
    center = observation["center"] if center is None else np.asarray(center, dtype=float)
    poses = np.zeros((2, 7))
    poses[:, :3] = center
    poses[:, 3:] = observation["quaternion"]
    poses[0, :3] -= half_gap * observation["rotation"][:, 0]
    poses[1, :3] += half_gap * observation["rotation"][:, 0]
    return poses


def _boundary(tets):
    faces = np.concatenate([tets[:, ids] for ids in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3))])
    _, indices, counts = np.unique(np.sort(faces, axis=1), axis=0, return_index=True, return_counts=True)
    return faces[indices[counts == 1]]


def _swept_box_hits(triangles, half, travel):
    """Exact SAT coverage for a triangle and a box extruded along one segment.

    Expanded box half-extents conservatively include the requested clearance.
    This is a frozen-geometry approach check, not continuous detection against
    a deforming future state. Reobserve before closure.
    """
    edges = (triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 1], triangles[:, 0] - triangles[:, 2])
    axes = [np.broadcast_to(axis, (len(triangles), 3)) for axis in np.eye(3)]
    axes += [np.broadcast_to(np.cross(travel, axis), (len(triangles), 3)) for axis in np.eye(3)]
    axes += [np.cross(edges[0], edges[1])]
    axes += [np.cross(edge, axis) for edge in edges for axis in (*np.eye(3), travel)]
    hit = np.ones(len(triangles), dtype=bool)
    for axis in axes:
        projection = np.sum(triangles * axis[:, None, :], axis=2)
        radius = np.abs(axis) @ half
        shift = axis @ travel
        hit &= (projection.min(axis=1) <= radius + np.maximum(shift, 0)) & (
            projection.max(axis=1) >= -radius + np.minimum(shift, 0)
        )
    return hit


def _minimum_proxy_z(poses, pad_half):
    return min(float(pose[2] - np.abs(_rotation(pose[3:])[2]) @ pad_half) for pose in poses)


def table_safe_cord_center(observation, pad_half, table_z, table_clearance=0.0005, max_center_raise=0.002):
    """Raise only the drive target slightly when the cord remains inside the pad.

    The closing axis is horizontal, so this leaves the measured closing width
    unchanged. A raise over the declared bound, or one leaving less than half
    the observed cord section under the pad vertically/tangentially, is refused.
    No object coordinates or physical fingertip poses are changed here.
    """
    center = observation["center"].copy()
    rotation = observation["rotation"]
    pad_half = np.asarray(pad_half, dtype=float)
    if not np.isfinite(max_center_raise) or max_center_raise < 0:
        raise ValueError("max_center_raise must be finite and nonnegative")
    min_center_z = table_z + table_clearance + np.abs(rotation[2]) @ pad_half
    offset = max(0.0, float(min_center_z - center[2]))
    if offset > max_center_raise + 1.0e-9:
        raise ValueError(f"apex needs {offset:.6f} m target raise, beyond the {max_center_raise:.6f} m safe bound")
    center[2] += offset
    relative_bounds = observation["section_bounds_in_gripper_m"] - np.array((0.0, 0.0, offset)) @ rotation
    widths = relative_bounds[1] - relative_bounds[0]
    overlaps = np.maximum(0.0, np.minimum(relative_bounds[1], pad_half) - np.maximum(relative_bounds[0], -pad_half))
    if np.any(overlaps[1:] < 0.5 * widths[1:]):
        raise ValueError("table-safe target would leave less than half the observed cord section under the pad")
    return center, {
        "observed_cord_center_world_m": observation["center"].tolist(),
        "grasp_center_world_m": center.tolist(),
        "target_center_raise_m": offset,
        "maximum_target_center_raise_m": float(max_center_raise),
        "section_bounds_relative_to_target_m": relative_bounds.tolist(),
        "pad_section_overlap_tangent_m": float(overlaps[1]),
        "pad_section_overlap_vertical_m": float(overlaps[2]),
        "pad_section_overlap_vertical_fraction": float(overlaps[2] / max(widths[2], 1.0e-12)),
        "minimum_commanded_pad_table_clearance_m": float(center[2] - np.abs(rotation[2]) @ pad_half - table_z),
    }


def cord_approach_displacement(observation, current, pad_half, center, *, normal_approach=False):
    """From the grasp centre to a hover with the existing vertical clearance.

    The optional tool-normal approach changes only the command direction. Its
    upward component must be at least 0.1, avoiding an arbitrarily long lateral
    approach when the fitted tool normal becomes almost horizontal.
    """
    highest = float(current[:, 2].max() + 2 * np.linalg.norm(pad_half) + 0.025)
    rise = max(0.04, highest - center[2])
    if not normal_approach:
        return np.array((0.0, 0.0, rise))
    direction = np.asarray(observation["rotation"][:, 2], dtype=float)
    if not np.isfinite(direction).all() or not np.isclose(np.linalg.norm(direction), 1.0):
        raise ValueError("observed approach normal must be finite and unit length")
    if direction[2] < 0.1:
        raise ValueError("observed approach normal must have upward component at least 0.1")
    return direction * (rise / direction[2])


def _approach_geometry(
    observation,
    current,
    tets,
    pad_half,
    table_z,
    particle_radius,
    clearance,
    table_clearance,
    max_center_raise,
    *,
    normal_approach=False,
):
    center, center_geometry = table_safe_cord_center(observation, pad_half, table_z, table_clearance, max_center_raise)
    rotation = observation["rotation"]
    travel_world = cord_approach_displacement(observation, current, pad_half, center, normal_approach=normal_approach)
    center_geometry.update(
        {
            "normal_approach": bool(normal_approach),
            "approach_direction_world": (travel_world / np.linalg.norm(travel_world)).tolist(),
            "approach_displacement_world_m": travel_world.tolist(),
        }
    )
    local_triangles = current[_boundary(tets)] @ rotation
    half_width = observation["width_m"] / 2
    base_gap = half_width + pad_half[0] + particle_radius + clearance + 0.001
    candidates = base_gap + np.array((0.0, 0.002, 0.004, 0.006, 0.009, 0.013, 0.018, 0.025, 0.035, 0.05))
    # A clear window can lie between the sparse samples after the toy settles.
    # Preserve the cheap original path, then refine the same aperture range.
    # Each additional candidate still faces the identical whole-boundary check.
    midpoints = (candidates[:-1] + candidates[1:]) / 2
    refinement = base_gap + np.arange(0.0005, 0.05, 0.0005)
    candidates = np.concatenate((candidates, midpoints, refinement))
    trials = []
    expanded = pad_half + particle_radius + clearance
    for gap in candidates:
        poses = cord_targets(observation, gap, center=center)
        hover = poses.copy()
        hover[:, :3] += travel_world
        if min(_minimum_proxy_z(poses, pad_half), _minimum_proxy_z(hover, pad_half)) < (
            table_z + table_clearance - 1.0e-9
        ):
            trials.append({"half_gap_m": float(gap), "rejected": "oriented pad would intersect table clearance"})
            continue
        collisions = []
        for pose in poses:
            local = local_triangles - pose[:3] @ rotation
            collisions.append(int(_swept_box_hits(local, expanded, rotation.T @ travel_world).sum()))
        trials.append({"half_gap_m": float(gap), "swept_triangle_overlaps_by_side": collisions})
        if not any(collisions):
            return poses, hover, trials, center_geometry
    raise ValueError(f"no collision-clear static approach for these fingertip dimensions: {trials}")


def build_cord_pick_trajectory(
    start_poses,
    current_positions,
    source_vertices,
    tets,
    labels,
    *,
    dt,
    table_z=0.75,
    pad_half=DEFAULT_PAD_HALF,
    particle_radius=0.0007,
    compression_per_side=0.0007,
    approach_clearance=0.001,
    table_clearance=0.0005,
    displacement=(0.18, -0.10),
    lift_height=0.35,
    elapsed_before_s=0.0,
    approach_only=False,
    max_center_raise=0.002,
    parallel=False,
    normal_approach=False,
):
    """Plan a quantified single-cord pinch, or just its approach for reacquisition.

    Returns ``poses, metadata, apex_ids``. Default flat pads are 6 x 12 x 12 mm
    (full dimensions) with NO LEDGES. Keep finite force/torque caps in the
    dynamic gripper. ``approach_only=True`` ends before closing so the caller
    can obtain fresh ``observe_cord`` measurements and use ``cord_targets``
    through closure. A static complete path is an experiment, not a guarantee
    that a moving cord remains under its initial targets. ``normal_approach``
    approaches along the fitted tool normal instead of world vertical.
    """
    start = _poses(start_poses)
    current = _points(current_positions, "current_positions", minimum=4)
    tets = np.asarray(tets, dtype=int)
    pad_half = np.asarray(pad_half, dtype=float)
    if pad_half.shape != (3,) or not np.isfinite(pad_half).all() or np.any(pad_half <= 0):
        raise ValueError("pad_half must contain three positive dimensions")
    observation = observe_cord(current, source_vertices, tets, labels)
    closed_gap = observation["width_m"] / 2 + pad_half[0] + particle_radius - compression_per_side
    if closed_gap <= pad_half[0] + 0.0002:
        raise ValueError("requested compression would make the flat fingers collide")
    opened, hover, candidates, center_geometry = _approach_geometry(
        observation,
        current,
        tets,
        pad_half,
        table_z,
        particle_radius,
        approach_clearance,
        table_clearance,
        max_center_raise,
        normal_approach=normal_approach,
    )
    safe = start.copy()
    safe[:, 2] += max(0.0, float(hover[:, 2].max() - _minimum_proxy_z(start, pad_half)))
    closed = cord_targets(observation, closed_gap, center=opened[:, :3].mean(axis=0))
    lifted = closed.copy()
    lifted[:, 2] += lift_height
    transferred = lifted.copy()
    transferred[:, :2] += np.asarray(displacement)
    waypoints = [("raise_clear", safe, 0.5), ("align_above_cord", hover, 0.7), ("approach", opened, 0.7)]
    if not approach_only:
        waypoints.extend(
            (
                ("close", closed, 0.6),
                ("grasp_hold", closed, 0.4),
                ("lift", lifted, 2.4),
                ("lift_hold", lifted, 0.7),
                ("transport", transferred, 1.8),
                ("transport_hold", transferred, 0.3),
            )
        )
    poses, metadata = _path(start, waypoints, dt, elapsed_before_s, parallel=parallel)
    metadata.update(
        {
            "grasp": "single_upper_cord",
            "pad_half_extents_m": pad_half.tolist(),
            "support_ledges": False,
            "apex_source_ids": observation["apex_ids"].tolist(),
            "frame_fit_source_ids": observation["fit_ids"].tolist(),
            "grasp_center_world_m": observation["center"].tolist(),
            "grasp_quaternion_xyzw": observation["quaternion"].tolist(),
            "closing_axis_world": observation["rotation"][:, 0].tolist(),
            "cord_tangent_world": observation["rotation"][:, 1].tolist(),
            "frame_fit_rms_m": observation["fit_rms_m"],
            "observed_cord_width_m": observation["width_m"],
            "observed_section_bounds_in_gripper_m": observation["section_bounds_in_gripper_m"].tolist(),
            "closed_half_gap_m": float(closed_gap),
            "target_inner_gap_m": float(2 * (closed_gap - pad_half[0])),
            "compression_per_side_m": float(compression_per_side),
            "approach_candidates": candidates,
            "approach_frozen_mesh_clearance_m": float(approach_clearance),
            "reobserve_before_close": True,
            "approach_end_s": metadata["phases"]["approach"]["end_s"],
            "target_translation_xy_m": list(displacement),
        }
    )
    metadata.update(center_geometry)
    if not approach_only:
        metadata["grasp_check_s"] = metadata["phases"]["grasp_hold"]["end_s"]
        metadata["transport_window_s"] = [
            metadata["phases"]["transport"]["start_s"],
            metadata["phases"]["transport_hold"]["end_s"],
        ]
    return poses, metadata, observation["apex_ids"]


def build_cord_close_transfer(
    start_poses,
    current_positions,
    source_vertices,
    tets,
    labels,
    *,
    dt,
    table_z=0.75,
    pad_half=DEFAULT_PAD_HALF,
    particle_radius=0.0007,
    compression_per_side=0.0007,
    table_clearance=0.0005,
    displacement=(0.18, -0.10),
    lift_height=0.35,
    elapsed_before_s=0.0,
    max_center_raise=0.002,
):
    """Reobserve after approach, then close, hold, lift and transport.

    No second approach is inserted. All target positions and the cord tangent
    come from the supplied CURRENT particle positions; the source coordinates
    supply only stable apex membership and frame correspondences. A moving
    cord can still require feedback during closure: actual bilateral load is
    the grasp criterion, rather than completion of this command sequence.
    """
    start = _poses(start_poses)
    pad_half = np.asarray(pad_half, dtype=float)
    displacement = np.asarray(displacement, dtype=float)
    if pad_half.shape != (3,) or not np.isfinite(pad_half).all() or np.any(pad_half <= 0):
        raise ValueError("pad_half must contain three positive dimensions")
    if displacement.shape != (2,) or not np.isfinite(displacement).all():
        raise ValueError("displacement must contain two finite coordinates")
    if not np.isfinite(lift_height) or lift_height <= 0:
        raise ValueError("lift_height must be positive")
    if not np.isfinite(compression_per_side) or compression_per_side < 0:
        raise ValueError("compression_per_side must be nonnegative")
    observation = observe_cord(current_positions, source_vertices, tets, labels)
    closed_gap = observation["width_m"] / 2 + pad_half[0] + particle_radius - compression_per_side
    if closed_gap <= pad_half[0] + 0.0002:
        raise ValueError("requested compression would make the flat fingers collide")
    center, center_geometry = table_safe_cord_center(observation, pad_half, table_z, table_clearance, max_center_raise)
    closed = cord_targets(observation, closed_gap, center=center)
    if _minimum_proxy_z(closed, pad_half) < table_z + table_clearance - 1.0e-9:
        raise ValueError("reobserved cord is too low for the oriented flat fingertips")
    lifted = closed.copy()
    lifted[:, 2] += lift_height
    transferred = lifted.copy()
    transferred[:, :2] += displacement
    waypoints = (
        ("close", closed, 0.8),
        ("grasp_hold", closed, 0.4),
        ("lift", lifted, 2.4),
        ("lift_hold", lifted, 0.7),
        ("transport", transferred, 1.8),
        ("transport_hold", transferred, 0.3),
    )
    poses, metadata = _path(start, waypoints, dt, elapsed_before_s)
    metadata.update(
        {
            "grasp": "single_upper_cord_reacquired",
            "pad_half_extents_m": pad_half.tolist(),
            "support_ledges": False,
            "apex_source_ids": observation["apex_ids"].tolist(),
            "frame_fit_source_ids": observation["fit_ids"].tolist(),
            "grasp_center_world_m": observation["center"].tolist(),
            "grasp_quaternion_xyzw": observation["quaternion"].tolist(),
            "closing_axis_world": observation["rotation"][:, 0].tolist(),
            "cord_tangent_world": observation["rotation"][:, 1].tolist(),
            "frame_fit_rms_m": observation["fit_rms_m"],
            "observed_cord_width_m": observation["width_m"],
            "observed_section_bounds_in_gripper_m": observation["section_bounds_in_gripper_m"].tolist(),
            "closed_half_gap_m": float(closed_gap),
            "target_inner_gap_m": float(2 * (closed_gap - pad_half[0])),
            "compression_per_side_m": float(compression_per_side),
            "reacquisition_center_shift_m": (observation["center"] - start[:, :3].mean(axis=0)).tolist(),
            "commanded_center_shift_m": (center - start[:, :3].mean(axis=0)).tolist(),
            "target_translation_xy_m": displacement.tolist(),
            "grasp_check_s": metadata["phases"]["grasp_hold"]["end_s"],
            "transport_window_s": [
                metadata["phases"]["transport"]["start_s"],
                metadata["phases"]["transport_hold"]["end_s"],
            ],
        }
    )
    metadata.update(center_geometry)
    return poses, metadata, observation["apex_ids"]


def build_cord_place_trajectory(
    start_poses,
    current_object_points,
    *,
    dt,
    table_z=0.75,
    pad_half=DEFAULT_PAD_HALF,
    required_clearance=0.05,
    support_center_clearance=0.0007,
    release_half_gap=0.025,
    elapsed_before_s=0.0,
    desired_body_xy=None,
    observed_body_points=None,
    parallel=False,
):
    """Place while preserving fitted orientation; open along each local x axis."""
    start = _poses(start_poses)
    points = _points(current_object_points, "current_object_points")
    pad_half = np.asarray(pad_half, dtype=float)
    clearance = float(points[:, 2].min() - table_z)
    if clearance < required_clearance:
        raise ValueError(f"object transport clearance {clearance:.6f} < {required_clearance:.6f} m")
    lowered = start.copy()
    lowered[:, 2] -= clearance - support_center_clearance
    correction_xy = np.zeros(2)
    if desired_body_xy is not None:
        target = np.asarray(desired_body_xy, dtype=float)
        if target.shape != (2,) or not np.isfinite(target).all() or observed_body_points is None:
            raise ValueError("desired_body_xy needs two finite coordinates and observed_body_points")
        body = _points(observed_body_points, "observed_body_points")
        correction_xy = target - body[:, :2].mean(axis=0)
        lowered[:, :2] += correction_xy
    elif observed_body_points is not None:
        raise ValueError("observed_body_points needs desired_body_xy")
    if _minimum_proxy_z(lowered, pad_half) < table_z:
        raise ValueError("observed placement would put the oriented fingertips through the table")
    axis = _rotation(start[0, 3:])[:, 0] + _rotation(start[1, 3:])[:, 0]
    axis /= np.linalg.norm(axis)
    center = lowered[:, :3].mean(axis=0)
    if release_half_gap <= abs(np.dot(lowered[1, :3] - lowered[0, :3], axis)) / 2:
        raise ValueError("release_half_gap must open the current pinch")
    opened = lowered.copy()
    opened[0, :3], opened[1, :3] = center - release_half_gap * axis, center + release_half_gap * axis
    if _minimum_proxy_z(opened, pad_half) < table_z:
        raise ValueError("opening along the tilted closing axis would intersect the table")
    retreat = opened.copy()
    retreat[:, 2] += 0.07
    waypoints = (
        ("placement_hold", start, 0.4),
        ("lower_to_support", lowered, 2.0),
        ("support_hold", lowered, 0.6),
        ("release", opened, 0.8),
        ("retreat", retreat, 0.7),
        ("settle", retreat, 1.5),
    )
    poses, metadata = _path(start, waypoints, dt, elapsed_before_s, parallel=parallel)
    metadata.update(
        {
            "support_check_s": metadata["phases"]["support_hold"]["end_s"],
            "observed_initial_clearance_m": clearance,
            "settled_window_s": [metadata["phases"]["settle"]["start_s"], metadata["phases"]["settle"]["end_s"]],
            "closing_axis_world": axis.tolist(),
            "support_ledges": False,
            "body_xy_correction_m": correction_xy.tolist(),
            "desired_body_xy_m": None if desired_body_xy is None else np.asarray(desired_body_xy).tolist(),
        }
    )
    return poses, metadata
