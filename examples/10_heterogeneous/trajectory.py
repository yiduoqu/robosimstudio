# SPDX-License-Identifier: Apache-2.0
"""Observed-geometry target paths for the bounded floating-finger example.

These helpers produce commands for ``ScriptedGripper.set_trajectory``. They
never change physical state, materials, attachments, or actuator limits. Split
pickup/transport from placement so placement starts from the observed hanging
object instead of lowering the gripper through an assumed table contact.
"""

from __future__ import annotations

import numpy as np

# Match the existing pad/ledge proxies in gripper.py, in metres.
PAD_HALF_X = 0.003
LEDGE_BOTTOM_Z = -0.008
LEDGE_TOP_Z = -0.0056


def _points(values, name, minimum=1):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < minimum or not np.isfinite(values).all():
        raise ValueError(f"{name} must be finite [N, 3] points with at least {minimum} rows")
    return values


def _poses(values):
    values = np.asarray(values, dtype=np.float64).copy()
    if values.shape != (2, 7) or not np.isfinite(values).all():
        raise ValueError("start_poses must be two finite world-frame transforms [2, 7]")
    norms = np.linalg.norm(values[:, 3:], axis=1, keepdims=True)
    if np.any(norms < 1.0e-8):
        raise ValueError("start quaternions cannot be zero")
    values[:, 3:] /= norms
    return values


def _fingers(center, half_gap):
    poses = np.zeros((2, 7), dtype=np.float64)
    poses[:, :3] = center
    poses[:, 6] = 1.0
    poses[0, 0] -= half_gap
    poses[1, 0] += half_gap
    return poses


def _parallel_geometry(poses):
    """Common carrier midpoint and aperture for a consistent parallel pair."""
    left, right = poses
    if abs(np.dot(left[3:], right[3:])) < 1 - 1.0e-7:
        raise ValueError("parallel waypoints require a common orientation")
    x, y, z, w = left[3:]
    axis = np.array((1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)))
    delta = right[:3] - left[:3]
    gap = np.dot(delta, axis) / 2
    if gap <= 0 or np.linalg.norm(delta - 2 * gap * axis) > 1.0e-5:
        raise ValueError("parallel waypoint separation must follow its positive closing axis")
    return poses[:, :3].mean(axis=0), gap


def _path(start, waypoints, dt, elapsed_before_s, *, parallel=False):
    """Sample a few explicit phases with zero velocity at their boundaries."""
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(elapsed_before_s) or elapsed_before_s < 0:
        raise ValueError("dt must be positive and elapsed_before_s nonnegative")
    previous = _poses(start)
    chunks = [previous[None]]
    phases = {}
    ticks = 0
    for name, waypoint, seconds in waypoints:
        goal = _poses(waypoint)
        count = max(1, int(np.ceil(seconds / dt - 1.0e-9)))
        t = np.arange(1, count + 1, dtype=np.float64) / count
        u = t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
        chunk = np.zeros((count, 2, 7), dtype=np.float64)
        chunk[:, :, :3] = previous[None, :, :3] + u[:, None, None] * (goal - previous)[None, :, :3]
        # Shortest-arc quaternion interpolation keeps reorientation smooth even
        # if a preceding dynamic hold left the fingers slightly tilted.
        qa, qb = previous[:, 3:].copy(), goal[:, 3:].copy()
        dot = np.sum(qa * qb, axis=1)
        qb[dot < 0] *= -1
        angle = np.arccos(np.clip(np.abs(dot), 0.0, 1.0))
        for finger in range(2):
            if angle[finger] < 1.0e-6:
                quat = qa[finger] + u[:, None] * (qb[finger] - qa[finger])
            else:
                quat = (
                    np.sin((1 - u[:, None]) * angle[finger]) * qa[finger]
                    + np.sin(u[:, None] * angle[finger]) * qb[finger]
                ) / np.sin(angle[finger])
            chunk[:, finger, 3:] = quat / np.linalg.norm(quat, axis=1, keepdims=True)
        if parallel:
            # Interpolate the mechanism's carrier and aperture. Independently
            # interpolated finger positions cut a chord during reorientation
            # and do not satisfy the physical slider constraints.
            center0, gap0 = _parallel_geometry(previous)
            center1, gap1 = _parallel_geometry(goal)
            center = center0 + u[:, None] * (center1 - center0)
            gap = gap0 + u * (gap1 - gap0)
            x, y, z, w = chunk[:, 0, 3:].T
            axes = np.stack((1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)), axis=1)
            chunk[:, 0, :3] = center - gap[:, None] * axes
            chunk[:, 1, :3] = center + gap[:, None] * axes
        phases[name] = {
            "start_s": float(elapsed_before_s + ticks * dt),
            "end_s": float(elapsed_before_s + (ticks + count) * dt),
            "start_sample": ticks,
            "end_sample": ticks + count,
        }
        chunks.append(chunk)
        ticks += count
        previous = goal
    return np.concatenate(chunks).astype(np.float32), {
        "dt_s": float(dt),
        "elapsed_before_s": float(elapsed_before_s),
        "duration_s": float(ticks * dt),
        "elapsed_end_s": float(elapsed_before_s + ticks * dt),
        "phases": phases,
        "parallel_carrier_interpolation": bool(parallel),
    }


def build_pick_trajectory(
    start_poses,
    settled_rim_points,
    *,
    dt,
    table_z=0.75,
    displacement=(0.18, -0.10),
    closed_half_gap=None,
    compression_per_side=0.0045,
    lift_height=0.35,
    object_points=None,
    ring_points=None,
    required_clearance=0.05,
    table_clearance=0.0003,
    particle_radius=0.0007,
    elapsed_before_s=0.0,
):
    """Approach the observed rim, close, hold, lift, and transport in contact.

    Pass positions measured after physical settling. The selected rim patch is
    the same upper-loop particle set already used by the demo. The jaws span
    world x and remain upright; this helper assumes the demo's flat starting
    pose. ``elapsed_before_s`` includes settling in the returned phase times.

    A 4.5 mm per-side target compression gives approximately a 20 mm centre
    half-gap for the current 42 mm rim. This is a target, not prescribed jaw
    motion: the dynamic force-limited servo can stall outside it. Actuator caps
    remain the caller's explicit configuration and must be validated separately.
    """
    start = _poses(start_poses)
    rim = _points(settled_rim_points, "settled_rim_points", minimum=8)
    displacement = np.asarray(displacement, dtype=np.float64)
    parameters = (table_z, compression_per_side, lift_height, required_clearance, table_clearance, particle_radius)
    if not all(np.isfinite(v) for v in parameters) or displacement.shape != (2,) or not np.isfinite(displacement).all():
        raise ValueError("trajectory geometry must be finite")
    if min(compression_per_side, required_clearance, table_clearance, particle_radius) < 0 or lift_height <= 0:
        raise ValueError("clearances/compression must be nonnegative and lift positive")
    lower, upper = rim.min(axis=0), rim.max(axis=0)
    center = (lower + upper) / 2
    half_span = (upper[0] - lower[0]) / 2
    if half_span <= PAD_HALF_X:
        raise ValueError("rim is too narrow for the current finger geometry")
    closed = half_span + PAD_HALF_X - compression_per_side if closed_half_gap is None else float(closed_half_gap)
    if not np.isfinite(closed) or closed <= PAD_HALF_X:
        raise ValueError("closed_half_gap must leave a positive gap between the pads")
    opened = max(half_span + PAD_HALF_X + 0.009, closed + 0.009)
    # The ledges sweep past the entire loop during descent. Its lower section
    # can be wider than the selected upper grasp patch and otherwise gets
    # brushed sideways before closure starts.
    if ring_points is not None:
        all_ring = _points(ring_points, "ring_points")
        opened = max(opened, float(np.max(np.abs(all_ring[:, 0] - center[0]))) + 0.009 + 0.004)
    # A finite-thickness shelf has no free space beneath a loop resting on the
    # table. Keep the WHOLE proxy above the table and pinch the upper cord;
    # ordinary compliant contact may lift its edge during closure.
    center[2] = max(center[2] + 0.003, table_z + table_clearance - LEDGE_BOTTOM_Z)
    highest = center[2] + float(lift_height)
    reach = None
    if object_points is not None:
        points = _points(object_points, "object_points")
        reach = float(np.linalg.norm(points - center, axis=1).max())
        highest = max(highest, table_z + required_clearance + reach + 0.01)
    hover = center.copy()
    hover[2] += 0.035
    lifted = center.copy()
    lifted[2] = highest
    transferred = lifted.copy()
    transferred[:2] += displacement
    open_pose, close_pose = _fingers(center, opened), _fingers(center, closed)
    lift_pose, transfer_pose = _fingers(lifted, closed), _fingers(transferred, closed)
    waypoints = (
        ("initial_hold", start, 0.3),
        ("align_above_rim", _fingers(hover, opened), 0.6),
        ("approach", open_pose, 0.8),
        ("close", close_pose, 0.8),
        ("grasp_hold", close_pose, 0.5),
        ("lift", lift_pose, 2.4),
        ("lift_hold", lift_pose, 0.7),
        ("transport", transfer_pose, 1.8),
        ("transport_hold", transfer_pose, 0.3),
    )
    poses, metadata = _path(start, waypoints, dt, elapsed_before_s)
    metadata.update(
        {
            "grasp_center_world_m": center.tolist(),
            "rim_bounds_world_m": [lower.tolist(), upper.tolist()],
            "closed_half_gap_m": float(closed),
            "open_half_gap_m": float(opened),
            "commanded_lift_m": float(highest - center[2]),
            "object_reach_from_grasp_m": reach,
            "required_transport_clearance_m": float(required_clearance),
            "target_translation_xy_m": displacement.tolist(),
            "shelf_top_above_lowest_rim_surface_m": float(center[2] + LEDGE_TOP_Z - (lower[2] - particle_radius)),
            "minimum_commanded_rigid_table_clearance_m": float(center[2] + LEDGE_BOTTOM_Z - table_z),
            "grasp_check_s": metadata["phases"]["grasp_hold"]["end_s"],
            "transport_window_s": [
                metadata["phases"]["transport"]["start_s"],
                metadata["phases"]["transport_hold"]["end_s"],
            ],
        }
    )
    return poses, metadata


def build_place_trajectory(
    start_poses,
    current_object_points,
    *,
    dt,
    table_z=0.75,
    required_clearance=0.05,
    support_center_clearance=0.0007,
    release_half_gap=0.070,
    elapsed_before_s=0.0,
):
    """Lower the observed hanging object onto support, then open and withdraw.

    Call only after verifying actual pickup/transport and all-particle clearance
    throughout transport. This function also rejects an object already below
    the required clearance. It translates targets downward by the observed
    lowest-particle height, holds before release, and never tips a shelf or
    continues lowering after the jaws open. Check actual table support at the
    returned ``support_check_s``; failed support is not a successful placement.
    """
    start = _poses(start_poses)
    points = _points(current_object_points, "current_object_points")
    parameters = (table_z, required_clearance, support_center_clearance, release_half_gap)
    if not all(np.isfinite(v) for v in parameters) or min(required_clearance, support_center_clearance) < 0:
        raise ValueError("placement geometry must be finite and clearances nonnegative")
    clearance = float(points[:, 2].min() - table_z)
    if clearance < required_clearance:
        raise ValueError(f"object has not achieved transport clearance: {clearance:.6f} < {required_clearance:.6f} m")
    if release_half_gap <= abs(start[1, 0] - start[0, 0]) / 2:
        raise ValueError("release_half_gap must open the jaws beyond their current separation")
    lowered = start.copy()
    lowered[:, 2] -= clearance - support_center_clearance
    # Return gently to upright while lowering, so the whole proxy's known
    # lowest point remains meaningful. The path interpolates this rotation.
    lowered[:, 3:] = (0.0, 0.0, 0.0, 1.0)
    # Placing a very small payload must not send fingers below the bench. The
    # soft shape is still free to rotate/deform during lowering.
    if lowered[:, 2].min() + LEDGE_BOTTOM_Z < table_z:
        raise ValueError("observed placement would put a finger shelf through the table")
    opened = lowered.copy()
    center_x = float(lowered[:, 0].mean())
    opened[0, 0], opened[1, 0] = center_x - release_half_gap, center_x + release_half_gap
    withdrawn = opened.copy()
    withdrawn[:, 1] -= 0.04
    retreat = withdrawn.copy()
    retreat[:, 2] += 0.06
    waypoints = (
        ("placement_hold", start, 0.4),
        ("lower_to_support", lowered, 2.0),
        ("support_hold", lowered, 0.6),
        ("release", opened, 0.8),
        ("withdraw", withdrawn, 0.6),
        ("retreat", retreat, 0.6),
        ("settle", retreat, 1.5),
    )
    poses, metadata = _path(start, waypoints, dt, elapsed_before_s)
    metadata.update(
        {
            "observed_initial_clearance_m": clearance,
            "commanded_lowering_m": float(clearance - support_center_clearance),
            "support_check_s": metadata["phases"]["support_hold"]["end_s"],
            "release_window_s": [metadata["phases"]["release"]["start_s"], metadata["phases"]["release"]["end_s"]],
            "settled_window_s": [metadata["phases"]["settle"]["start_s"], metadata["phases"]["settle"]["end_s"]],
        }
    )
    return poses, metadata
