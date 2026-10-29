# SPDX-License-Identifier: Apache-2.0
"""The loop's own geometry, measured from the mesh, and the path's easing curve.

Both scenes grip this object by its loop, and both need the same three numbers
about it -- where the hole's centre is, how wide the loop is where the pads close
on it, and how far its base is below that -- so they are measured here once
instead of twice, and the scenes state only what they do with them.

Nothing here reads the parts' materials or builds anything: this is measurement
of the source mesh, in the mesh's own frame.
"""

from __future__ import annotations

import numpy as np

__all__ = ["ring_geometry", "smooth"]


def smooth(time: float, start: float, end: float) -> float:
    """Smoothstep from 0 to 1 over ``[start, end]``, and clamped outside it."""
    t = float(np.clip((time - start) / (end - start), 0.0, 1.0))
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def ring_geometry(vertices: np.ndarray, tets: np.ndarray, labels: np.ndarray) -> dict:
    """The loop's hole, its width in the closing direction, and the toy's extents.

    The hole is not a circle, so its centre is found the way one measures a
    hole: walk the loop's vertices in angular sectors around a tentative centre
    and keep each sector's nearest point -- the inner surface where the hole is
    widest inward -- then recentre on those points until they stop moving. What
    remains is an inner radius per sector; the smallest is what the rod has to
    clear and what the toy seats on. The same plane gives the loop's outer reach
    in x about that centre, which is where the pads close.
    """
    loop = np.unique(tets[labels == 2])
    plane = vertices[loop][:, :2]
    centre = plane.mean(axis=0)
    sectors = 72
    for _ in range(30):
        angle = np.arctan2(plane[:, 1] - centre[1], plane[:, 0] - centre[0])
        radius = np.linalg.norm(plane - centre, axis=1)
        bins = np.floor((angle + np.pi) / (2.0 * np.pi) * sectors).astype(int) % sectors
        inner = np.array([plane[bins == b][np.argmin(radius[bins == b])] for b in range(sectors) if np.any(bins == b)])
        centre = inner.mean(axis=0)
    angle = np.arctan2(plane[:, 1] - centre[1], plane[:, 0] - centre[0])
    radius = np.linalg.norm(plane - centre, axis=1)
    bins = np.floor((angle + np.pi) / (2.0 * np.pi) * sectors).astype(int) % sectors
    inner_radius = np.array([radius[bins == b].min() for b in range(sectors) if np.any(bins == b)])
    widest_y, reach_y, reach_x = _outer_profile(vertices[loop][:, :2] - centre, float(centre[1]))
    return {
        "widest_local_y": widest_y,
        "widest_reach": float(reach_x.max()),
        "reach_profile": (reach_y, reach_x),
        "loop_ids": loop,
        # The ring is symmetric through its own thickness: its mean z is the
        # plane the rod has to meet, and the hole centre's third coordinate.
        "hole_centre_local": np.array([centre[0], centre[1], float(vertices[loop, 2].mean())]),
        "hole_inradius": float(inner_radius.min()),
        "hole_inradius_median": float(np.median(inner_radius)),
        "side_reach": float(np.max(np.abs(plane[:, 0] - centre[0]))),
        "base_local_y": float(vertices[:, 1].min()),
        "loop_half_thickness": float(np.abs(vertices[loop, 2]).max()),
    }


def _outer_profile(offset: np.ndarray, y_origin: float, step: float = 0.002) -> tuple:
    """The loop's outward reach in x against height, slice by slice.

    The pads close along x, so what they meet is the loop's outer surface in x,
    and how wide that is depends on the height they meet it at. The profile is
    what says where a flat pad grips well: at the widest band the surface is
    vertical and the grip is a friction pinch on a convex cord, which creeps out;
    a few millimetres above it the surface leans back, so a downward slip brings
    wider material into the faces and the loop's own weight wedges it in.

    Args:
        offset: The loop's vertices in the loop's own plane, about the hole centre.
        y_origin: The hole centre's y in the mesh's frame, to state slices in.
        step: Slice height [m].

    Returns:
        ``(widest_y, slice_y, reach)``, all in the mesh's frame: the widest band's
        local y, and the slice centres and their reaches, for a scene that wants a
        reach at some other height.
    """
    low = float(offset[:, 1].min())
    count = int(np.ceil((float(offset[:, 1].max()) - low) / step)) + 1
    index = np.clip(((offset[:, 1] - low) / step).astype(int), 0, count - 1)
    slice_y = np.array([low + (b + 0.5) * step for b in range(count)])
    reach = np.zeros(count)
    for b in range(count):
        in_slice = index == b
        if np.any(in_slice):
            reach[b] = np.abs(offset[in_slice, 0]).max()
    return float(y_origin + slice_y[int(np.argmax(reach))]), slice_y + float(y_origin), reach
