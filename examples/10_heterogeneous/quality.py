# SPDX-License-Identifier: Apache-2.0
"""Small, optional device diagnostics for the labelled FEM examples."""

from __future__ import annotations

import numpy as np
import warp as wp


@wp.kernel
def _volumes(
    q: wp.array[wp.vec3],
    tets: wp.array[wp.vec4i],
    labels: wp.array[int],
    rest_det: wp.array[float],
    particle_stride: int,
    minimum: wp.array2d[float],
    inversions: wp.array2d[int],
    nonfinite: wp.array2d[int],
):
    world, t = wp.tid()
    tet = tets[t]
    offset = world * particle_stride
    a = q[tet[0] + offset]
    b = q[tet[1] + offset] - a
    c = q[tet[2] + offset] - a
    d = q[tet[3] + offset] - a
    j = wp.dot(b, wp.cross(c, d)) / rest_det[t]
    part = labels[t]
    if not wp.isfinite(j):
        wp.atomic_add(nonfinite, world, part, 1)
        return
    wp.atomic_min(minimum, world, part, j)
    if j <= 0.0:
        wp.atomic_add(inversions, world, part, 1)


class MeshQuality:
    """Latch worst volume and inversion-substep occurrences, per world and part."""

    def __init__(self, sim, entity):
        self.sim = sim
        self.entity = entity
        self.worlds = sim.num_worlds
        device = sim.model.device
        self.tets = wp.array(entity.source_tets + entity.particles[0], dtype=wp.vec4i, device=device)
        self.labels = wp.array(entity.tet_part_labels, dtype=int, device=device)
        verts = entity.source_vertices * entity.spec.scale
        det = np.linalg.det(verts[entity.source_tets[:, 1:]] - verts[entity.source_tets[:, :1]])
        self.det = wp.array(det, dtype=float, device=device)
        self.stride = sim.scene.worlds.particles_per_world if sim.num_worlds > 1 else 0
        self.minimum = wp.ones((self.worlds, 3), dtype=float, device=device)
        self.inversions = wp.zeros((self.worlds, 3), dtype=int, device=device)
        self.nonfinite = wp.zeros((self.worlds, 3), dtype=int, device=device)
        sim.pipeline.hooks.add("post_solve", self.sample, name="labelled-volume-quality")
        sim.pipeline.invalidate_graph()

    def sample(self, state, dt):
        del dt
        wp.launch(
            _volumes,
            dim=(self.worlds, len(self.entity.source_tets)),
            inputs=[
                state.particle_q,
                self.tets,
                self.labels,
                self.det,
                self.stride,
                self.minimum,
                self.inversions,
                self.nonfinite,
            ],
        )

    def clear(self):
        self.minimum.fill_(1.0)
        self.inversions.zero_()
        self.nonfinite.zero_()

    def read(self):
        return {
            "minimum_J_by_world_part": self.minimum.numpy().tolist(),
            "inverted_tet_substeps_by_world_part": self.inversions.numpy().tolist(),
            "nonfinite_tet_substeps_by_world_part": self.nonfinite.numpy().tolist(),
        }


def stretches(q, entity):
    """Frame-only principal stretches, used in diagnostic runs, outside timing."""
    tets = entity.source_tets
    rest = entity.source_vertices * entity.spec.scale
    # Edge vectors as rows: Dm^-1 Ds has the same singular values as F transpose.
    dm = rest[tets[:, 1:]] - rest[tets[:, :1]]
    ds = q[tets[:, 1:]] - q[tets[:, :1]]
    singular = np.linalg.svd(np.linalg.solve(dm, ds), compute_uv=False)
    return {
        str(part): {
            "p95": float(np.quantile(singular[entity.tet_part_labels == part, 0], 0.95)),
            "max": float(singular[entity.tet_part_labels == part, 0].max()),
        }
        for part in np.unique(entity.tet_part_labels)
    }


def contact_depths(q, body_q, model, contacts):
    """Recorded-contact penetration, including particle radius and shape margin.

    This checks the current contact planes; it cannot certify missing contacts.
    Host-side diagnostics only, outside rollout throughput measurements.
    """
    count = min(int(contacts.soft_contact_count.numpy()[0]), contacts.soft_contact_max)
    if not count:
        return np.empty(0)
    indices = contacts.soft_contact_indices.numpy()[:count]
    bary = contacts.soft_contact_barycentric.numpy()[:count]
    valid = indices >= 0
    weights = bary * valid
    ids = np.maximum(indices, 0)
    point = (q[ids] * weights[..., None]).sum(axis=1)
    radius = (model.particle_radius.numpy()[ids] * weights).sum(axis=1)
    shapes = contacts.soft_contact_shape.numpy()[:count]
    bodies = model.shape_body.numpy()[shapes]
    surface = contacts.soft_contact_body_pos.numpy()[:count].copy()
    moving = bodies >= 0
    poses = body_q[bodies[moving]]
    # Quaternion rotation of body-local contact points, in Newton's xyzw order.
    xyz, scalar = poses[:, 3:6], poses[:, 6:7]
    twice_cross = 2 * np.cross(xyz, surface[moving])
    surface[moving] += scalar * twice_cross + np.cross(xyz, twice_cross) + poses[:, :3]
    normals = contacts.soft_contact_normal.numpy()[:count]
    distance = ((point - surface) * normals).sum(axis=1)
    return np.maximum(radius + model.shape_margin.numpy()[shapes] - distance, 0)


@wp.kernel
def _contact_plane_depth(
    q: wp.array[wp.vec3],
    body_q: wp.array[wp.transform],
    radius: wp.array[float],
    shape_body: wp.array[int],
    margin: wp.array[float],
    count: wp.array[int],
    capacity: int,
    indices: wp.array[wp.vec3i],
    bary: wp.array[wp.vec3],
    shapes: wp.array[int],
    body_pos: wp.array[wp.vec3],
    normal: wp.array[wp.vec3],
    worst: wp.array[float],
    invalid: wp.array[int],
):
    c = wp.tid()
    if count[0] > capacity:
        wp.atomic_max(invalid, 0, 1)
    if c >= wp.min(count[0], capacity):
        return
    point = wp.vec3(0.0)
    r = float(0.0)
    for k in range(3):
        p = indices[c][k]
        if p >= 0:
            point += bary[c][k] * q[p]
            r += bary[c][k] * radius[p]
    shape = shapes[c]
    surface = body_pos[c]
    b = shape_body[shape]
    if b >= 0:
        surface = wp.transform_point(body_q[b], surface)
    depth = r + margin[shape] - wp.dot(point - surface, normal[c])
    if wp.isfinite(depth):
        wp.atomic_max(worst, 0, depth)
    else:
        wp.atomic_max(invalid, 0, 1)


@wp.kernel
def _box_table_depth(
    body_q: wp.array[wp.transform],
    shapes: wp.array[int],
    shape_body: wp.array[int],
    shape_transform: wp.array[wp.transform],
    shape_scale: wp.array[wp.vec3],
    table_z: float,
    worst: wp.array[float],
    invalid: wp.array[int],
):
    shape = shapes[wp.tid()]
    pose = wp.transform_multiply(body_q[shape_body[shape]], shape_transform[shape])
    half = shape_scale[shape]
    for k in range(8):
        corner = wp.vec3(
            half[0] * float(2 * (k % 2) - 1),
            half[1] * float(2 * ((k // 2) % 2) - 1),
            half[2] * float(2 * ((k // 4) % 2) - 1),
        )
        z = wp.transform_point(pose, corner)[2]
        if wp.isfinite(z):
            wp.atomic_max(worst, 0, wp.max(0.0, table_z - z))
        else:
            wp.atomic_max(invalid, 0, 1)


class ContactQuality:
    """Latch recorded-plane and gripper-box/table depths every physics substep.

    Plane measurements cannot detect missing contact records. The analytic
    box/table check is independent of rigid contact detection. This diagnostic
    covers the example's single-world floating gripper, outside benchmarks.
    """

    def __init__(self, sim, recording, gripper, table_z):
        import newton

        if sim.num_worlds != 1:
            raise ValueError("ContactQuality is a single-world example diagnostic")
        self.sim, self.recording, self.table_z = sim, recording, float(table_z)
        shapes = [*gripper.finger_shapes]
        if hasattr(gripper, "carrier_shape"):
            shapes.append(gripper.carrier_shape)
        kinds = sim.model.shape_type.numpy()[shapes]
        if np.any(kinds != int(newton.GeoType.BOX)):
            raise ValueError("analytic table diagnostic requires box gripper shapes")
        self.shapes = wp.array(shapes, dtype=int, device=sim.model.device)
        self.plane_depth = wp.zeros(1, dtype=float, device=sim.model.device)
        self.box_depth = wp.zeros(1, dtype=float, device=sim.model.device)
        self.invalid = wp.zeros(1, dtype=int, device=sim.model.device)
        sim.pipeline.hooks.add("post_solve", self.sample, name="labelled-contact-quality")
        sim.pipeline.invalidate_graph()

    def sample(self, state, dt):
        del dt
        r, m = self.recording, self.recording.model
        c = r.contacts
        # Coupled contacts use entry-local particle/body indices. The public
        # current entry state is the soft output at this post-solve phase.
        local = state if r.island is None else r.state()
        wp.launch(
            _contact_plane_depth,
            dim=c.soft_contact_max,
            inputs=[
                local.particle_q,
                local.body_q,
                m.particle_radius,
                m.shape_body,
                m.shape_margin,
                c.soft_contact_count,
                c.soft_contact_max,
                c.soft_contact_indices,
                c.soft_contact_barycentric,
                c.soft_contact_shape,
                c.soft_contact_body_pos,
                c.soft_contact_normal,
                self.plane_depth,
                self.invalid,
            ],
            device=m.device,
        )
        m = self.sim.model
        wp.launch(
            _box_table_depth,
            dim=len(self.shapes),
            inputs=[
                state.body_q,
                self.shapes,
                m.shape_body,
                m.shape_transform,
                m.shape_scale,
                self.table_z,
                self.box_depth,
                self.invalid,
            ],
            device=m.device,
        )

    def clear(self):
        self.plane_depth.zero_()
        self.box_depth.zero_()
        self.invalid.zero_()

    def read(self):
        return {
            "maximum_recorded_plane_penetration_m": float(self.plane_depth.numpy()[0]),
            "maximum_gripper_box_table_penetration_m": float(self.box_depth.numpy()[0]),
            "invalid_or_overflow": bool(self.invalid.numpy()[0]),
        }


def finger_table_depth(poses, spec, table_z):
    """Analytic proxy-box/table penetration for the floating fingers."""
    x, y, z, w = np.asarray(poses)[:, 3:].T
    row_z = np.column_stack((2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    bottom = poses[:, 2] - np.abs(row_z) @ [0.003, spec.pad_half_width, 0.006]
    if spec.ledge:
        shelf_bottom = poses[:, 2] - 0.0068 * row_z[:, 2] - np.abs(row_z) @ [0.006, spec.pad_half_width, 0.0012]
        bottom = np.minimum(bottom, shelf_bottom)
    return float(max(0, table_z - bottom.min()))
