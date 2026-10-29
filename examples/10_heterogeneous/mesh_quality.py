# SPDX-License-Identifier: Apache-2.0
"""Substep-latched quality for a labelled FEM volume, plus contact penetration.

Looking at one frame in fifteen is what hid the old failures: a tet that inverts
for two substeps and recovers between samples leaves no trace in a frame series.
:class:`MeshQuality` is a ``post_solve`` hook, so it sees every substep, and it
reports the worst volume ratio the volume actually reached rather than the one
that happened to survive to the next render.

J is the ratio of a tet's current signed volume to its rest volume, taken from
the source vertices and the source tetrahedra, so it is independent of where the
object was placed. An inversion is ``J <= 0``; a nonfinite J is counted instead
of contributing a minimum, because it has no ordering.
"""

from __future__ import annotations

import numpy as np
import warp as wp


@wp.kernel
def _volumes(
    q: wp.array[wp.vec3],
    tets: wp.array[wp.vec4i],
    labels: wp.array[int],
    rest_det: wp.array[float],
    minimum: wp.array[float],
    inversions: wp.array[int],
    nonfinite: wp.array[int],
):
    t = wp.tid()
    tet = tets[t]
    a = q[tet[0]]
    b = q[tet[1]] - a
    c = q[tet[2]] - a
    d = q[tet[3]] - a
    j = wp.dot(b, wp.cross(c, d)) / rest_det[t]
    part = labels[t]
    if not wp.isfinite(j):
        wp.atomic_add(nonfinite, part, 1)
        return
    wp.atomic_min(minimum, part, j)
    if j <= 0.0:
        wp.atomic_add(inversions, part, 1)


class MeshQuality:
    """Latch the worst volume ratio and the inversion substeps, per part.

    ``read()`` is the window since the last ``clear()``: per part, the smallest J
    reached, and how many tet-substeps were inverted or nonfinite.
    """

    def __init__(self, sim, entity):
        device = sim.model.device
        self.device = device
        self.tets = wp.array(entity.source_tets + entity.particles[0], dtype=wp.vec4i, device=device)
        self.labels = wp.array(entity.tet_part_labels, dtype=int, device=device)
        vertices = entity.source_vertices * entity.spec.scale
        rest = np.linalg.det(vertices[entity.source_tets[:, 1:]] - vertices[entity.source_tets[:, :1]])
        self.rest_det = wp.array(rest, dtype=float, device=device)
        self.parts = len(np.unique(entity.tet_part_labels))
        self.minimum = wp.ones(self.parts, dtype=float, device=device)
        self.inversions = wp.zeros(self.parts, dtype=int, device=device)
        self.nonfinite = wp.zeros(self.parts, dtype=int, device=device)
        sim.pipeline.hooks.add("post_solve", self.sample, name="labelled-volume-quality")
        sim.pipeline.invalidate_graph()

    def sample(self, state, dt):
        del dt
        wp.launch(
            _volumes,
            dim=len(self.tets),
            inputs=[
                state.particle_q,
                self.tets,
                self.labels,
                self.rest_det,
                self.minimum,
                self.inversions,
                self.nonfinite,
            ],
            device=self.device,
        )

    def clear(self):
        self.minimum.fill_(1.0)
        self.inversions.zero_()
        self.nonfinite.zero_()

    def read(self):
        return {
            "minimum_J_by_part": self.minimum.numpy().tolist(),
            "inverted_tet_substeps_by_part": self.inversions.numpy().tolist(),
            "nonfinite_tet_substeps_by_part": self.nonfinite.numpy().tolist(),
        }


def penetration(q, body_q, model, contacts):
    """Recorded-contact penetration, including particle radius and shape margin.

    This checks the contact planes the collision pipeline stored; it cannot
    certify a contact it missed. Host-side, so call it outside a timing window.
    ``body_q`` may be ``None``: a scene with no rigid bodies at all stores
    contacts only against static shapes.
    """
    if body_q is None:
        body_q = np.zeros((0, 7), dtype=np.float32)
    count = min(int(contacts.soft_contact_count.numpy()[0]), contacts.soft_contact_max)
    if not count:
        return np.empty(0)
    indices = contacts.soft_contact_indices.numpy()[:count]
    bary = contacts.soft_contact_barycentric.numpy()[:count]
    weights = bary * (indices >= 0)
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
