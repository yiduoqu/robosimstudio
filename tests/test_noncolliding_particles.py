# SPDX-License-Identifier: Apache-2.0
"""Decorative scene geometry must not contact either rigid or soft objects."""

from __future__ import annotations

import unittest

import newton

import robosimstudio as rss
from robosimstudio.domains.static.entity import EntityStatic

TIER = "fast"


class TestNoncollidingParticles(unittest.TestCase):
    def test_collision_intent_is_independent_of_visibility_and_covers_both_contact_paths(self):
        collision_flags = int(newton.ShapeFlags.COLLIDE_SHAPES | newton.ShapeFlags.COLLIDE_PARTICLES)
        visible_flag = int(newton.ShapeFlags.VISIBLE)
        for collide, visible in ((False, True), (True, True), (True, False), (False, False)):
            with self.subTest(collide=collide, visible=visible):
                builder = newton.ModelBuilder()
                neighbor = builder.add_shape_box(body=-1, hx=0.1, hy=0.1, hz=0.1, label="neighbor")
                neighbor_flags = int(builder.shape_flags[neighbor])
                entity = EntityStatic(
                    rss.slab(name="source_zone", half=(0.06, 0.06, 0.00025), collide=collide, visible=visible)
                )
                entity.build_into(builder, episode=None)
                self.assertEqual(entity.range.shapes, (1, 2))
                flags = int(builder.shape_flags[1])
                self.assertEqual(flags & collision_flags, collision_flags if collide else 0)
                self.assertEqual(bool(flags & visible_flag), visible)
                self.assertEqual(int(builder.shape_flags[neighbor]), neighbor_flags)


if __name__ == "__main__":
    unittest.main()
