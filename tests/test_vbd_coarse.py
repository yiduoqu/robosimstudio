# SPDX-License-Identifier: Apache-2.0
"""The deterministic coarse correction must affect the current physical iterate."""

from __future__ import annotations

import unittest

import numpy as np
import warp as wp

TIER = "medium"


class TestCoarseTranslation(unittest.TestCase):
    def test_rotation_preserves_shape_history_and_other_world(self):
        if not wp.is_cuda_available():
            self.skipTest("deterministic atomic lowering requires CUDA")
        from robosimstudio_materials.vbd_coarse import _apply_object_se3

        options = wp.get_module_options(_apply_object_se3.module).copy()
        self.addCleanup(wp.set_module_options, options, module=_apply_object_se3.module)
        wp.set_module_options(
            {"deterministic": wp.DeterministicMode.RUN_TO_RUN, "deterministic_max_records": 0},
            module=_apply_object_se3.module,
        )
        device = "cuda:0"
        tetra = np.array([[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]], dtype=np.float32)
        current = np.concatenate((tetra + np.array([0.4, 0.2, 0.8]), tetra + np.array([0.7, 0.2, 0.8]))).astype(
            np.float32
        )
        reference = current - np.float32(0.001)
        center = current.reshape(2, 4, 3).mean(axis=1)
        angle = 0.001
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        translation = np.array([0.00001, -0.00002, 0.00003], dtype=np.float32)
        expected = current.copy()
        expected[:4] = (current[:4] - center[0]) @ rotation.T + center[0] + translation
        q = wp.array(current, dtype=wp.vec3, device=device)
        displacement = wp.array(current - reference, dtype=wp.vec3, device=device)
        wp.launch(
            _apply_object_se3,
            dim=8,
            inputs=[
                wp.array([0, 0, 0, 0, 1, 1, 1, 1], dtype=int, device=device),
                wp.array(center, dtype=wp.vec3, device=device),
                wp.array([translation, np.zeros(3)], dtype=wp.vec3, device=device),
                wp.array([[0, 0, angle], [0, 0, 0]], dtype=wp.vec3, device=device),
                wp.array(reference, dtype=wp.vec3, device=device),
                q,
                displacement,
            ],
            device=device,
        )
        actual = q.numpy()
        np.testing.assert_allclose(actual[:4], expected[:4], rtol=0, atol=1.2e-7)
        np.testing.assert_array_equal(actual[4:], current[4:])
        np.testing.assert_array_equal(actual, reference + displacement.numpy())
        before_edges = np.linalg.norm(current[:4, None] - current[None, :4], axis=2)
        after_edges = np.linalg.norm(actual[:4, None] - actual[None, :4], axis=2)
        np.testing.assert_allclose(after_edges, before_edges, rtol=0, atol=1.2e-7)

    def test_deterministic_application_moves_positions_and_history_together(self):
        if not wp.is_cuda_available():
            self.skipTest("deterministic atomic lowering requires CUDA")
        from robosimstudio_materials.vbd_coarse import _apply_block

        options = wp.get_module_options(_apply_block.module).copy()
        self.addCleanup(wp.set_module_options, options, module=_apply_block.module)
        wp.set_module_options(
            {"deterministic": wp.DeterministicMode.RUN_TO_RUN, "deterministic_max_records": 0},
            module=_apply_block.module,
        )
        device = "cuda:0"
        reference = np.array([[0.4, 0.2, 0.8], [0.41, 0.2, 0.8], [0.4, 0.21, 0.8]], dtype=np.float32)
        delta = np.array([[0.001, -0.002, 0.003]], dtype=np.float32)
        q = wp.array(reference, dtype=wp.vec3, device=device)
        displacement = wp.zeros_like(q)
        wp.launch(
            _apply_block,
            dim=len(reference),
            inputs=[
                wp.zeros(len(reference), dtype=int, device=device),
                wp.array(delta, dtype=wp.vec3, device=device),
                wp.array(reference, dtype=wp.vec3, device=device),
                q,
                displacement,
            ],
            device=device,
        )
        np.testing.assert_array_equal(q.numpy(), reference + delta)
        np.testing.assert_array_equal(displacement.numpy(), np.repeat(delta, len(reference), axis=0))


if __name__ == "__main__":
    unittest.main()
