# SPDX-License-Identifier: Apache-2.0
"""Source estimates and explicit material overrides stay distinct in a labelled FEM build."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import newton
import numpy as np

from robosimstudio_materials.heterogeneous import EntityHeterogeneousSoft, HeterogeneousSoftSpec

TIER = "fast"


class TestHeterogeneousMaterials(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        mesh = root / "object.mesh"
        mesh.write_text(
            "MeshVersionFormatted 1\nDimension 3\nVertices\n5\n"
            "0 0 0 0\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 -1 0\n"
            "Tetrahedra\n2\n1 2 3 4 0\n1 3 2 5 0\nEnd\n"
        )
        params = root / "params.npz"
        self.elastic = np.array([[5e5, 0.45], [5e6, 0.49]])
        np.savez(params, tet_part_labels=[0, 2], tet_E_nu=self.elastic, tet_density=[300, 1100])
        materials = root / "materials.json"
        materials.write_text(
            json.dumps(
                {
                    "predictions": [
                        {
                            "part_index": label,
                            "youngs_modulus_pa": young,
                            "poisson_ratio": nu,
                            "density_kg_m3": density,
                            "part_color_rgb": [100, 150, 200],
                        }
                        for label, (young, nu), density in zip((0, 2), self.elastic, (300, 1100), strict=True)
                    ]
                }
            )
        )
        self.paths = {"mesh_path": str(mesh), "params_path": str(params), "materials_path": str(materials)}

    def build(self, **kwargs):
        entity = EntityHeterogeneousSoft(HeterogeneousSoftSpec(**self.paths, **kwargs))
        builder = newton.ModelBuilder()
        entity.build(builder, None)
        return entity, builder

    def test_source_defaults_and_overrides_reach_the_builder_without_changing_source_or_mass(self):
        source_bytes = {name: Path(path).read_bytes() for name, path in self.paths.items()}
        original, baseline = self.build()
        changed, builder = self.build(material_overrides={2: {"youngs_modulus_pa": 1e7, "poisson_ratio": 0.45}})
        np.testing.assert_array_equal(original.effective_elastic, self.elastic)
        np.testing.assert_array_equal(original.effective_damping, [250, 2500])
        np.testing.assert_array_equal(changed.source_elastic, self.elastic)
        np.testing.assert_array_equal(changed.effective_elastic, [[5e5, 0.45], [1e7, 0.45]])
        np.testing.assert_array_equal(changed.effective_damping, original.effective_damping)
        np.testing.assert_array_equal(builder.particle_mass, baseline.particle_mass)
        np.testing.assert_array_equal(builder.tet_indices, baseline.tet_indices)
        np.testing.assert_array_equal(changed.tet_part_labels, original.tet_part_labels)
        self.assertEqual(changed.part_materials, original.part_materials)
        young, nu = changed.effective_elastic.T
        np.testing.assert_allclose(
            np.asarray(builder.tet_materials),
            np.column_stack((young / (2 * (1 + nu)), young * nu / ((1 + nu) * (1 - 2 * nu)), [250, 2500])),
        )
        damped, damped_builder = self.build(material_overrides={2: {"damping_pa_s": 0.0}})
        np.testing.assert_array_equal(damped.effective_elastic, self.elastic)
        np.testing.assert_array_equal(damped.effective_damping, [250, 0])
        self.assertEqual(damped_builder.tet_materials[1][2], 0)
        for name, path in self.paths.items():
            self.assertEqual(Path(path).read_bytes(), source_bytes[name])

    def test_invalid_override_fails_before_adding_any_elements(self):
        invalid = (
            {2: {"youngs_modulus_pa": 0}},
            {2: {"youngs_modulus_pa": float("inf")}},
            {2: {"poisson_ratio": 0.5}},
            {2: {"poisson_ratio": -0.1}},
            {2: {"damping_pa_s": -1}},
            {2: {"damping_pa_s": float("nan")}},
            {2: {"density_kg_m3": 500}},
            {9: {"youngs_modulus_pa": 1e6}},
            {"2": {"youngs_modulus_pa": 1e6}},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides):
                builder = newton.ModelBuilder()
                with self.assertRaises(ValueError):
                    spec = HeterogeneousSoftSpec(**self.paths, material_overrides=overrides)
                    EntityHeterogeneousSoft(spec).build(builder, None)
                self.assertEqual(builder.particle_count, 0)
                self.assertEqual(builder.tet_count, 0)
