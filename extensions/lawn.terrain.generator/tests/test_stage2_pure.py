from __future__ import annotations

import json
import tempfile
import unittest

import numpy as np

from lawn.terrain.generator.assets import audit_assets
from lawn.terrain.generator.compute import generate_arrays
from lawn.terrain.generator.config import load_config


class Stage2PureTests(unittest.TestCase):
    def _config(self, elevation_delta_m=0):
        data = {
            "schema_version": 1,
            "seed": 42,
            "terrain": {"size_m": [6, 4], "resolution_m": 1, "elevation_delta_m": elevation_delta_m},
            "assets": [{"name": "Grass", "density_per_100m2": 100}],
        }
        temp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        with temp:
            json.dump(data, temp)
        return load_config(temp.name)

    def test_fixed_seed_is_deterministic(self):
        config = self._config()
        first = generate_arrays(config, {"Grass"})
        second = generate_arrays(config, {"Grass"})
        np.testing.assert_array_equal(first.heights, second.heights)
        for a, b in zip(first.placements["Grass"], second.placements["Grass"]):
            np.testing.assert_array_equal(a, b)
        self.assertEqual(len(first.placements["Grass"][0]), 24)

    def test_missing_assets_are_disabled(self):
        with tempfile.TemporaryDirectory() as directory:
            records = audit_assets(directory)
        self.assertFalse(records["Switchgrass"].available)
        self.assertFalse(records["Container"].available)

    def test_mesh_indices_are_valid(self):
        generated = generate_arrays(self._config(), {"Grass"})
        self.assertTrue(np.isfinite(generated.vertices).all())
        self.assertGreaterEqual(generated.face_indices.min(), 0)
        self.assertLess(generated.face_indices.max(), len(generated.vertices))

    def test_nonflat_heightfield_is_deterministic_without_optional_packages(self):
        config = self._config(elevation_delta_m=1.0)
        first = generate_arrays(config, {"Grass"})
        second = generate_arrays(config, {"Grass"})
        np.testing.assert_array_equal(first.heights, second.heights)
        self.assertGreater(float(np.ptp(first.heights)), 0.9)


if __name__ == "__main__":
    unittest.main()
