from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from lawn.terrain.generator.assets import audit_assets
from lawn.terrain.generator.compute import generate_arrays
from lawn.terrain.generator.config import load_config
from lawn.terrain.generator.truth import export_truth


class Stage3PureTests(unittest.TestCase):
    def _load(self, data, directory=None):
        target = Path(directory or tempfile.mkdtemp()) / "config.json"
        target.write_text(json.dumps(data), encoding="utf-8")
        return load_config(target)

    def _base(self, mode="flat"):
        return {
            "schema_version": 1,
            "scene_name": "stage3_test",
            "global_seed": 99,
            "terrain": {
                "mode": mode,
                "size_m": [12, 10],
                "resolution_m": 0.5,
                "elevation_delta_m": 0.6,
                "slope_deg": 22,
                "max_slope_deg": 8,
                "max_cross_slope_deg": 8,
                "smoothing_passes": 2,
                "spawn_pad": {"center_m": [-3, -2], "radius_m": 1.5},
            },
            "regions": {
                "boundary_buffer_m": 0.5,
                "tall_grass_band_m": 0.5,
                "zones": [
                    {"name": "bed", "kind": "no_mow_zone", "shape": "rectangle", "center_m": [2, 1], "size_m": [2, 3]},
                    {"name": "soil", "kind": "bare_soil_patch", "shape": "circle", "center_m": [-1, 1], "radius_m": 0.8},
                ],
            },
            "assets": [
                {"name": "Grass", "density_per_100m2": 50, "allowed_masks": ["mowable_area"], "forbidden_masks": ["bare_soil_patch"]},
                {"name": "Birch", "density_per_100m2": 20, "allowed_masks": ["no_mow_zone"], "clearance_m": 0.5},
            ],
        }

    def test_all_terrain_modes_are_finite_and_slope_limited(self):
        with tempfile.TemporaryDirectory() as directory:
            heightmap = Path(directory) / "height.npy"
            np.save(heightmap, np.arange(30, dtype=np.float32).reshape(5, 6))
            for mode in ("flat", "single_slope", "rolling_lawn", "terraced_lawn", "heightmap"):
                data = self._base(mode)
                if mode == "heightmap":
                    data["terrain"]["heightmap_path"] = str(heightmap)
                    data["terrain"]["heightmap_scale_m"] = 1.0
                arrays = generate_arrays(self._load(data, directory), {"Grass", "Birch"})
                self.assertTrue(np.isfinite(arrays.heights).all(), mode)
                self.assertLessEqual(float(arrays.slope_deg.max()), 8.01, mode)

    def test_masks_spawn_pad_and_region_constrained_placements(self):
        config = self._load(self._base("rolling_lawn"))
        arrays = generate_arrays(config, {"Grass", "Birch"})
        self.assertEqual(set(arrays.masks), {"mowable_area", "no_mow_zone", "boundary_buffer", "tall_grass_band", "bare_soil_patch", "spawn_pad"})
        self.assertFalse(np.any(arrays.masks["mowable_area"] & arrays.masks["no_mow_zone"]))
        self.assertGreater(int(arrays.masks["spawn_pad"].sum()), 0)
        self.assertLess(float(np.ptp(arrays.heights[arrays.masks["spawn_pad"]])), 1e-6)
        for name, required, forbidden in (("Grass", "mowable_area", "bare_soil_patch"), ("Birch", "no_mow_zone", None)):
            positions = arrays.placements[name][0]
            sx, sy = config.terrain.size_m
            ix = np.rint((positions[:, 0] + sx / 2) / sx * (arrays.heights.shape[0] - 1)).astype(int)
            iy = np.rint((positions[:, 1] + sy / 2) / sy * (arrays.heights.shape[1] - 1)).astype(int)
            self.assertTrue(arrays.masks[required][ix, iy].all())
            if forbidden:
                self.assertFalse(arrays.masks[forbidden][ix, iy].any())

    def test_truth_bundle_contains_navigation_contract(self):
        config = self._load(self._base("single_slope"))
        arrays = generate_arrays(config, {"Grass", "Birch"})
        with tempfile.TemporaryDirectory() as directory:
            export_truth(directory, config, arrays, audit_assets(directory))
            expected = {"elevation.npy", "slope_deg.npy", "region_masks.npz", "occupancy.pgm", "occupancy.yaml", "work_areas.geojson", "semantics.json", "manifest.json"}
            self.assertEqual(expected, {path.name for path in Path(directory).iterdir()})
            manifest = json.loads((Path(directory) / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["seed"], 99)
            self.assertEqual(manifest["terrain_mode"], "single_slope")
            self.assertEqual(len(manifest["height_sha256"]), 64)

    def test_external_asset_manifest_is_config_driven(self):
        with tempfile.TemporaryDirectory() as directory:
            asset = Path(directory) / "bench.usda"
            asset.write_text("#usda 1.0", encoding="utf-8")
            data = self._base()
            data["asset_manifest"] = [{"name": "Bench", "path": "bench.usda", "kind": "object", "base_scale": 0.01}]
            config = self._load(data, directory)
            records = audit_assets(directory, config.asset_manifest)
            self.assertTrue(records["Bench"].available)
            self.assertEqual(records["Bench"].base_scale, 0.01)

    def test_offroad_visual_config_and_surface_alignment(self):
        data = self._base("single_slope")
        data["terrain"]["slope_deg"] = 6
        data["assets"][0]["align_to_surface"] = True
        data["surface"] = {
            "diffuse_texture": "textures/forest.tif",
            "normal_texture": "textures/forest_normal.tif",
            "texture_scale": 0.25,
        }
        data["lighting"] = {
            "enabled": True,
            "hdri_path": "textures/forest.hdr",
            "dome_intensity": 800,
            "sun_rotation_deg": [35, -25, 20],
        }
        config = self._load(data)
        arrays = generate_arrays(config, {"Grass", "Birch"})
        orientations = arrays.placements["Grass"][1]
        self.assertTrue(config.lighting.enabled)
        self.assertEqual(config.surface.texture_scale, 0.25)
        self.assertGreater(float(np.max(np.abs(orientations[:, 1:3]))), 0.001)
        np.testing.assert_allclose(np.linalg.norm(orientations, axis=1), 1.0, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
