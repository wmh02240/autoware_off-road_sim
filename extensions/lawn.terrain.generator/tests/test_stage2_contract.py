from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class Stage2ContractTests(unittest.TestCase):
    def test_manifest_and_module_name_match(self):
        manifest = (ROOT / "config/extension.toml").read_text(encoding="utf-8")
        self.assertIn('name = "lawn.terrain.generator"', manifest)
        self.assertIn('"omni.physx" = {}', manifest)
        self.assertIn('"omni.usd" = {}', manifest)
        self.assertNotIn("test_hello_world", "\n".join(str(p) for p in ROOT.rglob("*")))

    def test_generator_never_creates_a_physics_scene(self):
        sources = "\n".join(path.read_text(encoding="utf-8") for path in (ROOT / "lawn/terrain/generator").glob("*.py"))
        self.assertNotIn("UsdPhysics.Scene.Define", sources)
        self.assertNotIn('DefinePrim(Sdf.Path("/World/physicsScene")', sources)

    def test_worker_compute_module_has_no_usd_import(self):
        source = (ROOT / "lawn/terrain/generator/compute.py").read_text(encoding="utf-8")
        self.assertNotIn("from pxr", source)
        self.assertNotIn("import omni", source)
        self.assertNotIn("perlin_noise", source)

    def test_point_instancer_uses_relationship_prototype(self):
        source = (ROOT / "lawn/terrain/generator/usd_writer.py").read_text(encoding="utf-8")
        self.assertIn("GetPrototypesRel().SetTargets", source)
        self.assertNotIn("CreateVisibilityAttr", source)

    def test_ui_resolves_registration_symlink_for_default_config(self):
        source = (ROOT / "lawn/terrain/generator/ui.py").read_text(encoding="utf-8")
        self.assertIn("Path(self._extension_path).resolve()", source)


if __name__ == "__main__":
    unittest.main()
