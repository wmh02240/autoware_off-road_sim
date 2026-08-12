from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADLESS_SCRIPT = ROOT.parents[1] / "scripts/tools/generate_lawn_scene.py"


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
        self.assertNotIn("prototype.CreateVisibilityAttr", source)

    def test_point_instancer_prototype_owns_source_unit_scale(self):
        source = (ROOT / "lawn/terrain/generator/usd_writer.py").read_text(encoding="utf-8")
        self.assertIn('UsdGeom.XformOp.PrecisionFloat, "sourceUnits"', source)
        self.assertIn("Gf.Vec3f(record.base_scale, record.base_scale, record.base_scale)", source)
        self.assertIn("for s in scales", source)
        self.assertNotIn("prototype.SetInstanceable(True)", source)


    def test_nested_instancer_trees_use_direct_references(self):
        source = (ROOT / "lawn/terrain/generator/usd_writer.py").read_text(encoding="utf-8")
        self.assertIn('if record.kind == "tree":', source)
        self.assertIn("self._add_tree_references", source)
        self.assertIn('f"{tree_parent}/Instance_{index:04d}"', source)
        self.assertIn("CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)", source)

    def test_ui_resolves_registration_symlink_for_default_config(self):
        source = (ROOT / "lawn/terrain/generator/ui.py").read_text(encoding="utf-8")
        self.assertIn("Path(self._extension_path).resolve()", source)

    def test_ui_exposes_interactive_stage3_editor(self):
        source = (ROOT / "lawn/terrain/generator/ui.py").read_text(encoding="utf-8")
        for label in (
            "Load Config",
            "Save Config",
            "Generate All",
            "Clear All",
            "General Settings",
            "Terrain",
            "Regions and Work Areas",
            "Trees, Rocks and Ground Vegetation",
            "Ground Surface",
            "Environment Lighting",
            "Output and Navigation Truth",
        ):
            self.assertIn(label, source)
        self.assertIn("ui.ScrollingFrame", source)
        self.assertIn("ui.Separator(height=1)", source)
        self.assertIn("ui.VStack(height=92", source)
        self.assertIn("_runtime_config", source)
        self.assertIn("export_truth", source)

    def test_ui_documents_units_and_deterministic_seed(self):
        source = (ROOT / "lawn/terrain/generator/ui.py").read_text(encoding="utf-8")
        for label in (
            "Random Seed (deterministic)",
            "Density (instances/100 m²)",
            "Min Scale (multiplier)",
            "Max Scale (multiplier)",
            "Boundary/Object Clearance (m)",
            "Bare Soil Fraction (0–1)",
            "Generated with seed=",
        ):
            self.assertIn(label, source)

    def test_headless_lifecycle_validation_is_opt_in(self):
        source = HEADLESS_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('"--validate-lifecycle"', source)
        self.assertIn("if args.validate_lifecycle:", source)
        self.assertIn("lifecycle_validation=SKIPPED", source)
        self.assertNotIn("for _ in range(3):", source)
        self.assertIn("skip_cleanup=True", source)


if __name__ == "__main__":
    unittest.main()
