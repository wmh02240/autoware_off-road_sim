#!/usr/bin/env python3
"""Generate a lawn USD from config with Isaac Sim and no UI dependency."""

from __future__ import annotations

import argparse
import hashlib
import sys
import traceback
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--truth-output", default="", help="Override the config truth_directory")
    parser.add_argument(
        "--validate-lifecycle",
        action="store_true",
        help="Also smoke-test the UI and extension disable/re-enable lifecycle",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.cycles < 1:
        raise ValueError("--cycles must be at least 1")
    project_root = Path(__file__).resolve().parents[2]
    extension_root = project_root / "extensions/lawn.terrain.generator"
    sys.path.insert(0, str(extension_root))

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import omni.kit.app
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, UsdUtils

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.add_path(str(project_root / "extensions"))
        if not manager.set_extension_enabled_immediate("lawn.terrain.generator", True):
            raise RuntimeError("Extension Manager could not enable lawn.terrain.generator")
        app.update()

        from lawn.terrain.generator.assets import discover_asset_root
        from lawn.terrain.generator.config import load_config
        from lawn.terrain.generator.controller import GeneratorController

        context = omni.usd.get_context()
        context.new_stage()
        app.update()
        stage = context.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        config = load_config(args.config)
        asset_root = discover_asset_root(str(extension_root), config.asset_root)
        controller = GeneratorController(stage, str(asset_root))
        print(f"asset_root={asset_root}", flush=True)
        print(f"missing_assets={','.join(controller.missing_assets)}", flush=True)

        digest = None
        expected_counts = None
        for cycle in range(args.cycles):
            print(f"cycle={cycle + 1}/{args.cycles} step=compute", flush=True)
            arrays = controller.compute(config)
            current_digest = hashlib.sha256(arrays.heights.tobytes()).hexdigest()
            print(f"cycle={cycle + 1}/{args.cycles} step=usd_apply", flush=True)
            counts = controller.apply(config, arrays)
            print(f"cycle={cycle + 1}/{args.cycles} step=usd_apply_done", flush=True)
            if digest is not None and current_digest != digest:
                raise AssertionError("fixed seed produced a different heightfield")
            if expected_counts is not None and counts != expected_counts:
                raise AssertionError("fixed seed produced different object counts")
            digest, expected_counts = current_digest, counts
            if not stage.GetPrimAtPath(config.parent_path).IsValid():
                raise AssertionError("generated root prim is missing")
            if cycle + 1 < args.cycles:
                controller.clear(config.parent_path)
                if stage.GetPrimAtPath(config.parent_path).IsValid():
                    raise AssertionError("clear left the generated root prim behind")

        physics_scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if physics_scenes:
            raise AssertionError(f"generator authored PhysicsScene prims: {physics_scenes}")
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        print("step=usd_export", flush=True)
        if not stage.GetRootLayer().Export(str(output)):
            raise RuntimeError(f"failed to export {output}")
        print("step=dependency_scan", flush=True)
        _, _, unresolved = UsdUtils.ComputeAllDependencies(str(output))
        unexpected = [str(item) for item in unresolved if Path(str(item)).name != "OmniPBR.mdl"]
        if unexpected:
            raise AssertionError(f"unresolved USD dependencies: {unexpected}")
        print(f"height_sha256={digest}", flush=True)
        print(f"counts={expected_counts}", flush=True)
        print(f"runtime_dependencies={','.join(map(str, unresolved))}", flush=True)
        print(f"output={output}", flush=True)

        truth_output = args.truth_output or config.output.truth_directory
        if truth_output:
            from lawn.terrain.generator.truth import export_truth

            exported = export_truth(truth_output, config, arrays, controller.assets)
            print(f"truth_output={exported}", flush=True)

        if args.validate_lifecycle:
            # The exported files are already complete. Remove the heavy scene
            # from the live stage before exercising extension/UI lifecycle so
            # RTX does not reload thousands of forest instances on each toggle.
            controller.clear(config.parent_path)
            app.update()

            # Construct and destroy the real window class. Extension startup
            # above already exercised Window-menu registration; this validates
            # the UI body even in a no-window CI session.
            import omni.ui as ui
            from lawn.terrain.generator.constants import EXTENSION_NAME
            from lawn.terrain.generator.ui import GeneratorWindow

            smoke_window = GeneratorWindow(str(extension_root), lambda _root="": controller, str(asset_root))
            smoke_window.show()
            app.update()
            if ui.Workspace.get_window(EXTENSION_NAME) is None:
                raise AssertionError(f"UI window was not registered as {EXTENSION_NAME!r}")
            smoke_window.destroy()
            app.update()
            print("ui_window=PASS", flush=True)

            if not manager.set_extension_enabled_immediate("lawn.terrain.generator", False):
                raise RuntimeError("Extension Manager could not disable lawn.terrain.generator")
            if not manager.set_extension_enabled_immediate("lawn.terrain.generator", True):
                raise RuntimeError("Extension Manager could not re-enable lawn.terrain.generator")
            app.update()
            print("extension_toggle=PASS", flush=True)
        else:
            print("lifecycle_validation=SKIPPED (use --validate-lifecycle)", flush=True)
        print("generation=PASS", flush=True)
        return 0
    except Exception:
        # SimulationApp.close() may terminate Kit before Python's implicit
        # exception hook runs, so emit the actionable traceback first.
        traceback.print_exc()
        return 1
    finally:
        for _ in range(3):
            app.update()
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    raise SystemExit(main())
