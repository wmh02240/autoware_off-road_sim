#!/usr/bin/env python3
"""使用 Isaac Sim 根据配置生成草坪 USD，且不依赖用户界面。"""

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
            # 导出的文件已经完整。在测试扩展/UI 生命周期之前，从实时 Stage 中移除
            # 负载较重的场景，避免 RTX 每次切换时重新加载数千个森林实例。
            controller.clear(config.parent_path)
            app.update()

            # 创建并销毁真实窗口类。上面的扩展启动过程已测试窗口菜单注册；
            # 此处即使在无窗口的 CI 会话中，也会验证 UI 主体。
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
        # SimulationApp.close() 可能会在 Python 隐式异常钩子运行前终止 Kit，
        # 因此先输出可用于定位问题的回溯信息。
        traceback.print_exc()
        return 1
    finally:
        # 此处不要继续推进额外帧，也不要同步关闭负载较重的 USD Stage。
        # 所有请求的文件均已同步导出，并且本工具不运行 Replicator，
        # 因而 Isaac Sim 文档所述的立即退出路径适用于这个一次性进程。
        app.close(wait_for_replicator=False, skip_cleanup=True)


if __name__ == "__main__":
    raise SystemExit(main())
