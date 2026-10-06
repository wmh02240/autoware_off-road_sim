#!/usr/bin/env python3
"""为生成的草坪 USD 渲染具有确定性的验证预览图。"""

from __future__ import annotations

import argparse
import traceback
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
    try:
        import carb
        import omni.usd
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        from pxr import Gf, UsdGeom

        carb.settings.get_settings().set_int("/persistent/app/viewport/displayOptions", 0)
        stage_path = str(Path(args.stage).resolve())
        context = omni.usd.get_context()
        if not context.open_stage(stage_path):
            raise RuntimeError(f"could not open stage: {stage_path}")
        for _ in range(60):
            app.update()
        stage = context.get_stage()
        camera = UsdGeom.Camera.Define(stage, "/World/LawnPreviewCamera")
        camera.CreateFocalLengthAttr(24.0)
        camera.CreateHorizontalApertureAttr(36.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000.0))
        eye = Gf.Vec3d(-10.0, -14.0, 7.0)
        target = Gf.Vec3d(2.0, 11.0, 1.5)
        transform = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse()
        camera.AddTransformOp().Set(transform)
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("no active viewport is available")
        viewport.set_active_camera(str(camera.GetPath()))
        viewport.set_texture_resolution((args.width, args.height))
        for _ in range(90):
            app.update()
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        capture_viewport_to_file(viewport, str(output))
        for _ in range(240):
            app.update()
        if not output.is_file():
            raise RuntimeError(f"preview capture did not complete: {output}")
        output.chmod(0o644)
        print(f"preview={output}", flush=True)
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    raise SystemExit(main())
