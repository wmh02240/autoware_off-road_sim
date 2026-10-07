#!/usr/bin/env python3
"""Export a composed USD vehicle as a static, RViz-compatible URDF.

The Isaac Sim URDF exporter preserves PhysX loop joints, which standard URDF
and RViz cannot represent.  This tool intentionally exports the vehicle's
current visual appearance as one mesh attached to a single ``base_link``.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import traceback
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-path", required=True, help="Vehicle USD file")
    parser.add_argument("--output-dir", required=True, help="URDF output directory")
    parser.add_argument(
        "--root-prim",
        default="",
        help="Vehicle root prim; auto-detected from the Chassis when omitted",
    )
    parser.add_argument("--robot-name", default="", help="URDF robot name; defaults to the USD file stem")
    parser.add_argument("--base-link", default="base_link", help="URDF root link name")
    parser.add_argument(
        "--mesh-uri",
        default="",
        help="Mesh URI written to the URDF; defaults to the generated file:// URI",
    )
    return parser.parse_args()


def _safe_name(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return value.strip("_") or "vehicle"


def _triangle_normal(a, b, c) -> tuple[float, float, float]:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length <= 1e-15:
        return 0.0, 0.0, 1.0
    return nx / length, ny / length, nz / length


def _color_tuple(value) -> tuple[float, float, float] | None:
    try:
        values = tuple(float(value[index]) for index in range(3))
    except (IndexError, TypeError, ValueError):
        return None
    if all(math.isfinite(item) for item in values):
        return tuple(max(0.0, min(1.0, item)) for item in values)
    return None


def _material(prim) -> tuple[str, tuple[float, float, float]]:
    from pxr import Usd, UsdGeom, UsdShade

    material_prim = None
    try:
        bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if bound:
            material_prim = bound.GetPrim()
    except Exception:
        material_prim = None

    label = str(material_prim.GetPath()) if material_prim and material_prim.IsValid() else str(prim.GetPath())
    name = _safe_name(label)
    if material_prim and material_prim.IsValid():
        preferred = (
            "diffuse_color_constant",
            "diffuseColor",
            "base_color",
            "baseColor",
            "albedo",
            "color",
        )
        for shader_prim in Usd.PrimRange(material_prim):
            if not shader_prim.IsA(UsdShade.Shader):
                continue
            inputs = {item.GetBaseName(): item for item in UsdShade.Shader(shader_prim).GetInputs()}
            for input_name in preferred:
                if input_name in inputs:
                    color = _color_tuple(inputs[input_name].Get(Usd.TimeCode.Default()))
                    if color is not None:
                        return name, color

    display = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get(Usd.TimeCode.Default())
    if display:
        color = _color_tuple(display[0])
        if color is not None:
            return name, color

    lowered = label.lower()
    fallback = (0.48, 0.50, 0.54)
    for keywords, color in (
        (("tire", "rubber", "wheel"), (0.035, 0.04, 0.045)),
        (("red",), (0.55, 0.035, 0.025)),
        (("blue",), (0.035, 0.12, 0.48)),
        (("black",), (0.025, 0.025, 0.025)),
        (("aluminum", "aluminium", "metal", "steel"), (0.42, 0.45, 0.49)),
        (("glass", "lens"), (0.12, 0.22, 0.28)),
    ):
        if any(keyword in lowered for keyword in keywords):
            fallback = color
            break
    return name, fallback


def _find_vehicle_root(stage, requested_path: str):
    from pxr import Usd

    if requested_path:
        vehicle_root = stage.GetPrimAtPath(requested_path)
        if not vehicle_root.IsValid():
            raise RuntimeError(f"root prim does not exist: {requested_path}")
    else:
        default_root = stage.GetDefaultPrim()
        if not default_root.IsValid():
            raise RuntimeError("stage has no valid default prim")
        vehicle_root = default_root
        for prim in Usd.PrimRange(default_root):
            path = str(prim.GetPath())
            if prim.GetName().lower() == "chassis" and "/Rigid_Bodies/" in path:
                candidate = stage.GetPrimAtPath(path.split("/Rigid_Bodies/", 1)[0])
                if candidate.IsValid():
                    vehicle_root = candidate
                    break

    return vehicle_root


def _write_obj(stage, root, output: Path) -> tuple[int, int, tuple[float, ...], tuple[float, ...]]:
    from pxr import Gf, Usd, UsdGeom

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    xforms = UsdGeom.XformCache(Usd.TimeCode.Default())
    vertex_offset = 0
    mesh_count = 0
    triangle_count = 0
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    reference_root = stage.GetDefaultPrim()
    root_relative, _ = xforms.ComputeRelativeTransform(root, reference_root)
    root_origin = root_relative.Transform(Gf.Vec3d(0.0, 0.0, 0.0))

    prims = []
    materials: dict[str, tuple[float, float, float]] = {}
    # Do not use TraverseInstanceProxies here.  The vehicle contains
    # referenced sensor/graph data whose proxy expansion is extremely
    # expensive, while its render meshes are ordinary composed prims.
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        material_name, color = _material(prim)
        materials.setdefault(material_name, color)
        prims.append((prim, material_name))

    material = output.with_suffix(".mtl")
    with material.open("w", encoding="utf-8") as stream:
        for name, color in materials.items():
            stream.write(
                f"newmtl {name}\n"
                f"Ka {color[0] * 0.25:.6g} {color[1] * 0.25:.6g} {color[2] * 0.25:.6g}\n"
                f"Kd {color[0]:.6g} {color[1]:.6g} {color[2]:.6g}\n"
                "Ks 0.12 0.12 0.12\n"
                "Ns 24\n\n"
            )

    with output.open("w", encoding="utf-8") as stream:
        stream.write(f"mtllib {material.name}\n")
        for prim, material_name in prims:
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get(Usd.TimeCode.Default())
            counts = mesh.GetFaceVertexCountsAttr().Get(Usd.TimeCode.Default())
            indices = mesh.GetFaceVertexIndicesAttr().Get(Usd.TimeCode.Default())
            if not points or not counts or not indices:
                continue

            # Resolve through the stage default prim so the vehicle root's
            # authored unit scale is retained, then remove only its spawn
            # translation.  Computing directly relative to the model root
            # would incorrectly cancel that scale.
            relative, _ = xforms.ComputeRelativeTransform(prim, reference_root)
            transformed = []
            for point in points:
                value = relative.Transform(point)
                xyz = tuple(float(value[index] - root_origin[index]) * meters_per_unit for index in range(3))
                transformed.append(xyz)
                for index in range(3):
                    mins[index] = min(mins[index], xyz[index])
                    maxs[index] = max(maxs[index], xyz[index])

            stream.write(f"\ng {_safe_name(str(prim.GetPath()))}\nusemtl {material_name}\n")
            for x, y, z in transformed:
                stream.write(f"v {x:.9g} {y:.9g} {z:.9g}\n")

            cursor = 0
            flip = mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded
            for count in counts:
                face = [int(value) for value in indices[cursor : cursor + count]]
                cursor += count
                for index in range(1, len(face) - 1):
                    triangle = [face[0], face[index], face[index + 1]]
                    if flip:
                        triangle[1], triangle[2] = triangle[2], triangle[1]
                    normal = _triangle_normal(*(transformed[value] for value in triangle))
                    stream.write(f"vn {normal[0]:.9g} {normal[1]:.9g} {normal[2]:.9g}\n")
                    normal_index = triangle_count + 1
                    obj_indices = [vertex_offset + value + 1 for value in triangle]
                    stream.write("f " + " ".join(f"{value}//{normal_index}" for value in obj_indices) + "\n")
                    triangle_count += 1
            vertex_offset += len(transformed)
            mesh_count += 1

    if mesh_count == 0:
        raise RuntimeError(f"no visible UsdGeom.Mesh prims found below {root.GetPath()}")
    return mesh_count, triangle_count, tuple(mins), tuple(maxs)


def _write_urdf(output: Path, robot_name: str, base_link: str, mesh_uri: str) -> None:
    import xml.etree.ElementTree as ET

    robot = ET.Element("robot", {"name": robot_name})
    link = ET.SubElement(robot, "link", {"name": base_link})
    visual = ET.SubElement(link, "visual", {"name": "vehicle_visual"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": mesh_uri})
    ET.indent(robot, space="  ")
    output.write_bytes(b'<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(robot, encoding="utf-8") + b"\n")


def main() -> int:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        from pxr import Usd

        usd_path = Path(args.usd_path).resolve()
        output_dir = Path(args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        mesh_dir = output_dir / "meshes"
        mesh_dir.mkdir(parents=True, exist_ok=True)
        robot_name = _safe_name(args.robot_name or usd_path.stem)
        mesh_path = mesh_dir / f"{robot_name}.obj"
        urdf_path = output_dir / f"{robot_name}.urdf"

        stage = Usd.Stage.Open(str(usd_path))
        if stage is None:
            raise RuntimeError(f"could not open USD stage: {usd_path}")
        root = _find_vehicle_root(stage, args.root_prim)

        mesh_count, triangle_count, mins, maxs = _write_obj(stage, root, mesh_path)
        dimensions = tuple(maxs[index] - mins[index] for index in range(3))
        mesh_uri = args.mesh_uri or mesh_path.as_uri()
        _write_urdf(urdf_path, robot_name, args.base_link, mesh_uri)
        print(f"urdf={urdf_path}", flush=True)
        print(f"mesh={mesh_path}", flush=True)
        print(f"source_meshes={mesh_count}", flush=True)
        print(f"triangles={triangle_count}", flush=True)
        print(f"vehicle_root={root.GetPath()}", flush=True)
        print(f"base_frame={root.GetPath()}", flush=True)
        print("bounds_min_m=" + ",".join(f"{value:.6g}" for value in mins), flush=True)
        print("bounds_max_m=" + ",".join(f"{value:.6g}" for value in maxs), flush=True)
        print("dimensions_m=" + ",".join(f"{value:.6g}" for value in dimensions), flush=True)
        print("rviz_export=PASS", flush=True)
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        app.close(wait_for_replicator=False, skip_cleanup=True)


if __name__ == "__main__":
    sys.exit(main())
