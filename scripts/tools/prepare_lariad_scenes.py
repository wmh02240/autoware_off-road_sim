#!/usr/bin/env python3
"""为 LARIAD 场景创建非破坏性的 Isaac Sim 6 兼容图层。

请使用 Isaac Sim 自带的 Python 运行本脚本。生成的图层位于被忽略的外部资源
检出目录中；上游二进制 USD 文件绝不会被修改。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

try:
    from pxr import Kind, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
except ImportError as exc:
    raise SystemExit(
        "pxr is unavailable. Run with Isaac Sim's bundled python.sh, not system python3."
    ) from exc


LARIAD_COMMIT = "69640cf19eec1d90214ed4a3615788dd5dc1e3c4"
SCENE_NAMES = ("easy", "medium", "hard")
RUNTIME_PROVIDED_ASSETS = {"OmniPBR.mdl"}
GENERATOR_ASSETS = {
    "Switchgrass": "Switchgrass/Switchgrass.usd",
    "Container": "Container_J01/Container_J01_126x120x133cm_PR_V_NVD_01.usd",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_head(checkout: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _rewrite_asset_path(asset_path: str) -> str:
    legacy_prefix = "../isaac/extsUser/terrain.generator/"
    if asset_path.startswith(legacy_prefix):
        return "../terrain.generator/" + asset_path[len(legacy_prefix) :]
    if asset_path == "./edges.usd":
        return "../assets/edges.usd"
    return asset_path


def _rewrite_reference(reference: Sdf.Reference) -> Sdf.Reference:
    return Sdf.Reference(
        _rewrite_asset_path(reference.assetPath),
        reference.primPath,
        reference.layerOffset,
        reference.customData,
    )


def _rewrite_payload(payload: Sdf.Payload) -> Sdf.Payload:
    return Sdf.Payload(
        _rewrite_asset_path(payload.assetPath),
        payload.primPath,
        payload.layerOffset,
    )


def _move_looks_under_map(layer: Sdf.Layer) -> None:
    if not layer.GetPrimAtPath("/Looks"):
        return
    edit = Sdf.BatchNamespaceEdit()
    edit.Add(Sdf.Path("/Looks"), Sdf.Path("/map/Looks"))
    if not layer.CanApply(edit):
        raise RuntimeError("Cannot move /Looks below /map")
    if not layer.Apply(edit):
        raise RuntimeError("Failed to move /Looks below /map")

    old_prefix = Sdf.Path("/Looks")
    new_prefix = Sdf.Path("/map/Looks")

    def rewrite_path(path: Sdf.Path) -> Sdf.Path:
        return path.ReplacePrefix(old_prefix, new_prefix) if path.HasPrefix(old_prefix) else path

    def rewrite_targets(path: Sdf.Path) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.RelationshipSpec):
            spec.targetPathList.ModifyItemEdits(rewrite_path)
        elif isinstance(spec, Sdf.AttributeSpec):
            spec.connectionPathList.ModifyItemEdits(rewrite_path)

    layer.Traverse("/", rewrite_targets)


def _rewrite_dependencies(layer: Sdf.Layer) -> None:
    def rewrite(path: Sdf.Path) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PrimSpec):
            spec.referenceList.ModifyItemEdits(_rewrite_reference)
            spec.payloadList.ModifyItemEdits(_rewrite_payload)
        elif isinstance(spec, Sdf.AttributeSpec):
            value = spec.default
            if isinstance(value, Sdf.AssetPath) and value.path:
                rewritten = _rewrite_asset_path(value.path)
                if rewritten != value.path:
                    spec.default = Sdf.AssetPath(rewritten)

    layer.Traverse("/", rewrite)


def _remove_spec_if_present(layer: Sdf.Layer, path: str) -> None:
    sdf_path = Sdf.Path(path)
    if layer.GetObjectAtPath(sdf_path):
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(sdf_path, Sdf.Path.emptyPath)
        if not layer.CanApply(edit) or not layer.Apply(edit):
            raise RuntimeError(f"Cannot remove {path} from {layer.identifier}")


def _prepare_scene(source: Path, output: Path) -> dict:
    source_layer = Sdf.Layer.FindOrOpen(str(source))
    if not source_layer:
        raise RuntimeError(f"Cannot open source layer: {source}")

    output.parent.mkdir(parents=True, exist_ok=True)
    output_layer = (
        Sdf.Layer.FindOrOpen(str(output))
        if output.exists()
        else Sdf.Layer.CreateNew(str(output))
    )
    if not output_layer:
        raise RuntimeError(f"Cannot create output layer: {output}")
    output_layer.TransferContent(source_layer)

    # 保持上游文件不变，同时排除其中的机器人、缺失的占位 Payload，
    # 以及仅供编辑器使用的测量辅助对象。
    _remove_spec_if_present(output_layer, "/map/odom")
    _remove_spec_if_present(output_layer, "/map/block")
    _remove_spec_if_present(output_layer, "/Viewport_Measure")

    _move_looks_under_map(output_layer)
    _rewrite_dependencies(output_layer)
    output_layer.Save()

    stage = Usd.Stage.Open(str(output), Usd.Stage.LoadAll)
    if not stage:
        raise RuntimeError(f"Cannot compose adapted stage: {output}")
    map_prim = stage.GetPrimAtPath("/map")
    if not map_prim:
        raise RuntimeError(f"Adapted stage has no /map prim: {output}")

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetDefaultPrim(map_prim)
    Usd.ModelAPI(map_prim).SetKind(Kind.Tokens.assembly)

    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.Scene):
            stage.RemovePrim(prim.GetPath())

    stage.GetRootLayer().customLayerData = {
        "adapter": "autoware-off-road-sim phase 1",
        "lariadCommit": LARIAD_COMMIT,
        "sourceLayer": f"../assets/{source.name}",
    }
    stage.GetRootLayer().Save()
    return _validate_scene(output)


def _validate_scene(path: Path) -> dict:
    stage = Usd.Stage.Open(str(path), Usd.Stage.LoadAll)
    if not stage:
        raise RuntimeError(f"Cannot open adapted stage: {path}")

    errors: list[str] = []
    default_prim = stage.GetDefaultPrim()
    if not default_prim or default_prim.GetPath() != Sdf.Path("/map"):
        errors.append("defaultPrim must be /map")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        errors.append("metersPerUnit must be 1")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        errors.append("upAxis must be Z")
    if stage.GetPrimAtPath("/map/odom"):
        errors.append("embedded Barakuda prim /map/odom was not removed")
    if stage.GetPrimAtPath("/map/block"):
        errors.append("missing block payload prim /map/block was not removed")
    if stage.GetPrimAtPath("/Looks"):
        errors.append("/Looks must be nested below /map")
    if not stage.GetPrimAtPath("/map/Looks"):
        errors.append("/map/Looks is missing")

    physics_scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if physics_scenes:
        errors.append(f"environment contains PhysicsScene prims: {physics_scenes}")

    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(path))
    unresolved_paths = sorted(str(item) for item in unresolved)
    unexpected_unresolved = [
        item for item in unresolved_paths if Path(item).name not in RUNTIME_PROVIDED_ASSETS
    ]
    if unexpected_unresolved:
        errors.append(f"unexpected unresolved dependencies: {unexpected_unresolved}")

    if errors:
        raise RuntimeError(f"Validation failed for {path.name}: " + "; ".join(errors))

    return {
        "file": path.name,
        "default_prim": str(default_prim.GetPath()),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "physics_scenes": physics_scenes,
        "runtime_provided_assets": unresolved_paths,
        "sha256": _sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    repo_root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--lariad-root",
        type=Path,
        default=repo_root / "external_assets" / "lariad_offroad_nav",
    )
    parser.add_argument("--check", action="store_true", help="Validate existing outputs only")
    args = parser.parse_args()

    lariad_root = args.lariad_root.resolve()
    if _git_head(lariad_root) != LARIAD_COMMIT:
        raise SystemExit(f"LARIAD checkout does not match pinned commit {LARIAD_COMMIT}")

    output_dir = lariad_root / "compat"
    results = []
    for name in SCENE_NAMES:
        output = output_dir / f"{name}.usda"
        if args.check:
            results.append(_validate_scene(output))
        else:
            results.append(_prepare_scene(lariad_root / "assets" / f"{name}.usd", output))
        print(f"[OK] {name}: {output}")

    asset_status = {}
    data_root = lariad_root / "terrain.generator" / "data"
    for asset_name, relative_path in GENERATOR_ASSETS.items():
        asset_path = data_root / relative_path
        asset_status[asset_name] = {
            "path": relative_path,
            "present": asset_path.is_file(),
            "enabled": asset_path.is_file(),
        }

    manifest = {
        "schema_version": 1,
        "lariad_commit": LARIAD_COMMIT,
        "scenes": results,
        "generator_asset_status": asset_status,
        "notes": [
            "OmniPBR.mdl is supplied by the Isaac Sim MDL runtime.",
            "Switchgrass and Container remain disabled while their source files are absent.",
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
