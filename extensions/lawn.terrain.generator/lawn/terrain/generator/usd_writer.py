"""USD authoring for generated arrays. Call this module only on Kit's main thread."""

from __future__ import annotations

import re
import threading
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from .assets import AssetRecord
from .compute import GeneratedArrays
from .config import GeneratorConfig


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


class UsdSceneWriter:
    """Owns stage mutations and rejects accidental worker-thread writes."""

    def __init__(self, stage: Usd.Stage):
        self.stage = stage
        self._main_thread = threading.get_ident()

    def _assert_main_thread(self) -> None:
        if threading.get_ident() != self._main_thread:
            raise RuntimeError("USD stage edits must run on the Kit main thread")

    def clear(self, parent_path: str) -> None:
        self._assert_main_thread()
        path = Sdf.Path(parent_path)
        if self.stage.GetPrimAtPath(path).IsValid():
            self.stage.RemovePrim(path)

    def apply(
        self,
        config: GeneratorConfig,
        arrays: GeneratedArrays,
        assets: dict[str, AssetRecord],
    ) -> dict[str, int]:
        self._assert_main_thread()
        self.clear(config.parent_path)
        root = UsdGeom.Xform.Define(self.stage, config.parent_path).GetPrim()
        Usd.ModelAPI(root).SetKind("assembly")
        root.SetCustomDataByKey("lawnGeneratorVersion", "2.0.0-autoware.1")
        root.SetCustomDataByKey("lawnSeed", config.seed)
        world = self.stage.GetPrimAtPath("/World")
        if world.IsValid() and not self.stage.GetDefaultPrim().IsValid():
            self.stage.SetDefaultPrim(world)
        self._add_terrain(config, arrays)
        counts = self._add_assets(config, arrays, assets)
        return {"terrain_vertices": len(arrays.vertices), **counts}

    def _add_terrain(self, config: GeneratorConfig, arrays: GeneratedArrays) -> None:
        terrain_path = f"{config.parent_path}/Terrain"
        mesh = UsdGeom.Mesh.Define(self.stage, f"{terrain_path}/Mesh")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(arrays.vertices))
        mesh.CreateFaceVertexCountsAttr([3] * len(arrays.face_indices))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(arrays.face_indices.reshape(-1)))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(False)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        collision.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)
        self._create_terrain_material(f"{terrain_path}/GroundMaterial", mesh)

    def _create_terrain_material(self, path: str, mesh: UsdGeom.Mesh) -> None:
        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, f"{path}/OmniPBRShader")
        shader.CreateImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
        shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.12, 0.28, 0.07))
        shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.9)
        material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    def _add_assets(
        self,
        config: GeneratorConfig,
        arrays: GeneratedArrays,
        assets: dict[str, AssetRecord],
    ) -> dict[str, int]:
        result: dict[str, int] = {}
        for asset_cfg in config.assets:
            placement = arrays.placements.get(asset_cfg.name)
            record = assets.get(asset_cfg.name)
            if placement is None or record is None or not record.available:
                result[asset_cfg.name] = 0
                continue
            positions, orientations, scales = placement
            count = len(positions)
            result[asset_cfg.name] = count
            if count == 0:
                continue
            name = _safe_name(asset_cfg.name)
            instancer = UsdGeom.PointInstancer.Define(
                self.stage, f"{config.parent_path}/Instances/{name}"
            )
            prototype = UsdGeom.Xform.Define(
                self.stage, f"{config.parent_path}/Instances/{name}/Prototypes/{name}"
            ).GetPrim()
            prototype.GetReferences().AddReference(str(record.path))
            prototype.SetInstanceable(True)
            instancer.GetPrototypesRel().SetTargets([prototype.GetPath()])
            instancer.CreateProtoIndicesAttr([0] * count)
            instancer.CreatePositionsAttr([Gf.Vec3f(*map(float, p)) for p in positions])
            instancer.CreateOrientationsAttr(
                [Gf.Quath(float(q[0]), Gf.Vec3h(float(q[1]), float(q[2]), float(q[3]))) for q in orientations]
            )
            scaled = scales * record.base_scale
            instancer.CreateScalesAttr([Gf.Vec3f(float(s), float(s), float(s)) for s in scaled])
            # Collision proxies remain simple primitives. Referenced scan meshes never
            # receive triangle-mesh collision APIs per instance.
            if record.kind in {"tree", "rock"}:
                self._add_collision_proxies(config.parent_path, name, record.kind, positions, scaled)
        return result

    def _add_collision_proxies(self, parent: str, name: str, kind: str, positions, scales) -> None:
        proxy_parent = f"{parent}/CollisionProxies/{name}"
        UsdGeom.Xform.Define(self.stage, proxy_parent)
        for index, (position, scale) in enumerate(zip(positions, scales)):
            path = f"{proxy_parent}/Proxy_{index}"
            if kind == "tree":
                shape = UsdGeom.Capsule.Define(self.stage, path)
                shape.CreateAxisAttr(UsdGeom.Tokens.z)
                shape.CreateRadiusAttr(max(0.08, float(scale) * 18.0))
                shape.CreateHeightAttr(max(0.5, float(scale) * 450.0))
            else:
                shape = UsdGeom.Sphere.Define(self.stage, path)
                shape.CreateRadiusAttr(max(0.1, float(scale) * 0.45))
            xform = UsdGeom.Xformable(shape.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
