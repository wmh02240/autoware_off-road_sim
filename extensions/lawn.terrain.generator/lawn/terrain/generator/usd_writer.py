"""USD authoring for generated arrays. Call this module only on Kit's main thread."""

from __future__ import annotations

import re
import threading
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

from .assets import AssetRecord
from .compute import GeneratedArrays
from .config import BusinessObjectConfig, GeneratorConfig


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


class UsdSceneWriter:
    """Owns stage mutations and rejects accidental worker-thread writes."""

    def __init__(self, stage: Usd.Stage, asset_root: str = ""):
        self.stage = stage
        self.asset_root = Path(asset_root).resolve() if asset_root else Path()
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
        root.SetCustomDataByKey("lawnGeneratorVersion", "3.0.0-autoware.1")
        root.SetCustomDataByKey("lawnSeed", config.seed)
        root.SetCustomDataByKey("sceneName", config.scene_name)
        root.SetCustomDataByKey("terrainMode", config.terrain.mode)
        world = self.stage.GetPrimAtPath("/World")
        if world.IsValid() and not self.stage.GetDefaultPrim().IsValid():
            self.stage.SetDefaultPrim(world)
        self._add_terrain(config, arrays)
        counts = self._add_assets(config, arrays, assets)
        object_counts = self._add_business_objects(config, arrays, assets)
        self._add_semantics(config, arrays)
        self._add_lighting(config)
        return {"terrain_vertices": len(arrays.vertices), **counts, **object_counts}

    def _add_terrain(self, config: GeneratorConfig, arrays: GeneratedArrays) -> None:
        terrain_path = f"{config.parent_path}/Terrain"
        mesh = UsdGeom.Mesh.Define(self.stage, f"{terrain_path}/Mesh")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(arrays.vertices))
        mesh.CreateFaceVertexCountsAttr([3] * len(arrays.face_indices))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(arrays.face_indices.reshape(-1)))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(False)
        mesh.GetPrim().SetCustomDataByKey("semanticLabel", "terrain_grass")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        collision.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)
        self._create_terrain_material(config, f"{terrain_path}/GroundMaterial", mesh)

    def _asset_path(self, value: str) -> str:
        if not value:
            return ""
        source = Path(value).expanduser()
        return str((source if source.is_absolute() else self.asset_root / source).resolve())

    def _create_terrain_material(self, config: GeneratorConfig, path: str, mesh: UsdGeom.Mesh) -> None:
        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, f"{path}/OmniPBRShader")
        shader.CreateImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
        shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*config.surface.diffuse_color))
        shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.9)
        texture_inputs = {
            "diffuse_texture": config.surface.diffuse_texture,
            "normalmap_texture": config.surface.normal_texture,
            "reflectionroughness_texture": config.surface.roughness_texture,
            "ao_texture": config.surface.ao_texture,
        }
        for input_name, source in texture_inputs.items():
            resolved = self._asset_path(source)
            if resolved:
                if not Path(resolved).is_file():
                    raise ValueError(f"surface texture does not exist: {resolved}")
                shader.CreateInput(input_name, Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(resolved))
        shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(config.surface.project_uvw)
        shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(config.surface.texture_scale, config.surface.texture_scale))
        if config.surface.ao_texture:
            shader.CreateInput("ao_to_diffuse", Sdf.ValueTypeNames.Float).Set(1.0)
        material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    def _add_lighting(self, config: GeneratorConfig) -> None:
        if not config.lighting.enabled:
            return
        parent = f"{config.parent_path}/Lighting"
        UsdGeom.Xform.Define(self.stage, parent)
        if config.lighting.hdri_path:
            hdri = self._asset_path(config.lighting.hdri_path)
            if not Path(hdri).is_file():
                raise ValueError(f"lighting HDRI does not exist: {hdri}")
            dome = UsdLux.DomeLight.Define(self.stage, f"{parent}/ForestDomeLight")
            dome.CreateTextureFileAttr(Sdf.AssetPath(hdri))
            dome.CreateIntensityAttr(config.lighting.dome_intensity)
            UsdGeom.XformCommonAPI(dome.GetPrim()).SetRotate(Gf.Vec3f(*config.lighting.dome_rotation_deg))
        sun = UsdLux.DistantLight.Define(self.stage, f"{parent}/Sun")
        sun.CreateIntensityAttr(config.lighting.sun_intensity)
        sun.CreateAngleAttr(config.lighting.sun_angle_deg)
        sun.CreateEnableColorTemperatureAttr(True)
        sun.CreateColorTemperatureAttr(config.lighting.color_temperature_k)
        UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(Gf.Vec3f(*config.lighting.sun_rotation_deg))

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
            semantic = asset_cfg.semantic_label or (
                "vegetation_short_grass" if asset_cfg.name == "Grass" else
                "vegetation_tall_grass" if asset_cfg.name == "Switchgrass" else
                "tree_trunk" if record.kind == "tree" else record.kind
            )
            scaled = scales * record.base_scale

            # The scanned tree USDs contain their own branch PointInstancers and
            # GeomSubsets. Nesting those assets inside another PointInstancer
            # breaks Fabric material paths and can drop the outer 0.01 scale on
            # leaf instances. Direct references still share asset data while
            # preserving the complete internal hierarchy and transforms.
            if record.kind == "tree":
                self._add_tree_references(
                    config.parent_path, name, record, positions, orientations, scaled, semantic, asset_cfg.lidar_visible
                )
                self._add_collision_proxies(config.parent_path, name, record.kind, positions, scaled)
                continue

            instancer = UsdGeom.PointInstancer.Define(
                self.stage, f"{config.parent_path}/Instances/{name}"
            )
            prototype = UsdGeom.Xform.Define(
                self.stage, f"{config.parent_path}/Instances/{name}/Prototypes/{name}"
            ).GetPrim()
            prototype.GetReferences().AddReference(str(record.path))
            prototype.SetInstanceable(True)
            prototype.SetCustomDataByKey("semanticLabel", semantic)
            prototype.SetCustomDataByKey("lidarVisible", asset_cfg.lidar_visible)
            instancer.GetPrototypesRel().SetTargets([prototype.GetPath()])
            instancer.CreateProtoIndicesAttr([0] * count)
            instancer.CreatePositionsAttr([Gf.Vec3f(*map(float, p)) for p in positions])
            instancer.CreateOrientationsAttr(
                [Gf.Quath(float(q[0]), Gf.Vec3h(float(q[1]), float(q[2]), float(q[3]))) for q in orientations]
            )
            instancer.CreateScalesAttr([Gf.Vec3f(float(s), float(s), float(s)) for s in scaled])
            # Collision proxies remain simple primitives. Referenced scan meshes never
            # receive triangle-mesh collision APIs per instance.
            if record.kind in {"tree", "rock"}:
                self._add_collision_proxies(config.parent_path, name, record.kind, positions, scaled)
        return result

    def _add_tree_references(
        self,
        parent: str,
        name: str,
        record: AssetRecord,
        positions,
        orientations,
        scales,
        semantic: str,
        lidar_visible: bool,
    ) -> None:
        tree_parent = f"{parent}/Instances/{name}"
        UsdGeom.Xform.Define(self.stage, tree_parent)
        for index, (position, orientation, scale) in enumerate(zip(positions, orientations, scales)):
            prim = UsdGeom.Xform.Define(self.stage, f"{tree_parent}/Instance_{index:04d}").GetPrim()
            prim.GetReferences().AddReference(str(record.path))
            prim.SetCustomDataByKey("semanticLabel", semantic)
            prim.SetCustomDataByKey("lidarVisible", lidar_visible)
            xform = UsdGeom.Xformable(prim)
            xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
                Gf.Quatf(float(orientation[0]), Gf.Vec3f(*map(float, orientation[1:4])))
            )
            xform.AddScaleOp().Set(Gf.Vec3d(float(scale), float(scale), float(scale)))

    def _ground_height(self, arrays: GeneratedArrays, config: GeneratorConfig, x: float, y: float) -> float:
        sx, sy = config.terrain.size_m
        ix = int(round((x + sx / 2) / sx * (arrays.heights.shape[0] - 1)))
        iy = int(round((y + sy / 2) / sy * (arrays.heights.shape[1] - 1)))
        ix = max(0, min(ix, arrays.heights.shape[0] - 1))
        iy = max(0, min(iy, arrays.heights.shape[1] - 1))
        return float(arrays.heights[ix, iy])

    def _add_business_objects(
        self,
        config: GeneratorConfig,
        arrays: GeneratedArrays,
        assets: dict[str, AssetRecord],
    ) -> dict[str, int]:
        result: dict[str, int] = {}
        if not config.objects:
            return result
        UsdGeom.Xform.Define(self.stage, f"{config.parent_path}/BusinessObjects")
        for item in config.objects:
            name = _safe_name(item.name)
            path = f"{config.parent_path}/BusinessObjects/{name}"
            ground = self._ground_height(arrays, config, item.position_m[0], item.position_m[1])
            record = assets.get(item.asset_name) if item.asset_name else None
            if item.asset_name and (record is None or not record.available):
                result[f"object:{item.name}"] = 0
                continue
            if record is not None:
                prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
                prim.GetReferences().AddReference(str(record.path))
                xform = UsdGeom.Xformable(prim)
                xform.AddTranslateOp().Set(Gf.Vec3d(item.position_m[0], item.position_m[1], ground + item.position_m[2]))
                xform.AddRotateZOp().Set(item.yaw_deg)
                xform.AddScaleOp().Set(Gf.Vec3f(*(float(v) * record.base_scale for v in item.size_m)))
                if item.collision:
                    proxy = UsdGeom.Cube.Define(self.stage, f"{path}/CollisionProxy")
                    proxy.CreateSizeAttr(1.0)
                    UsdGeom.Imageable(proxy.GetPrim()).MakeInvisible()
                    proxy_xform = UsdGeom.Xformable(proxy.GetPrim())
                    proxy_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, item.size_m[2] / 2))
                    proxy_xform.AddScaleOp().Set(Gf.Vec3f(*map(float, item.size_m)))
                    UsdPhysics.CollisionAPI.Apply(proxy.GetPrim())
            elif item.shape == "cylinder":
                shape = UsdGeom.Cylinder.Define(self.stage, path)
                shape.CreateAxisAttr(UsdGeom.Tokens.z)
                shape.CreateRadiusAttr(max(item.size_m[0], item.size_m[1]) / 2)
                shape.CreateHeightAttr(item.size_m[2])
                prim = shape.GetPrim()
                xform = UsdGeom.Xformable(prim)
                xform.AddTranslateOp().Set(Gf.Vec3d(item.position_m[0], item.position_m[1], ground + item.position_m[2] + item.size_m[2] / 2))
                xform.AddRotateZOp().Set(item.yaw_deg)
            else:
                shape = UsdGeom.Cube.Define(self.stage, path)
                shape.CreateSizeAttr(1.0)
                prim = shape.GetPrim()
                xform = UsdGeom.Xformable(prim)
                xform.AddTranslateOp().Set(Gf.Vec3d(item.position_m[0], item.position_m[1], ground + item.position_m[2] + item.size_m[2] / 2))
                xform.AddRotateZOp().Set(item.yaw_deg)
                xform.AddScaleOp().Set(Gf.Vec3f(*map(float, item.size_m)))
            prim.SetCustomDataByKey("semanticLabel", item.semantic_label)
            prim.SetCustomDataByKey("businessCategory", item.category)
            if item.collision and record is None:
                UsdPhysics.CollisionAPI.Apply(prim)
                PhysxSchema.PhysxCollisionAPI.Apply(prim)
            result[f"object:{item.name}"] = 1
        return result

    def _add_semantics(self, config: GeneratorConfig, arrays: GeneratedArrays) -> None:
        scope = UsdGeom.Scope.Define(self.stage, f"{config.parent_path}/Semantics").GetPrim()
        scope.CreateAttribute("lawn:gridResolutionM", Sdf.ValueTypeNames.Double).Set(config.terrain.resolution_m)
        scope.CreateAttribute("lawn:gridWidth", Sdf.ValueTypeNames.Int).Set(int(arrays.heights.shape[0]))
        scope.CreateAttribute("lawn:gridHeight", Sdf.ValueTypeNames.Int).Set(int(arrays.heights.shape[1]))
        for suffix, value in zip(("X", "Y", "Z", "YawDeg"), arrays.recommended_spawn_pose):
            scope.CreateAttribute(f"lawn:recommendedSpawn{suffix}", Sdf.ValueTypeNames.Double).Set(float(value))
        for name, mask in arrays.masks.items():
            region = UsdGeom.Scope.Define(self.stage, f"{config.parent_path}/Semantics/{_safe_name(name)}").GetPrim()
            region.SetCustomDataByKey("semanticLabel", name)
            region.SetCustomDataByKey("cellCount", int(mask.sum()))

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
            UsdGeom.Imageable(shape.GetPrim()).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
