from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


SUPPORTED_TERRAIN_MODES = {"flat", "single_slope", "rolling_lawn", "terraced_lawn", "heightmap", "legacy_fbm"}
SUPPORTED_ZONE_KINDS = {"no_mow_zone", "bare_soil_patch", "spawn_pad"}
SUPPORTED_ZONE_SHAPES = {"circle", "rectangle", "polygon"}
SUPPORTED_OBJECT_SHAPES = {"box", "cylinder"}


@dataclass(frozen=True)
class SpawnPadConfig:
    center_m: tuple[float, float] = (0.0, 0.0)
    radius_m: float = 0.0


@dataclass(frozen=True)
class TerrainConfig:
    size_m: tuple[float, float] = (20.0, 20.0)
    resolution_m: float = 0.5
    elevation_delta_m: float = 1.0
    octaves: int = 4
    persistence: float = 0.4
    lacunarity: float = 2.0
    mode: str = "legacy_fbm"
    base_height_m: float = 0.0
    slope_direction_deg: float = 0.0
    slope_deg: float = 0.0
    max_slope_deg: float = 15.0
    max_cross_slope_deg: float = 15.0
    max_local_curvature_per_m: float = 2.0
    smoothing_passes: int = 2
    wavelength_m: float = 18.0
    terrace_count: int = 3
    transition_width_m: float = 1.0
    heightmap_path: str = ""
    heightmap_scale_m: float = 1.0
    spawn_pad: SpawnPadConfig = field(default_factory=SpawnPadConfig)


@dataclass(frozen=True)
class ZoneConfig:
    name: str
    kind: str
    shape: str
    center_m: tuple[float, float] = (0.0, 0.0)
    radius_m: float = 0.0
    size_m: tuple[float, float] = (0.0, 0.0)
    points_m: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class RegionConfig:
    enabled: bool = False
    boundary_buffer_m: float = 0.0
    tall_grass_band_m: float = 0.0
    bare_soil_fraction: float = 0.0
    bare_soil_patch_count: int = 0
    zones: tuple[ZoneConfig, ...] = ()


@dataclass(frozen=True)
class AssetConfig:
    name: str
    density_per_100m2: float
    scale_min: float = 0.9
    scale_max: float = 1.1
    allowed_masks: tuple[str, ...] = ()
    forbidden_masks: tuple[str, ...] = ()
    clearance_m: float = 0.0
    semantic_label: str = ""
    lidar_visible: bool = True
    align_to_surface: bool = False


@dataclass(frozen=True)
class AssetManifestEntry:
    name: str
    path: str
    kind: str = "object"
    base_scale: float = 1.0


@dataclass(frozen=True)
class BusinessObjectConfig:
    name: str
    category: str
    shape: str
    position_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    yaw_deg: float = 0.0
    semantic_label: str = ""
    collision: bool = True
    asset_name: str = ""


@dataclass(frozen=True)
class OutputConfig:
    truth_directory: str = ""


@dataclass(frozen=True)
class SurfaceConfig:
    diffuse_texture: str = ""
    normal_texture: str = ""
    roughness_texture: str = ""
    ao_texture: str = ""
    project_uvw: bool = True
    texture_scale: float = 1.0
    diffuse_color: tuple[float, float, float] = (0.12, 0.28, 0.07)


@dataclass(frozen=True)
class LightingConfig:
    enabled: bool = False
    hdri_path: str = ""
    dome_intensity: float = 1000.0
    dome_rotation_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    sun_intensity: float = 2500.0
    sun_angle_deg: float = 1.0
    sun_rotation_deg: tuple[float, float, float] = (35.0, -25.0, 25.0)
    color_temperature_k: float = 5500.0


@dataclass(frozen=True)
class GeneratorConfig:
    schema_version: int = 1
    seed: int = 1
    parent_path: str = "/World/GeneratedForest"
    asset_root: str = ""
    scene_name: str = "generated_lawn"
    preset: str = ""
    terrain: TerrainConfig = field(default_factory=TerrainConfig)
    regions: RegionConfig = field(default_factory=RegionConfig)
    assets: tuple[AssetConfig, ...] = ()
    asset_manifest: tuple[AssetManifestEntry, ...] = ()
    objects: tuple[BusinessObjectConfig, ...] = ()
    output: OutputConfig = field(default_factory=OutputConfig)
    surface: SurfaceConfig = field(default_factory=SurfaceConfig)
    lighting: LightingConfig = field(default_factory=LightingConfig)


def _mapping_from_file(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        value = json.loads(text)
    else:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for YAML generator configs") from exc
        value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError("generator config must contain a mapping")
    return value


def _pair(value: Any, field_name: str, *, positive: bool = False) -> tuple[float, float]:
    result = tuple(float(v) for v in value)
    if len(result) != 2 or (positive and min(result) <= 0):
        qualifier = " two positive values" if positive else " two values"
        raise ValueError(f"{field_name} must contain{qualifier}")
    return result[0], result[1]


def _triple(value: Any, field_name: str, *, positive: bool = False) -> tuple[float, float, float]:
    result = tuple(float(v) for v in value)
    if len(result) != 3 or (positive and min(result) <= 0):
        qualifier = " three positive values" if positive else " three values"
        raise ValueError(f"{field_name} must contain{qualifier}")
    return result[0], result[1], result[2]


def _load_terrain(source: dict[str, Any], config_dir: Path) -> TerrainConfig:
    size = _pair(source.get("size_m", (20.0, 20.0)), "terrain.size_m", positive=True)
    resolution = float(source.get("resolution_m", 0.5))
    if resolution <= 0:
        raise ValueError("terrain.resolution_m must be positive")
    # An omitted mode is the stage-2 schema. Keep its exact heightfield path so
    # previously accepted configs and deterministic hashes remain unchanged.
    mode = str(source.get("mode", "legacy_fbm"))
    if mode not in SUPPORTED_TERRAIN_MODES:
        raise ValueError(f"unsupported terrain.mode: {mode}")
    spawn_source = source.get("spawn_pad", {}) or {}
    spawn = SpawnPadConfig(
        _pair(spawn_source.get("center_m", (0.0, 0.0)), "terrain.spawn_pad.center_m"),
        max(0.0, float(spawn_source.get("radius_m", 0.0))),
    )
    heightmap_path = str(source.get("heightmap_path", ""))
    if heightmap_path and not Path(heightmap_path).is_absolute():
        heightmap_path = str((config_dir / heightmap_path).resolve())
    maximum_slope = float(source.get("max_slope_deg", 15.0))
    maximum_cross_slope = float(source.get("max_cross_slope_deg", maximum_slope))
    if not (0 < maximum_slope < 90) or not (0 < maximum_cross_slope < 90):
        raise ValueError("terrain slope limits must be between 0 and 90 degrees")
    return TerrainConfig(
        size_m=size,
        resolution_m=resolution,
        elevation_delta_m=max(0.0, float(source.get("elevation_delta_m", source.get("max_elevation_delta_m", 1.0)))),
        octaves=max(1, int(source.get("octaves", 4))),
        persistence=float(source.get("persistence", 0.4)),
        lacunarity=max(1.0, float(source.get("lacunarity", 2.0))),
        mode=mode,
        base_height_m=float(source.get("base_height_m", 0.0)),
        slope_direction_deg=float(source.get("slope_direction_deg", 0.0)),
        slope_deg=float(source.get("slope_deg", 0.0)),
        max_slope_deg=maximum_slope,
        max_cross_slope_deg=maximum_cross_slope,
        max_local_curvature_per_m=max(0.0, float(source.get("max_local_curvature_per_m", 2.0))),
        smoothing_passes=max(0, int(source.get("smoothing_passes", 2))),
        wavelength_m=max(resolution * 2, float(source.get("wavelength_m", 18.0))),
        terrace_count=max(1, int(source.get("terrace_count", 3))),
        transition_width_m=max(0.0, float(source.get("transition_width_m", 1.0))),
        heightmap_path=heightmap_path,
        heightmap_scale_m=max(0.0, float(source.get("heightmap_scale_m", 1.0))),
        spawn_pad=spawn,
    )


def _load_zone(item: dict[str, Any], index: int) -> ZoneConfig:
    kind = str(item.get("kind", "no_mow_zone"))
    shape = str(item.get("shape", "circle"))
    if kind not in SUPPORTED_ZONE_KINDS:
        raise ValueError(f"unsupported regions.zones[{index}].kind: {kind}")
    if shape not in SUPPORTED_ZONE_SHAPES:
        raise ValueError(f"unsupported regions.zones[{index}].shape: {shape}")
    center = _pair(item.get("center_m", (0.0, 0.0)), f"regions.zones[{index}].center_m")
    radius = max(0.0, float(item.get("radius_m", 0.0)))
    size = _pair(item.get("size_m", (0.0, 0.0)), f"regions.zones[{index}].size_m")
    points = tuple(_pair(point, f"regions.zones[{index}].points_m") for point in item.get("points_m", ()))
    if shape == "circle" and radius <= 0:
        raise ValueError(f"regions.zones[{index}].radius_m must be positive")
    if shape == "rectangle" and min(size) <= 0:
        raise ValueError(f"regions.zones[{index}].size_m must be positive")
    if shape == "polygon" and len(points) < 3:
        raise ValueError(f"regions.zones[{index}].points_m needs at least three points")
    return ZoneConfig(str(item.get("name", f"zone_{index}")), kind, shape, center, radius, size, points)


def load_config(path: str | Path) -> GeneratorConfig:
    config_path = Path(path).resolve()
    raw = _mapping_from_file(config_path)
    if raw.get("schema_version", 1) != 1:
        raise ValueError(f"unsupported schema_version: {raw.get('schema_version')}")
    terrain = _load_terrain(raw.get("terrain", {}) or {}, config_path.parent)
    region_source = raw.get("regions")
    if region_source is None:
        regions = RegionConfig()
    else:
        region_source = region_source or {}
        regions = RegionConfig(
            enabled=bool(region_source.get("enabled", True)),
            boundary_buffer_m=max(0.0, float(region_source.get("boundary_buffer_m", 0.0))),
            tall_grass_band_m=max(0.0, float(region_source.get("tall_grass_band_m", 0.0))),
            bare_soil_fraction=min(1.0, max(0.0, float(region_source.get("bare_soil_fraction", 0.0)))),
            bare_soil_patch_count=max(0, int(region_source.get("bare_soil_patch_count", 0))),
            zones=tuple(_load_zone(item, index) for index, item in enumerate(region_source.get("zones", ()))),
        )
    assets = tuple(
        AssetConfig(
            name=str(item["name"]),
            density_per_100m2=max(0.0, float(item.get("density_per_100m2", 0))),
            scale_min=float(item.get("scale_min", 0.9)),
            scale_max=float(item.get("scale_max", 1.1)),
            allowed_masks=tuple(str(v) for v in item.get("allowed_masks", ())),
            forbidden_masks=tuple(str(v) for v in item.get("forbidden_masks", ())),
            clearance_m=max(0.0, float(item.get("clearance_m", 0.0))),
            semantic_label=str(item.get("semantic_label", "")),
            lidar_visible=bool(item.get("lidar_visible", True)),
            align_to_surface=bool(item.get("align_to_surface", False)),
        )
        for item in raw.get("assets", ())
    )
    if any(item.scale_min <= 0 or item.scale_max < item.scale_min for item in assets):
        raise ValueError("asset scale ranges must be positive and ordered")
    manifest = tuple(
        AssetManifestEntry(str(item["name"]), str(item["path"]), str(item.get("kind", "object")), float(item.get("base_scale", 1.0)))
        for item in raw.get("asset_manifest", ())
    )
    objects = tuple(
        BusinessObjectConfig(
            name=str(item["name"]),
            category=str(item["category"]),
            shape=str(item.get("shape", "box")),
            position_m=_triple(item.get("position_m", (0.0, 0.0, 0.0)), f"objects[{index}].position_m"),
            size_m=_triple(item.get("size_m", (1.0, 1.0, 1.0)), f"objects[{index}].size_m", positive=True),
            yaw_deg=float(item.get("yaw_deg", 0.0)),
            semantic_label=str(item.get("semantic_label", item["category"])),
            collision=bool(item.get("collision", True)),
            asset_name=str(item.get("asset_name", "")),
        )
        for index, item in enumerate(raw.get("objects", ()))
    )
    if any(item.shape not in SUPPORTED_OBJECT_SHAPES for item in objects):
        raise ValueError(f"object shapes must be one of {sorted(SUPPORTED_OBJECT_SHAPES)}")
    parent = str(raw.get("parent_path", "/World/GeneratedForest"))
    if not parent.startswith("/") or parent == "/":
        raise ValueError("parent_path must be an absolute non-root prim path")
    output_source = raw.get("output", {}) or {}
    truth_directory = str(output_source.get("truth_directory", ""))
    if truth_directory and not Path(truth_directory).is_absolute():
        truth_directory = str((config_path.parent / truth_directory).resolve())
    surface_source = raw.get("surface", {}) or {}
    diffuse_color_value = tuple(float(v) for v in surface_source.get("diffuse_color", (0.12, 0.28, 0.07)))
    if len(diffuse_color_value) != 3 or any(v < 0 or v > 1 for v in diffuse_color_value):
        raise ValueError("surface.diffuse_color must contain three values between 0 and 1")
    surface = SurfaceConfig(
        diffuse_texture=str(surface_source.get("diffuse_texture", "")),
        normal_texture=str(surface_source.get("normal_texture", "")),
        roughness_texture=str(surface_source.get("roughness_texture", "")),
        ao_texture=str(surface_source.get("ao_texture", "")),
        project_uvw=bool(surface_source.get("project_uvw", True)),
        texture_scale=max(0.001, float(surface_source.get("texture_scale", 1.0))),
        diffuse_color=(diffuse_color_value[0], diffuse_color_value[1], diffuse_color_value[2]),
    )
    lighting_source = raw.get("lighting", {}) or {}
    lighting = LightingConfig(
        enabled=bool(lighting_source.get("enabled", False)),
        hdri_path=str(lighting_source.get("hdri_path", "")),
        dome_intensity=max(0.0, float(lighting_source.get("dome_intensity", 1000.0))),
        dome_rotation_deg=_triple(lighting_source.get("dome_rotation_deg", (0.0, 0.0, 0.0)), "lighting.dome_rotation_deg"),
        sun_intensity=max(0.0, float(lighting_source.get("sun_intensity", 2500.0))),
        sun_angle_deg=max(0.0, float(lighting_source.get("sun_angle_deg", 1.0))),
        sun_rotation_deg=_triple(lighting_source.get("sun_rotation_deg", (35.0, -25.0, 25.0)), "lighting.sun_rotation_deg"),
        color_temperature_k=max(1000.0, float(lighting_source.get("color_temperature_k", 5500.0))),
    )
    return GeneratorConfig(
        schema_version=1,
        seed=int(raw.get("global_seed", raw.get("seed", 1))),
        parent_path=parent,
        asset_root=str(raw.get("asset_root", "")),
        scene_name=str(raw.get("scene_name", "generated_lawn")),
        preset=str((raw.get("scene", {}) or {}).get("preset", raw.get("preset", ""))),
        terrain=terrain,
        regions=regions,
        assets=assets,
        asset_manifest=manifest,
        objects=objects,
        output=OutputConfig(truth_directory),
        surface=surface,
        lighting=lighting,
    )
