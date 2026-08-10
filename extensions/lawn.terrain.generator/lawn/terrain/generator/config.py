from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TerrainConfig:
    size_m: tuple[float, float] = (20.0, 20.0)
    resolution_m: float = 0.5
    elevation_delta_m: float = 1.0
    octaves: int = 4
    persistence: float = 0.4
    lacunarity: float = 2.0


@dataclass(frozen=True)
class AssetConfig:
    name: str
    density_per_100m2: float
    scale_min: float = 0.9
    scale_max: float = 1.1


@dataclass(frozen=True)
class GeneratorConfig:
    schema_version: int = 1
    seed: int = 1
    parent_path: str = "/World/GeneratedForest"
    asset_root: str = ""
    terrain: TerrainConfig = field(default_factory=TerrainConfig)
    assets: tuple[AssetConfig, ...] = ()


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


def load_config(path: str | Path) -> GeneratorConfig:
    raw = _mapping_from_file(Path(path))
    if raw.get("schema_version", 1) != 1:
        raise ValueError(f"unsupported schema_version: {raw.get('schema_version')}")
    source = raw.get("terrain", {})
    size = tuple(float(v) for v in source.get("size_m", (20.0, 20.0)))
    if len(size) != 2 or min(size) <= 0:
        raise ValueError("terrain.size_m must contain two positive values")
    resolution = float(source.get("resolution_m", 0.5))
    if resolution <= 0:
        raise ValueError("terrain.resolution_m must be positive")
    terrain = TerrainConfig(
        (size[0], size[1]), resolution,
        max(0.0, float(source.get("elevation_delta_m", 1.0))),
        max(1, int(source.get("octaves", 4))),
        float(source.get("persistence", 0.4)),
        max(1.0, float(source.get("lacunarity", 2.0))),
    )
    assets = tuple(AssetConfig(str(item["name"]), max(0.0, float(item.get("density_per_100m2", 0))), float(item.get("scale_min", 0.9)), float(item.get("scale_max", 1.1))) for item in raw.get("assets", []))
    if any(item.scale_min <= 0 or item.scale_max < item.scale_min for item in assets):
        raise ValueError("asset scale ranges must be positive and ordered")
    parent = str(raw.get("parent_path", "/World/GeneratedForest"))
    if not parent.startswith("/") or parent == "/":
        raise ValueError("parent_path must be an absolute non-root prim path")
    return GeneratorConfig(1, int(raw.get("seed", 1)), parent, str(raw.get("asset_root", "")), terrain, assets)

