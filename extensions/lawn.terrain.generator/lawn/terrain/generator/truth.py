"""Export navigation truth alongside the generated USD."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .assets import AssetRecord
from .compute import GeneratedArrays
from .config import GeneratorConfig, ZoneConfig


SEMANTIC_LABELS = (
    "terrain_grass",
    "terrain_soil",
    "vegetation_short_grass",
    "vegetation_tall_grass",
    "tree_trunk",
    "bush",
    "rock",
    "fence",
    "flower_bed",
    "charging_station",
    "mowable_area",
    "no_mow_zone",
)


def _ring(zone: ZoneConfig) -> list[list[float]]:
    if zone.shape == "polygon":
        points = [[float(x), float(y)] for x, y in zone.points_m]
    elif zone.shape == "rectangle":
        x, y = zone.center_m
        hx, hy = zone.size_m[0] / 2, zone.size_m[1] / 2
        points = [[x - hx, y - hy], [x + hx, y - hy], [x + hx, y + hy], [x - hx, y + hy]]
    else:
        angles = np.linspace(0, 2 * np.pi, 33)[:-1]
        points = [[zone.center_m[0] + zone.radius_m * np.cos(a), zone.center_m[1] + zone.radius_m * np.sin(a)] for a in angles]
    return points + [points[0]]


def _work_areas(config: GeneratorConfig) -> dict[str, Any]:
    sx, sy = config.terrain.size_m
    inset = config.regions.boundary_buffer_m
    outer = [[-sx / 2 + inset, -sy / 2 + inset], [sx / 2 - inset, -sy / 2 + inset], [sx / 2 - inset, sy / 2 - inset], [-sx / 2 + inset, sy / 2 - inset]]
    outer.append(outer[0])
    holes = [_ring(zone) for zone in config.regions.zones if zone.kind in {"no_mow_zone", "spawn_pad"}]
    no_mow_features = [
        {"type": "Feature", "properties": {"name": zone.name, "label": zone.kind}, "geometry": {"type": "Polygon", "coordinates": [_ring(zone)]}}
        for zone in config.regions.zones if zone.kind == "no_mow_zone"
    ]
    return {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {"name": "mowable_area", "label": "mowable_area"}, "geometry": {"type": "Polygon", "coordinates": [outer, *holes]}},
            *no_mow_features,
        ],
    }


def _asset_manifest(records: dict[str, AssetRecord]) -> list[dict[str, Any]]:
    entries = []
    for name, record in sorted(records.items()):
        digest = ""
        if record.available:
            hasher = hashlib.sha256()
            with record.path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    hasher.update(chunk)
            digest = hasher.hexdigest()
        entries.append({"name": name, "path": str(record.path), "available": record.available, "kind": record.kind, "sha256": digest})
    return entries


def export_truth(
    directory: str | Path,
    config: GeneratorConfig,
    arrays: GeneratedArrays,
    asset_records: dict[str, AssetRecord],
) -> Path:
    output = Path(directory).resolve()
    output.mkdir(parents=True, exist_ok=True)
    np.save(output / "elevation.npy", arrays.heights, allow_pickle=False)
    np.save(output / "slope_deg.npy", arrays.slope_deg, allow_pickle=False)
    np.savez_compressed(output / "region_masks.npz", **arrays.masks)
    image = np.flipud(arrays.occupancy.T)
    with (output / "occupancy.pgm").open("wb") as stream:
        stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
        stream.write(np.where(image >= 100, 0, 254).astype(np.uint8).tobytes())
    (output / "occupancy.yaml").write_text(
        "image: occupancy.pgm\n"
        f"resolution: {config.terrain.resolution_m}\n"
        f"origin: [{-config.terrain.size_m[0] / 2}, {-config.terrain.size_m[1] / 2}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    (output / "work_areas.geojson").write_text(json.dumps(_work_areas(config), indent=2), encoding="utf-8")
    (output / "semantics.json").write_text(json.dumps({"labels": SEMANTIC_LABELS}, indent=2), encoding="utf-8")
    manifest = {
        "schema_version": config.schema_version,
        "generator_version": "3.0.0-autoware.1",
        "scene_name": config.scene_name,
        "preset": config.preset,
        "seed": config.seed,
        "terrain_mode": config.terrain.mode,
        "grid_shape": list(arrays.heights.shape),
        "resolution_m": config.terrain.resolution_m,
        "height_sha256": hashlib.sha256(arrays.heights.tobytes()).hexdigest(),
        "recommended_spawn_pose": list(arrays.recommended_spawn_pose),
        "assets": _asset_manifest(asset_records),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return output
