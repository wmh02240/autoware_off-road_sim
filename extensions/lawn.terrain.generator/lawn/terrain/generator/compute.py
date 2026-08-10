"""Pure array computation; this module never imports or writes USD."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import GeneratorConfig, TerrainConfig, ZoneConfig


MASK_NAMES = (
    "mowable_area",
    "no_mow_zone",
    "boundary_buffer",
    "tall_grass_band",
    "bare_soil_patch",
    "spawn_pad",
)


@dataclass(frozen=True)
class GeneratedArrays:
    heights: np.ndarray
    vertices: np.ndarray
    face_indices: np.ndarray
    placements: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]
    slope_deg: np.ndarray
    masks: dict[str, np.ndarray]
    occupancy: np.ndarray
    recommended_spawn_pose: tuple[float, float, float, float]


def _grid(terrain: TerrainConfig) -> tuple[np.ndarray, np.ndarray]:
    nx = max(2, round(terrain.size_m[0] / terrain.resolution_m) + 1)
    ny = max(2, round(terrain.size_m[1] / terrain.resolution_m) + 1)
    return np.meshgrid(
        np.linspace(-terrain.size_m[0] / 2, terrain.size_m[0] / 2, nx, dtype=np.float64),
        np.linspace(-terrain.size_m[1] / 2, terrain.size_m[1] / 2, ny, dtype=np.float64),
        indexing="ij",
    )


def _fbm(shape: tuple[int, int], config: GeneratorConfig) -> np.ndarray:
    terrain = config.terrain
    rng = np.random.default_rng(config.seed)
    values = np.zeros(shape, dtype=np.float64)
    amplitude, frequency, amplitude_sum = 1.0, 1.0, 0.0
    for _ in range(terrain.octaves):
        cells = max(1, int(np.ceil(frequency)))
        lattice = rng.uniform(-1.0, 1.0, size=(cells + 1, cells + 1))
        x_coord = np.linspace(0.0, frequency, shape[0])
        y_coord = np.linspace(0.0, frequency, shape[1])
        x0 = np.minimum(np.floor(x_coord).astype(np.int32), cells - 1)
        y0 = np.minimum(np.floor(y_coord).astype(np.int32), cells - 1)
        tx, ty = x_coord - x0, y_coord - y0
        tx = tx * tx * tx * (tx * (tx * 6.0 - 15.0) + 10.0)
        ty = ty * ty * ty * (ty * (ty * 6.0 - 15.0) + 10.0)
        lower = lattice[x0[:, None], y0[None, :]] * (1.0 - tx[:, None]) + lattice[x0[:, None] + 1, y0[None, :]] * tx[:, None]
        upper = lattice[x0[:, None], y0[None, :] + 1] * (1.0 - tx[:, None]) + lattice[x0[:, None] + 1, y0[None, :] + 1] * tx[:, None]
        values += amplitude * (lower * (1.0 - ty[None, :]) + upper * ty[None, :])
        amplitude_sum += amplitude
        amplitude *= terrain.persistence
        frequency *= terrain.lacunarity
    values /= max(amplitude_sum, 1e-12)
    extent = float(np.ptp(values))
    return np.zeros(shape, dtype=np.float64) if extent <= 1e-12 else (values - float(values.min())) / extent - 0.5


def _resize_bilinear(source: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    x_old = np.arange(source.shape[0], dtype=np.float64)
    y_old = np.arange(source.shape[1], dtype=np.float64)
    x_new = np.linspace(0, source.shape[0] - 1, shape[0])
    y_new = np.linspace(0, source.shape[1] - 1, shape[1])
    along_x = np.stack([np.interp(x_new, x_old, source[:, column]) for column in range(source.shape[1])], axis=1)
    return np.stack([np.interp(y_new, y_old, along_x[row, :]) for row in range(shape[0])], axis=0)


def _load_heightmap(terrain: TerrainConfig, shape: tuple[int, int]) -> np.ndarray:
    path = Path(terrain.heightmap_path)
    if not path.is_file():
        raise ValueError(f"terrain heightmap does not exist: {path}")
    if path.suffix.lower() == ".npy":
        source = np.load(path, allow_pickle=False)
    elif path.suffix.lower() in {".csv", ".txt"}:
        source = np.loadtxt(path, delimiter="," if path.suffix.lower() == ".csv" else None)
    else:
        try:
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError("Pillow is required for image heightmaps; use .npy or .csv otherwise") from exc
        source = np.asarray(Image.open(path).convert("F"), dtype=np.float64)
    if source.ndim != 2 or min(source.shape) < 2 or not np.isfinite(source).all():
        raise ValueError("heightmap must be a finite two-dimensional array of at least 2x2")
    source = _resize_bilinear(source.astype(np.float64), shape)
    extent = float(np.ptp(source))
    return np.zeros(shape, dtype=np.float64) if extent <= 1e-12 else ((source - float(source.min())) / extent - 0.5) * terrain.heightmap_scale_m


def _smooth(values: np.ndarray, passes: int) -> np.ndarray:
    result = values.copy()
    for _ in range(passes):
        padded = np.pad(result, 1, mode="edge")
        result = (
            padded[1:-1, 1:-1] * 4.0
            + padded[:-2, 1:-1]
            + padded[2:, 1:-1]
            + padded[1:-1, :-2]
            + padded[1:-1, 2:]
        ) / 8.0
    return result


def _flatten_pad(values: np.ndarray, xx: np.ndarray, yy: np.ndarray, config: GeneratorConfig) -> np.ndarray:
    pad = config.terrain.spawn_pad
    if pad.radius_m <= 0:
        return values
    distance = np.hypot(xx - pad.center_m[0], yy - pad.center_m[1])
    inner = distance <= pad.radius_m
    if not np.any(inner):
        raise ValueError("terrain.spawn_pad does not intersect the terrain")
    target = float(np.mean(values[inner]))
    blend_width = max(config.terrain.resolution_m * 2.0, pad.radius_m * 0.4)
    blend = np.clip((pad.radius_m + blend_width - distance) / blend_width, 0.0, 1.0)
    blend[inner] = 1.0
    blend = blend * blend * (3.0 - 2.0 * blend)
    return values * (1.0 - blend) + target * blend


def _slope(values: np.ndarray, resolution: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gx, gy = np.gradient(values, resolution, resolution)
    return np.degrees(np.arctan(np.hypot(gx, gy))), gx, gy


def _enforce_constraints(values: np.ndarray, xx: np.ndarray, yy: np.ndarray, config: GeneratorConfig) -> np.ndarray:
    terrain = config.terrain
    result = _smooth(values, terrain.smoothing_passes)
    if terrain.max_local_curvature_per_m > 0:
        for _ in range(12):
            gx, gy = np.gradient(result, terrain.resolution_m, terrain.resolution_m)
            gxx = np.gradient(gx, terrain.resolution_m, axis=0)
            gyy = np.gradient(gy, terrain.resolution_m, axis=1)
            if float(np.max(np.abs(gxx) + np.abs(gyy))) <= terrain.max_local_curvature_per_m + 1e-9:
                break
            result = _smooth(result, 1)
    result = _flatten_pad(result, xx, yy, config)
    maximum = min(terrain.max_slope_deg, terrain.max_cross_slope_deg)
    slopes, _, _ = _slope(result, terrain.resolution_m)
    observed = float(slopes.max())
    if observed > maximum:
        center = float(np.mean(result))
        gradient_limit = np.tan(np.radians(maximum))
        observed_gradient = np.tan(np.radians(observed))
        result = center + (result - center) * (gradient_limit / observed_gradient) * 0.999
    return result


def _heightfield(config: GeneratorConfig) -> np.ndarray:
    terrain = config.terrain
    xx, yy = _grid(terrain)
    mode = terrain.mode
    noise = _fbm(xx.shape, config)
    if mode == "legacy_fbm":
        return (noise * terrain.elevation_delta_m).astype(np.float32)
    if mode == "flat":
        values = terrain.base_height_m + noise * terrain.elevation_delta_m
    elif mode == "single_slope":
        angle = np.radians(terrain.slope_direction_deg)
        axis = xx * np.cos(angle) + yy * np.sin(angle)
        values = terrain.base_height_m + axis * np.tan(np.radians(terrain.slope_deg))
        values += noise * terrain.elevation_delta_m
    elif mode == "rolling_lawn":
        angle = np.radians(terrain.slope_direction_deg)
        primary = xx * np.cos(angle) + yy * np.sin(angle)
        secondary = -xx * np.sin(angle) + yy * np.cos(angle)
        wave = np.sin(2 * np.pi * primary / terrain.wavelength_m) * np.cos(np.pi * secondary / terrain.wavelength_m)
        values = terrain.base_height_m + terrain.elevation_delta_m * (0.65 * noise + 0.35 * wave)
    elif mode == "terraced_lawn":
        angle = np.radians(terrain.slope_direction_deg)
        axis = xx * np.cos(angle) + yy * np.sin(angle)
        normalized = (axis - float(axis.min())) / max(float(np.ptp(axis)), 1e-12)
        stepped = np.floor(np.minimum(normalized, 1.0 - 1e-9) * terrain.terrace_count) / max(terrain.terrace_count - 1, 1)
        values = terrain.base_height_m + (stepped - 0.5) * terrain.elevation_delta_m
        transition_passes = max(terrain.smoothing_passes, round(terrain.transition_width_m / terrain.resolution_m))
        values = _smooth(values, transition_passes)
    else:
        values = terrain.base_height_m + _load_heightmap(terrain, xx.shape)
    return _enforce_constraints(values, xx, yy, config).astype(np.float32)


def _mesh(heights: np.ndarray, config: GeneratorConfig) -> tuple[np.ndarray, np.ndarray]:
    xx, yy = _grid(config.terrain)
    vertices = np.column_stack((xx.ravel(), yy.ravel(), heights.ravel())).astype(np.float32)
    nx, ny = heights.shape
    faces = np.empty(((nx - 1) * (ny - 1) * 2, 3), dtype=np.int32)
    cursor = 0
    for x in range(nx - 1):
        for y in range(ny - 1):
            index = x * ny + y
            faces[cursor] = (index, index + ny, index + ny + 1)
            faces[cursor + 1] = (index, index + ny + 1, index + 1)
            cursor += 2
    return vertices, faces


def _zone_mask(zone: ZoneConfig, xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
    if zone.shape == "circle":
        return np.hypot(xx - zone.center_m[0], yy - zone.center_m[1]) <= zone.radius_m
    if zone.shape == "rectangle":
        return (np.abs(xx - zone.center_m[0]) <= zone.size_m[0] / 2) & (np.abs(yy - zone.center_m[1]) <= zone.size_m[1] / 2)
    points = np.asarray(zone.points_m, dtype=np.float64)
    inside = np.zeros(xx.shape, dtype=bool)
    previous = len(points) - 1
    for current in range(len(points)):
        xi, yi = points[current]
        xj, yj = points[previous]
        crosses = ((yi > yy) != (yj > yy)) & (xx < (xj - xi) * (yy - yi) / (yj - yi + 1e-15) + xi)
        inside ^= crosses
        previous = current
    return inside


def _masks(config: GeneratorConfig, xx: np.ndarray, yy: np.ndarray) -> dict[str, np.ndarray]:
    shape = xx.shape
    if not config.regions.enabled:
        return {name: (np.ones(shape, dtype=bool) if name == "mowable_area" else np.zeros(shape, dtype=bool)) for name in MASK_NAMES}
    sx, sy = config.terrain.size_m
    edge_distance = np.minimum(sx / 2 - np.abs(xx), sy / 2 - np.abs(yy))
    boundary = edge_distance <= config.regions.boundary_buffer_m
    no_mow = np.zeros(shape, dtype=bool)
    bare = np.zeros(shape, dtype=bool)
    spawn = np.zeros(shape, dtype=bool)
    for zone in config.regions.zones:
        target = _zone_mask(zone, xx, yy)
        if zone.kind == "no_mow_zone":
            no_mow |= target
        elif zone.kind == "bare_soil_patch":
            bare |= target
        else:
            spawn |= target
    pad = config.terrain.spawn_pad
    if pad.radius_m > 0:
        spawn |= np.hypot(xx - pad.center_m[0], yy - pad.center_m[1]) <= pad.radius_m
    if config.regions.bare_soil_fraction > 0 and config.regions.bare_soil_patch_count > 0:
        rng = np.random.default_rng(config.seed + 1701)
        target_area = sx * sy * config.regions.bare_soil_fraction
        radius = np.sqrt(target_area / (np.pi * config.regions.bare_soil_patch_count))
        for _ in range(config.regions.bare_soil_patch_count):
            center_x = rng.uniform(-sx / 2 + radius, sx / 2 - radius)
            center_y = rng.uniform(-sy / 2 + radius, sy / 2 - radius)
            bare |= np.hypot(xx - center_x, yy - center_y) <= radius
    tall = (edge_distance <= config.regions.boundary_buffer_m + config.regions.tall_grass_band_m) & ~boundary
    tall |= no_mow
    mowable = ~(boundary | no_mow | spawn)
    bare &= mowable
    return {
        "mowable_area": mowable,
        "no_mow_zone": no_mow,
        "boundary_buffer": boundary,
        "tall_grass_band": tall,
        "bare_soil_patch": bare,
        "spawn_pad": spawn,
    }


def _erode(mask: np.ndarray, cells: int) -> np.ndarray:
    if cells <= 0:
        return mask
    result = mask.copy()
    for dx in range(-cells, cells + 1):
        for dy in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells:
                continue
            shifted = np.zeros_like(mask)
            source_x = slice(max(0, -dx), min(mask.shape[0], mask.shape[0] - dx))
            source_y = slice(max(0, -dy), min(mask.shape[1], mask.shape[1] - dy))
            target_x = slice(max(0, dx), min(mask.shape[0], mask.shape[0] + dx))
            target_y = slice(max(0, dy), min(mask.shape[1], mask.shape[1] + dy))
            shifted[target_x, target_y] = mask[source_x, source_y]
            result &= shifted
    return result


def _sample_height(heights: np.ndarray, x: np.ndarray, y: np.ndarray, config: GeneratorConfig) -> np.ndarray:
    sx, sy = config.terrain.size_m
    ix = np.clip(np.rint((x + sx / 2) / sx * (heights.shape[0] - 1)).astype(int), 0, heights.shape[0] - 1)
    iy = np.clip(np.rint((y + sy / 2) / sy * (heights.shape[1] - 1)).astype(int), 0, heights.shape[1] - 1)
    return heights[ix, iy]


def _surface_orientations(
    gx: np.ndarray,
    gy: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    config: GeneratorConfig,
) -> np.ndarray:
    sx, sy = config.terrain.size_m
    ix = np.clip(np.rint((x + sx / 2) / sx * (gx.shape[0] - 1)).astype(int), 0, gx.shape[0] - 1)
    iy = np.clip(np.rint((y + sy / 2) / sy * (gx.shape[1] - 1)).astype(int), 0, gx.shape[1] - 1)
    nx, ny, nz = -gx[ix, iy], -gy[ix, iy], np.ones(len(x), dtype=np.float64)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / norm, ny / norm, nz / norm
    align_w = np.sqrt(np.maximum(0.0, (1.0 + nz) / 2.0))
    denominator = np.maximum(2.0 * align_w, 1e-12)
    align_x, align_y = -ny / denominator, nx / denominator
    half = yaw / 2.0
    cos_yaw, sin_yaw = np.cos(half), np.sin(half)
    return np.column_stack((
        align_w * cos_yaw,
        align_x * cos_yaw + align_y * sin_yaw,
        -align_x * sin_yaw + align_y * cos_yaw,
        align_w * sin_yaw,
    )).astype(np.float32)


def _object_occupancy(config: GeneratorConfig, xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
    occupied = np.zeros(xx.shape, dtype=bool)
    margin = config.terrain.resolution_m / 2
    for item in config.objects:
        if not item.collision:
            continue
        dx = xx - item.position_m[0]
        dy = yy - item.position_m[1]
        angle = np.radians(-item.yaw_deg)
        local_x = dx * np.cos(angle) - dy * np.sin(angle)
        local_y = dx * np.sin(angle) + dy * np.cos(angle)
        if item.shape == "cylinder":
            occupied |= np.hypot(local_x, local_y) <= max(item.size_m[0], item.size_m[1]) / 2 + margin
        else:
            occupied |= (np.abs(local_x) <= item.size_m[0] / 2 + margin) & (np.abs(local_y) <= item.size_m[1] / 2 + margin)
    return occupied


def _placement_mask(asset, masks: dict[str, np.ndarray], object_occupancy: np.ndarray, config: GeneratorConfig) -> np.ndarray:
    valid = np.ones(next(iter(masks.values())).shape, dtype=bool)
    if asset.allowed_masks:
        valid[:] = False
        for name in asset.allowed_masks:
            if name not in masks:
                raise ValueError(f"asset {asset.name} uses unknown allowed mask: {name}")
            valid |= masks[name]
    for name in asset.forbidden_masks:
        if name not in masks:
            raise ValueError(f"asset {asset.name} uses unknown forbidden mask: {name}")
        valid &= ~masks[name]
    clearance_cells = int(np.ceil(asset.clearance_m / config.terrain.resolution_m))
    valid = _erode(valid, clearance_cells)
    if clearance_cells:
        valid &= _erode(~object_occupancy, clearance_cells)
    return valid


def generate_arrays(config: GeneratorConfig, available_assets: set[str] | None = None) -> GeneratedArrays:
    heights = _heightfield(config)
    vertices, faces = _mesh(heights, config)
    xx, yy = _grid(config.terrain)
    masks = _masks(config, xx, yy)
    slope_deg, gradient_x, gradient_y = _slope(heights, config.terrain.resolution_m)
    object_occupancy = _object_occupancy(config, xx, yy)
    occupancy = (masks["no_mow_zone"] | masks["boundary_buffer"] | object_occupancy).astype(np.uint8) * 100
    rng = np.random.default_rng(config.seed)
    placements: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    cell_area = config.terrain.resolution_m ** 2
    for asset in config.assets:
        if available_assets is not None and asset.name not in available_assets:
            continue
        valid = _placement_mask(asset, masks, object_occupancy, config)
        eligible_area = float(np.count_nonzero(valid)) * cell_area if config.regions.enabled or asset.allowed_masks or asset.forbidden_masks else config.terrain.size_m[0] * config.terrain.size_m[1]
        count = int(round(asset.density_per_100m2 * eligible_area / 100.0))
        indices = np.argwhere(valid)
        if count and not len(indices):
            count = 0
        if count:
            selected = indices[rng.choice(len(indices), size=count, replace=count > len(indices))]
        else:
            selected = np.empty((0, 2), dtype=int)
        jitter = rng.uniform(-config.terrain.resolution_m * 0.45, config.terrain.resolution_m * 0.45, size=(count, 2))
        x = xx[selected[:, 0], selected[:, 1]] + jitter[:, 0] if count else np.empty(0)
        y = yy[selected[:, 0], selected[:, 1]] + jitter[:, 1] if count else np.empty(0)
        x = np.clip(x, -config.terrain.size_m[0] / 2, config.terrain.size_m[0] / 2)
        y = np.clip(y, -config.terrain.size_m[1] / 2, config.terrain.size_m[1] / 2)
        positions = np.column_stack((x, y, _sample_height(heights, x, y, config))).astype(np.float32)
        yaw = rng.uniform(-np.pi, np.pi, count)
        if asset.align_to_surface:
            orientations = _surface_orientations(gradient_x, gradient_y, x, y, yaw, config)
        else:
            orientations = np.column_stack((np.cos(yaw / 2), np.zeros(count), np.zeros(count), np.sin(yaw / 2))).astype(np.float32)
        scales = rng.uniform(asset.scale_min, asset.scale_max, count).astype(np.float32)
        placements[asset.name] = (positions, orientations, scales)
    pad = config.terrain.spawn_pad
    spawn_x, spawn_y = pad.center_m
    spawn_z = float(_sample_height(heights, np.asarray([spawn_x]), np.asarray([spawn_y]), config)[0])
    return GeneratedArrays(heights, vertices, faces, placements, slope_deg.astype(np.float32), masks, occupancy, (spawn_x, spawn_y, spawn_z, config.terrain.slope_direction_deg))
