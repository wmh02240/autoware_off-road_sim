"""Pure array computation; this module never imports or writes USD."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import GeneratorConfig


@dataclass(frozen=True)
class GeneratedArrays:
    heights: np.ndarray
    vertices: np.ndarray
    face_indices: np.ndarray
    placements: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]


def _heightfield(config: GeneratorConfig) -> np.ndarray:
    terrain = config.terrain
    nx = max(2, round(terrain.size_m[0] / terrain.resolution_m) + 1)
    ny = max(2, round(terrain.size_m[1] / terrain.resolution_m) + 1)
    if terrain.elevation_delta_m == 0:
        return np.zeros((nx, ny), dtype=np.float32)

    # Smooth value-noise fBm implemented locally with NumPy. Keeping this pure
    # avoids an online/runtime dependency on the upstream noise package
    # package while retaining coherent, fixed-seed terrain generation.
    rng = np.random.default_rng(config.seed)
    values = np.zeros((nx, ny), dtype=np.float64)
    amplitude, frequency, amplitude_sum = 1.0, 1.0, 0.0
    for _ in range(terrain.octaves):
        cells = max(1, int(np.ceil(frequency)))
        lattice = rng.uniform(-1.0, 1.0, size=(cells + 1, cells + 1))
        x_coord = np.linspace(0.0, frequency, nx)
        y_coord = np.linspace(0.0, frequency, ny)
        x0 = np.minimum(np.floor(x_coord).astype(np.int32), cells - 1)
        y0 = np.minimum(np.floor(y_coord).astype(np.int32), cells - 1)
        tx = x_coord - x0
        ty = y_coord - y0
        # Quintic fade has zero first/second derivatives at cell boundaries.
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
    if extent <= 1e-12:
        return np.zeros((nx, ny), dtype=np.float32)
    values = (values - float(values.min())) / extent - 0.5
    return (values * terrain.elevation_delta_m).astype(np.float32)


def _mesh(heights: np.ndarray, config: GeneratorConfig) -> tuple[np.ndarray, np.ndarray]:
    nx, ny = heights.shape
    sx, sy = config.terrain.size_m
    xx, yy = np.meshgrid(np.linspace(-sx / 2, sx / 2, nx, dtype=np.float32), np.linspace(-sy / 2, sy / 2, ny, dtype=np.float32), indexing="ij")
    vertices = np.column_stack((xx.ravel(), yy.ravel(), heights.ravel())).astype(np.float32)
    faces = np.empty(((nx - 1) * (ny - 1) * 2, 3), dtype=np.int32)
    cursor = 0
    for x in range(nx - 1):
        for y in range(ny - 1):
            index = x * ny + y
            faces[cursor] = (index, index + ny, index + ny + 1)
            faces[cursor + 1] = (index, index + ny + 1, index + 1)
            cursor += 2
    return vertices, faces


def _sample_height(heights: np.ndarray, x: np.ndarray, y: np.ndarray, config: GeneratorConfig) -> np.ndarray:
    sx, sy = config.terrain.size_m
    ix = np.clip(np.rint((x + sx / 2) / sx * (heights.shape[0] - 1)).astype(int), 0, heights.shape[0] - 1)
    iy = np.clip(np.rint((y + sy / 2) / sy * (heights.shape[1] - 1)).astype(int), 0, heights.shape[1] - 1)
    return heights[ix, iy]


def generate_arrays(config: GeneratorConfig, available_assets: set[str] | None = None) -> GeneratedArrays:
    heights = _heightfield(config)
    vertices, faces = _mesh(heights, config)
    rng = np.random.default_rng(config.seed)
    placements = {}
    area = config.terrain.size_m[0] * config.terrain.size_m[1]
    for asset in config.assets:
        if available_assets is not None and asset.name not in available_assets:
            continue
        count = int(round(asset.density_per_100m2 * area / 100.0))
        x = rng.uniform(-config.terrain.size_m[0] / 2, config.terrain.size_m[0] / 2, count)
        y = rng.uniform(-config.terrain.size_m[1] / 2, config.terrain.size_m[1] / 2, count)
        positions = np.column_stack((x, y, _sample_height(heights, x, y, config))).astype(np.float32)
        yaw = rng.uniform(-np.pi, np.pi, count)
        orientations = np.column_stack((np.cos(yaw / 2), np.zeros(count), np.zeros(count), np.sin(yaw / 2))).astype(np.float32)
        scales = rng.uniform(asset.scale_min, asset.scale_max, count).astype(np.float32)
        placements[asset.name] = (positions, orientations, scales)
    return GeneratedArrays(heights, vertices, faces, placements)
