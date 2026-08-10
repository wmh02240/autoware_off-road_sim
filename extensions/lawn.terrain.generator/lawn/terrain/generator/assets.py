from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from .constants import ASSET_REGISTRY, ASSET_ROOT_ENV


@dataclass(frozen=True)
class AssetRecord:
    name: str
    path: Path
    available: bool
    kind: str
    base_scale: float


def discover_asset_root(extension_path: str, configured: str = "") -> Path:
    extension = Path(extension_path).resolve()
    candidates = [configured, os.environ.get(ASSET_ROOT_ENV, "")]
    if len(extension.parents) >= 2:
        candidates.append(str(extension.parents[1] / "external_assets/lariad_offroad_nav/terrain.generator/data"))
    for candidate in candidates:
        if candidate and Path(candidate).expanduser().is_dir():
            return Path(candidate).expanduser().resolve()
    return Path(configured or candidates[-1]).expanduser().resolve()


def audit_assets(asset_root: str | Path) -> dict[str, AssetRecord]:
    root = Path(asset_root)
    return {
        name: AssetRecord(name, (root / item["file"]).resolve(), (root / item["file"]).is_file(), str(item["kind"]), float(item.get("base_scale", 1.0)))
        for name, item in ASSET_REGISTRY.items()
    }

