"""Generation orchestration used by both extension UI and headless mode."""

from __future__ import annotations

from .assets import audit_assets
from .compute import GeneratedArrays, generate_arrays
from .config import GeneratorConfig
from .usd_writer import UsdSceneWriter


class GeneratorController:
    def __init__(self, stage, asset_root: str):
        self.asset_root = asset_root
        self.assets = audit_assets(asset_root)
        self.writer = UsdSceneWriter(stage, asset_root)

    @property
    def missing_assets(self) -> tuple[str, ...]:
        return tuple(name for name, record in self.assets.items() if not record.available)

    def compute(self, config: GeneratorConfig) -> GeneratedArrays:
        self.assets = audit_assets(self.asset_root, config.asset_manifest)
        available = {name for name, record in self.assets.items() if record.available}
        return generate_arrays(config, available)

    def apply(self, config: GeneratorConfig, arrays: GeneratedArrays) -> dict[str, int]:
        return self.writer.apply(config, arrays, self.assets)

    def generate(self, config: GeneratorConfig) -> dict[str, int]:
        return self.apply(config, self.compute(config))

    def clear(self, parent_path: str) -> None:
        self.writer.clear(parent_path)
