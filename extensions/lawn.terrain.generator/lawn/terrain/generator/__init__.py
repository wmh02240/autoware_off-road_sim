"""Isaac Sim 6.x lawn terrain generator extension."""

# Keep config/compute importable in ordinary Python for deterministic unit tests.
# Kit has these modules available and imports the extension class normally.
try:
    from .extension import LawnTerrainGeneratorExtension
except ModuleNotFoundError as exc:
    if not (exc.name in {"carb", "pxr"} or (exc.name or "").startswith("omni")):
        raise
    LawnTerrainGeneratorExtension = None

__all__ = ["LawnTerrainGeneratorExtension"]
