"""Scene-lighting safeguards shared by launch and preview tools."""

from __future__ import annotations

from typing import Any, Mapping, Optional


def ensure_fallback_lighting(
    stage: Any,
    environment_config: Optional[Mapping[str, Any]] = None,
) -> Optional[str]:
    """Create a fallback sun only when the composed stage has no active light."""
    config = dict((environment_config or {}).get("fallback_lighting", {}) or {})
    if not bool(config.get("enabled", True)):
        print("[Lighting] Fallback lighting disabled by configuration.")
        return None

    authored_lights = [
        prim.GetPath().pathString
        for prim in stage.Traverse()
        if prim.IsActive() and str(prim.GetTypeName()).endswith("Light")
    ]
    if authored_lights:
        print(f"[Lighting] Using authored scene light(s): {', '.join(authored_lights)}")
        return None

    from pxr import Gf, UsdGeom, UsdLux

    path = str(config.get("path", "/World/FallbackSun"))
    intensity = max(0.0, float(config.get("intensity", 2200.0)))
    angle = max(0.0, float(config.get("angle_deg", 1.0)))
    temperature = max(1000.0, float(config.get("color_temperature_k", 5500.0)))
    rotation = config.get("rotation_deg", [35.0, -25.0, 25.0])
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 3:
        raise ValueError("environment.fallback_lighting.rotation_deg must contain three values")

    sun = UsdLux.DistantLight.Define(stage, path)
    sun.CreateIntensityAttr(intensity)
    sun.CreateAngleAttr(angle)
    sun.CreateEnableColorTemperatureAttr(True)
    sun.CreateColorTemperatureAttr(temperature)
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(
        Gf.Vec3f(*(float(value) for value in rotation))
    )
    print(
        f"[Lighting] No authored lights found; created fallback sun at {path} "
        f"(intensity={intensity}, rotation={list(rotation)})."
    )
    return path
