# Changelog

## 2.0.0-autoware.1

- Ported extension discovery, menu lifecycle and stage handling to Isaac Sim 6.x.
- Added configurable external asset roots and startup diagnostics.
- Added deterministic config-driven and headless generation.
- Replaced the runtime `perlin_noise` dependency with deterministic NumPy fBm.
- Removed generator-owned `PhysicsScene` and per-tree triangle-mesh colliders.
- Replaced the prototype visibility workaround with standard PointInstancer prototypes.
