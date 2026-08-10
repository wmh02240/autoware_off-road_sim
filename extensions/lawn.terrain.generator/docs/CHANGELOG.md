# Changelog

## 3.0.0-autoware.1

- Added five mower-oriented terrain modes and enforced slope, curvature and
  spawn-pad constraints.
- Added six deterministic spatial masks and mask-aware vegetation placement.
- Added config-driven business objects, external asset manifests and semantic
  metadata.
- Added navigation truth exports for elevation, slope, masks, occupancy,
  work-area polygons, spawn pose and asset hashes.
- Added the `offroad_lawn_01` reference preset using pinned Offroad-Nav PBR,
  HDRI, mixed forest, undergrowth and moss-rock assets while retaining the
  stage-2 deterministic configuration path.
- Added surface-normal alignment and sampling without replacement to reduce
  visible repetition and overlap.
- Added a deterministic headless preview renderer for visual regression checks.

## 2.0.0-autoware.1

- Ported extension discovery, menu lifecycle and stage handling to Isaac Sim 6.x.
- Added configurable external asset roots and startup diagnostics.
- Added deterministic config-driven and headless generation.
- Replaced the runtime `perlin_noise` dependency with deterministic NumPy fBm.
- Removed generator-owned `PhysicsScene` and per-tree triangle-mesh colliders.
- Replaced the prototype visibility workaround with standard PointInstancer prototypes.
