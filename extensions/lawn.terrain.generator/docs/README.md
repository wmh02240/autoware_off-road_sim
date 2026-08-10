# Lawn Terrain Generator

Isaac Sim 6.x adaptation of LARIAD `terrain.generator` 2.0.0. Open it from
**Window > Lawn Terrain Generator**. Configure the upstream data directory via
the window, `--/exts/lawn.terrain.generator/assetRoot=...`, or the
`LARIAD_TERRAIN_ASSET_ROOT` environment variable.

Generation never creates a `PhysicsScene`. UI-triggered array calculation runs
off the Kit main thread and all USD edits run on the Kit main thread. The
command-line generator uses the same implementation without constructing UI.

