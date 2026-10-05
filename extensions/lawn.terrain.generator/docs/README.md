# Lawn Terrain Generator

Isaac Sim 6.x adaptation of LARIAD `terrain.generator` 2.0.0. Open it from
**Window > Lawn Terrain Generator**. Configure the upstream data directory via
the window, `--/exts/lawn.terrain.generator/assetRoot=...`, or the
`LARIAD_TERRAIN_ASSET_ROOT` environment variable.

The window retains the upstream generator's interactive workflow with a
scrolling, collapsible editor for general, terrain, work-region, asset, ground
surface, lighting and navigation-truth settings. **Load Config** populates all
controls from YAML/JSON, **Save Config** validates and writes the edited values,
and **Generate All** uses the current controls without requiring a save first.
The **Config** field includes a YAML/JSON file picker and **Truth Directory**
includes a directory picker. Both selected paths remain directly editable.
Polygon vertices, external asset manifests and business-object declarations
that do not have dedicated controls are preserved when a config is saved.

Generation never creates a `PhysicsScene`. UI-triggered array calculation runs
off the Kit main thread and all USD edits run on the Kit main thread. The
command-line generator uses the same implementation without constructing UI.

## Stage 3 mower scene contract

`scripts/configs/lawn_generator_stage3_offroad.yaml` is the reference
configuration. It demonstrates:

- `flat`, `single_slope`, `rolling_lawn`, `terraced_lawn`, and `heightmap`
  terrain modes with explicit slope, curvature, smoothing and flat spawn-pad
  constraints;
- `mowable_area`, `no_mow_zone`, `boundary_buffer`, `tall_grass_band`,
  `bare_soil_patch`, and `spawn_pad` masks;
- mask-aware vegetation density, clearance, semantics and LiDAR visibility;
- Offroad-Nav forest-floor PBR maps, autumn HDRI and a physical sun;
- a boundary forest composed from Birch, Spruce, Pine, Holly and Yew, with
  Bush, Blueberry, layered grass and moss-rock understory;
- collision-efficient optional business-object and actor-spawn stand-ins;
- external business assets declared through `asset_manifest` rather than code;
- elevation, slope, mask, occupancy, GeoJSON, semantic and asset-hash truth
  exports.

Heightmaps may be `.npy`, `.csv`, `.txt`, or an image format supported by
Pillow in the running Isaac Sim environment. Stage-2 YAML files remain
compatible and use the original deterministic fBm path when `terrain.mode` is
omitted.

Terrain parameters have mode-specific defaults. In the interactive editor,
changing **Mode** applies those defaults so that, for example, a newly selected
`single_slope` receives a non-zero requested slope instead of silently reusing
the previous mode's zero slope. Loading a YAML/JSON file still preserves every
explicit parameter in that file.
