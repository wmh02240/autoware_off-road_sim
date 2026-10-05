"""Interactive Isaac Sim UI backed by the versioned generator configuration."""

from __future__ import annotations

import asyncio
import copy
import json
import tempfile
from pathlib import Path

import carb
import omni.kit.app
import omni.ui as ui
from omni.kit.window.filepicker import FilePickerDialog

from .config import SUPPORTED_TERRAIN_MODES, TERRAIN_MODE_DEFAULTS, load_config
from .constants import EXTENSION_NAME


class GeneratorWindow:
    """Old-style parameter editor that still uses the stage-3 controller."""

    LABEL_WIDTH = 245

    def __init__(self, extension_path: str, controller_getter, asset_root: str):
        self._extension_path = extension_path
        self._controller_getter = controller_getter
        self._asset_root = asset_root
        self._window = None
        self._task = None
        self._status = None
        self._path_picker = None
        self._config_model = None
        self._models: dict[str, ui.AbstractValueModel] = {}
        self._asset_models: list[dict[str, ui.AbstractValueModel]] = []
        self._zone_models: list[dict[str, ui.AbstractValueModel]] = []
        self._raw: dict = {}
        self._config_path = self._default_config_path()

    def _default_config_path(self) -> Path:
        # Extension Manager may report the registration symlink under
        # release/exts. Resolve it before deriving the repository root.
        extension_path = Path(self._extension_path).resolve()
        return extension_path.parents[1] / "scripts/configs/lawn_generator_stage3_offroad.yaml"

    @staticmethod
    def _read_mapping(path: Path) -> dict:
        text = path.read_text(encoding="utf-8")
        if path.suffix.lower() == ".json":
            value = json.loads(text)
        else:
            import yaml

            value = yaml.safe_load(text)
        if not isinstance(value, dict):
            raise ValueError("generator config must contain a mapping")
        return value

    @staticmethod
    def _write_mapping(path: Path, value: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix.lower() == ".json":
            path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
        else:
            import yaml

            path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")

    @staticmethod
    def _nested(source: dict, *keys, default=None):
        value = source
        for key in keys:
            if not isinstance(value, dict):
                return default
            value = value.get(key)
        return default if value is None else value

    def show(self) -> None:
        if self._window is not None:
            self._window.visible = True
            return
        try:
            self._raw = self._read_mapping(self._config_path)
        except Exception as exc:
            carb.log_error(f"[{EXTENSION_NAME}] config load failed: {exc}")
            self._raw = {}
        self._models.clear()
        self._asset_models.clear()
        self._zone_models.clear()
        self._window = ui.Window(
            EXTENSION_NAME,
            width=570,
            height=760,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._visibility_changed)
        with self._window.frame:
            with ui.VStack(spacing=6):
                with ui.VStack(height=92, spacing=4):
                    self._build_actions()
                # A separator without an explicit height is a flexible widget
                # in an omni.ui VStack and consumes the remaining vertical
                # space, pushing the scrolling editor to the bottom.
                ui.Separator(height=1)
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                ):
                    with ui.VStack(spacing=5, height=0):
                        self._build_general()
                        self._build_terrain()
                        self._build_regions()
                        self._build_assets()
                        self._build_surface()
                        self._build_lighting()
                        self._build_output()
                        ui.Spacer(height=8)

    def _build_actions(self) -> None:
        with ui.HStack(height=26, spacing=5):
            ui.Label("Config", width=60)
            self._config_model = ui.SimpleStringModel(str(self._config_path))
            ui.StringField(self._config_model)
            ui.Button("Browse…", width=72, clicked_fn=self._browse_config)
        with ui.HStack(height=30, spacing=5):
            ui.Button("Load Config", clicked_fn=self._reload)
            ui.Button("Save Config", clicked_fn=self._save)
            ui.Button("Generate All", clicked_fn=self._start_generate)
            ui.Button("Clear All", clicked_fn=self._clear)
        self._status = ui.Label("Ready", height=30, word_wrap=True)

    def _string(self, key: str, label: str, value=""):
        model = ui.SimpleStringModel(str(value))
        self._models[key] = model
        with ui.HStack(height=24):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.StringField(model)
        return model

    def _choice(self, key: str, label: str, value: str, options: tuple[str, ...]):
        selected = options.index(str(value)) if str(value) in options else 0
        with ui.HStack(height=24):
            ui.Label(label, width=self.LABEL_WIDTH)
            combo = ui.ComboBox(selected, *options)
        model = combo.model.get_item_value_model()
        self._models[key] = model
        return model

    def _float(self, key: str, label: str, value=0.0):
        model = ui.SimpleFloatModel(float(value))
        self._models[key] = model
        with ui.HStack(height=24):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.FloatField(model)
        return model

    def _int(self, key: str, label: str, value=0):
        model = ui.SimpleIntModel(int(value))
        self._models[key] = model
        with ui.HStack(height=24):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.IntField(model)
        return model

    def _bool(self, key: str, label: str, value=False):
        model = ui.SimpleBoolModel(bool(value))
        self._models[key] = model
        with ui.HStack(height=24):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.CheckBox(model=model)
        return model

    def _pair(self, key: str, label: str, value=(0.0, 0.0)):
        values = list(value or (0.0, 0.0))
        models = (ui.SimpleFloatModel(float(values[0])), ui.SimpleFloatModel(float(values[1])))
        self._models[f"{key}.0"], self._models[f"{key}.1"] = models
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.FloatField(models[0])
            ui.FloatField(models[1])
        return models

    def _triple(self, key: str, label: str, value=(0.0, 0.0, 0.0)):
        values = list(value or (0.0, 0.0, 0.0))
        models = tuple(ui.SimpleFloatModel(float(v)) for v in values)
        for index, model in enumerate(models):
            self._models[f"{key}.{index}"] = model
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=self.LABEL_WIDTH)
            for model in models:
                ui.FloatField(model)
        return models

    @staticmethod
    def _config_path_filter(item) -> bool:
        if not item or item.is_folder:
            return True
        return Path(item.path).suffix.lower() in {".yaml", ".yml", ".json"}

    def _close_path_picker(self) -> None:
        if self._path_picker is None:
            return
        picker = self._path_picker
        self._path_picker = None
        picker.hide()

        async def deferred_destroy() -> None:
            await omni.kit.app.get_app().next_update_async()
            picker.destroy()

        asyncio.ensure_future(deferred_destroy())

    def _browse_config(self) -> None:
        self._close_path_picker()

        def selected(filename: str, dirname: str) -> None:
            if filename:
                self._config_model.set_value(str(Path(dirname) / filename))
            self._close_path_picker()

        self._path_picker = FilePickerDialog(
            "Select Generator Configuration",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=selected,
            click_cancel_handler=lambda _filename, _dirname: self._close_path_picker(),
            item_filter_fn=self._config_path_filter,
            item_filter_options=["Config Files (*.yaml, *.yml, *.json)"],
            file_extension_options=[
                (".yaml", "YAML Configuration"),
                (".yml", "YAML Configuration"),
                (".json", "JSON Configuration"),
            ],
        )

    def _browse_truth_directory(self) -> None:
        self._close_path_picker()

        def selected(filename: str, dirname: str) -> None:
            directory = Path(dirname)
            selected_path = directory / filename if filename else directory
            if filename and (selected_path.is_dir() or not selected_path.suffix):
                directory = selected_path
            self._models["output.truth_directory"].set_value(str(directory))
            self._close_path_picker()

        self._path_picker = FilePickerDialog(
            "Select Navigation Truth Directory",
            allow_multi_selection=False,
            apply_button_label="Select Folder",
            click_apply_handler=selected,
            click_cancel_handler=lambda _filename, _dirname: self._close_path_picker(),
            enable_file_bar=True,
        )
        self._path_picker.set_filebar_label_name("Folder Name")

    def _path_string(self, key: str, label: str, value, browse_fn):
        model = ui.SimpleStringModel(str(value))
        self._models[key] = model
        with ui.HStack(height=24, spacing=4):
            ui.Label(label, width=self.LABEL_WIDTH)
            ui.StringField(model)
            ui.Button("Browse…", width=72, clicked_fn=browse_fn)
        return model

    def _build_general(self) -> None:
        with ui.CollapsableFrame("General Settings", collapsed=False, height=0):
            with ui.VStack(spacing=3, height=0):
                self._string("scene_name", "Scene Name", self._raw.get("scene_name", "generated_lawn"))
                self._string("preset", "Preset", self._nested(self._raw, "scene", "preset", default=""))
                self._string("parent_path", "Parent Path", self._raw.get("parent_path", "/World/GeneratedForest"))
                self._int("global_seed", "Random Seed (deterministic)", self._raw.get("global_seed", self._raw.get("seed", 1)))
                self._string("asset_root", "Asset Root", self._raw.get("asset_root", ""))
                ui.Label(
                    "The same saved parameters, asset set and random seed produce the same terrain and placements.",
                    word_wrap=True,
                )
                ui.Label(f"Discovered assets: {self._asset_root}", word_wrap=True)

    def _build_terrain(self) -> None:
        terrain = self._raw.get("terrain", {}) or {}
        spawn = terrain.get("spawn_pad", {}) or {}
        with ui.CollapsableFrame("Terrain", collapsed=False, height=0):
            with ui.VStack(spacing=3, height=0):
                mode_model = self._choice(
                    "terrain.mode",
                    "Mode",
                    terrain.get("mode", "legacy_fbm"),
                    SUPPORTED_TERRAIN_MODES,
                )
                self._pair("terrain.size_m", "Area Length / Width (m)", terrain.get("size_m", (20, 20)))
                self._float("terrain.resolution_m", "Horizontal Resolution (m)", terrain.get("resolution_m", 0.5))
                self._float("terrain.base_height_m", "Base Height (m)", terrain.get("base_height_m", 0.0))
                self._float("terrain.elevation_delta_m", "Elevation Range (m)", terrain.get("elevation_delta_m", 1.0))
                self._float("terrain.wavelength_m", "Wavelength (m)", terrain.get("wavelength_m", 18.0))
                self._int("terrain.octaves", "Noise Octaves (count)", terrain.get("octaves", 4))
                self._float("terrain.persistence", "Noise Persistence (ratio)", terrain.get("persistence", 0.4))
                self._float("terrain.lacunarity", "Noise Lacunarity (multiplier)", terrain.get("lacunarity", 2.0))
                self._float("terrain.slope_direction_deg", "Slope Direction (deg)", terrain.get("slope_direction_deg", 0.0))
                self._float("terrain.slope_deg", "Requested Slope (deg)", terrain.get("slope_deg", 0.0))
                self._float("terrain.max_slope_deg", "Max Slope (deg)", terrain.get("max_slope_deg", 15.0))
                self._float("terrain.max_cross_slope_deg", "Max Cross Slope (deg)", terrain.get("max_cross_slope_deg", 15.0))
                self._float("terrain.max_local_curvature_per_m", "Max Local Curvature (1/m)", terrain.get("max_local_curvature_per_m", 2.0))
                self._int("terrain.smoothing_passes", "Smoothing Passes (count)", terrain.get("smoothing_passes", 2))
                self._int("terrain.terrace_count", "Terrace Count (count)", terrain.get("terrace_count", 3))
                self._float("terrain.transition_width_m", "Transition Width (m)", terrain.get("transition_width_m", 1.0))
                self._string("terrain.heightmap_path", "Heightmap Path", terrain.get("heightmap_path", ""))
                self._float("terrain.heightmap_scale_m", "Heightmap Scale (m)", terrain.get("heightmap_scale_m", 1.0))
                self._pair("terrain.spawn_pad.center_m", "Spawn Pad Center (m)", spawn.get("center_m", (0, 0)))
                self._float("terrain.spawn_pad.radius_m", "Spawn Pad Radius (m)", spawn.get("radius_m", 0.0))
                mode_model.add_value_changed_fn(self._terrain_mode_changed)

    def _terrain_mode_changed(self, model) -> None:
        """Apply meaningful parameters when the user selects another mode.

        Without this, the rolling-lawn preset's zero requested slope made
        ``flat`` and ``single_slope`` generate the exact same heightfield.
        Explicit YAML values are still retained when a config is loaded; these
        defaults apply only to an interactive mode change.
        """
        index = model.get_value_as_int()
        if not 0 <= index < len(SUPPORTED_TERRAIN_MODES):
            return
        mode = SUPPORTED_TERRAIN_MODES[index]
        for name, value in TERRAIN_MODE_DEFAULTS[mode].items():
            key = f"terrain.{name}"
            target = self._models.get(key)
            if target is not None:
                target.set_value(value)
        if self._status is not None:
            self._status.text = f"Mode: {mode} (mode defaults applied)"

    def _build_regions(self) -> None:
        regions = self._raw.get("regions", {}) or {}
        with ui.CollapsableFrame("Regions and Work Areas", collapsed=True, height=0):
            with ui.VStack(spacing=3, height=0):
                self._bool("regions.enabled", "Enable Regions", regions.get("enabled", True))
                self._float("regions.boundary_buffer_m", "Boundary Buffer (m)", regions.get("boundary_buffer_m", 0.0))
                self._float("regions.tall_grass_band_m", "Tall Grass Band (m)", regions.get("tall_grass_band_m", 0.0))
                self._float("regions.bare_soil_fraction", "Bare Soil Fraction (0–1)", regions.get("bare_soil_fraction", 0.0))
                self._int("regions.bare_soil_patch_count", "Bare Soil Patches (count)", regions.get("bare_soil_patch_count", 0))
                for index, zone in enumerate(regions.get("zones", ())):
                    with ui.CollapsableFrame(f"Zone {index + 1}: {zone.get('name', '')}", collapsed=True, height=0):
                        with ui.VStack(spacing=2, height=0):
                            models = {
                                "name": self._string(f"zone.{index}.name", "Name", zone.get("name", f"zone_{index}")),
                                "kind": self._string(f"zone.{index}.kind", "Kind", zone.get("kind", "no_mow_zone")),
                                "shape": self._string(f"zone.{index}.shape", "Shape", zone.get("shape", "circle")),
                                "center": self._pair(f"zone.{index}.center", "Center (m)", zone.get("center_m", (0, 0))),
                                "radius": self._float(f"zone.{index}.radius", "Radius (m)", zone.get("radius_m", 0.0)),
                                "size": self._pair(f"zone.{index}.size", "Size (m)", zone.get("size_m", (0, 0))),
                            }
                            if zone.get("shape") == "polygon":
                                ui.Label("Polygon vertices remain in YAML when saving.", word_wrap=True)
                            self._zone_models.append(models)

    def _build_assets(self) -> None:
        with ui.CollapsableFrame("Trees, Rocks and Ground Vegetation", collapsed=True, height=0):
            with ui.VStack(spacing=3, height=0):
                ui.Label(
                    "Count = density × eligible mask area / 100 m² (rounded). Scale is a unitless multiplier; "
                    "clearance erodes mask/object boundaries and is not spacing between plants.",
                    word_wrap=True,
                )
                for index, asset in enumerate(self._raw.get("assets", ())):
                    name = str(asset.get("name", f"Asset{index}"))
                    with ui.CollapsableFrame(name, collapsed=True, height=0):
                        with ui.VStack(spacing=2, height=0):
                            models = {
                                "density": self._float(f"asset.{index}.density", "Density (instances/100 m²)", asset.get("density_per_100m2", 0.0)),
                                "scale_min": self._float(f"asset.{index}.scale_min", "Min Scale (multiplier)", asset.get("scale_min", 0.9)),
                                "scale_max": self._float(f"asset.{index}.scale_max", "Max Scale (multiplier)", asset.get("scale_max", 1.1)),
                                "clearance": self._float(f"asset.{index}.clearance", "Boundary/Object Clearance (m)", asset.get("clearance_m", 0.0)),
                                "allowed": self._string(f"asset.{index}.allowed", "Allowed Masks (CSV names)", ", ".join(asset.get("allowed_masks", ()))),
                                "forbidden": self._string(f"asset.{index}.forbidden", "Forbidden Masks (CSV names)", ", ".join(asset.get("forbidden_masks", ()))),
                                "semantic": self._string(f"asset.{index}.semantic", "Semantic Label", asset.get("semantic_label", "")),
                                "lidar": self._bool(f"asset.{index}.lidar", "LiDAR Visible (metadata)", asset.get("lidar_visible", True)),
                                "align": self._bool(f"asset.{index}.align", "Align Rotation to Surface", asset.get("align_to_surface", False)),
                            }
                            self._asset_models.append(models)

    def _build_surface(self) -> None:
        surface = self._raw.get("surface", {}) or {}
        with ui.CollapsableFrame("Ground Surface", collapsed=True, height=0):
            with ui.VStack(spacing=3, height=0):
                self._string("surface.diffuse_texture", "Diffuse Texture", surface.get("diffuse_texture", ""))
                self._string("surface.normal_texture", "Normal Texture", surface.get("normal_texture", ""))
                self._string("surface.roughness_texture", "Roughness Texture", surface.get("roughness_texture", ""))
                self._string("surface.ao_texture", "AO Texture", surface.get("ao_texture", ""))
                self._bool("surface.project_uvw", "Project UVW", surface.get("project_uvw", True))
                self._float("surface.texture_scale", "Texture Scale", surface.get("texture_scale", 1.0))
                self._triple("surface.diffuse_color", "Diffuse RGB", surface.get("diffuse_color", (0.12, 0.28, 0.07)))

    def _build_lighting(self) -> None:
        lighting = self._raw.get("lighting", {}) or {}
        with ui.CollapsableFrame("Environment Lighting", collapsed=True, height=0):
            with ui.VStack(spacing=3, height=0):
                self._bool("lighting.enabled", "Enable Lighting", lighting.get("enabled", False))
                self._string("lighting.hdri_path", "HDRI Path", lighting.get("hdri_path", ""))
                self._float("lighting.dome_intensity", "Dome Intensity", lighting.get("dome_intensity", 1000.0))
                self._triple("lighting.dome_rotation_deg", "Dome Rotation (deg)", lighting.get("dome_rotation_deg", (0, 0, 0)))
                self._float("lighting.sun_intensity", "Sun Intensity", lighting.get("sun_intensity", 2500.0))
                self._float("lighting.sun_angle_deg", "Sun Angle (deg)", lighting.get("sun_angle_deg", 1.0))
                self._triple("lighting.sun_rotation_deg", "Sun Rotation (deg)", lighting.get("sun_rotation_deg", (35, -25, 25)))
                self._float("lighting.color_temperature_k", "Color Temperature (K)", lighting.get("color_temperature_k", 5500.0))

    def _build_output(self) -> None:
        output = self._raw.get("output", {}) or {}
        with ui.CollapsableFrame("Output and Navigation Truth", collapsed=True, height=0):
            with ui.VStack(spacing=3, height=0):
                self._path_string(
                    "output.truth_directory",
                    "Truth Directory",
                    output.get("truth_directory", ""),
                    self._browse_truth_directory,
                )

    @staticmethod
    def _csv(value: str) -> list[str]:
        return [item.strip() for item in value.split(",") if item.strip()]

    def _collect_mapping(self) -> dict:
        value = copy.deepcopy(self._raw)
        value["schema_version"] = 1
        value["scene_name"] = self._models["scene_name"].get_value_as_string()
        value.setdefault("scene", {})["preset"] = self._models["preset"].get_value_as_string()
        value["parent_path"] = self._models["parent_path"].get_value_as_string()
        value["global_seed"] = self._models["global_seed"].get_value_as_int()
        value["asset_root"] = self._models["asset_root"].get_value_as_string()

        terrain = value.setdefault("terrain", {})
        terrain.update({
            "mode": SUPPORTED_TERRAIN_MODES[self._models["terrain.mode"].get_value_as_int()],
            "size_m": [self._models["terrain.size_m.0"].get_value_as_float(), self._models["terrain.size_m.1"].get_value_as_float()],
            "resolution_m": self._models["terrain.resolution_m"].get_value_as_float(),
            "base_height_m": self._models["terrain.base_height_m"].get_value_as_float(),
            "elevation_delta_m": self._models["terrain.elevation_delta_m"].get_value_as_float(),
            "wavelength_m": self._models["terrain.wavelength_m"].get_value_as_float(),
            "octaves": self._models["terrain.octaves"].get_value_as_int(),
            "persistence": self._models["terrain.persistence"].get_value_as_float(),
            "lacunarity": self._models["terrain.lacunarity"].get_value_as_float(),
            "slope_direction_deg": self._models["terrain.slope_direction_deg"].get_value_as_float(),
            "slope_deg": self._models["terrain.slope_deg"].get_value_as_float(),
            "max_slope_deg": self._models["terrain.max_slope_deg"].get_value_as_float(),
            "max_cross_slope_deg": self._models["terrain.max_cross_slope_deg"].get_value_as_float(),
            "max_local_curvature_per_m": self._models["terrain.max_local_curvature_per_m"].get_value_as_float(),
            "smoothing_passes": self._models["terrain.smoothing_passes"].get_value_as_int(),
            "terrace_count": self._models["terrain.terrace_count"].get_value_as_int(),
            "transition_width_m": self._models["terrain.transition_width_m"].get_value_as_float(),
            "heightmap_path": self._models["terrain.heightmap_path"].get_value_as_string(),
            "heightmap_scale_m": self._models["terrain.heightmap_scale_m"].get_value_as_float(),
            "spawn_pad": {
                "center_m": [self._models["terrain.spawn_pad.center_m.0"].get_value_as_float(), self._models["terrain.spawn_pad.center_m.1"].get_value_as_float()],
                "radius_m": self._models["terrain.spawn_pad.radius_m"].get_value_as_float(),
            },
        })

        regions = value.setdefault("regions", {})
        regions.update({
            "enabled": self._models["regions.enabled"].get_value_as_bool(),
            "boundary_buffer_m": self._models["regions.boundary_buffer_m"].get_value_as_float(),
            "tall_grass_band_m": self._models["regions.tall_grass_band_m"].get_value_as_float(),
            "bare_soil_fraction": self._models["regions.bare_soil_fraction"].get_value_as_float(),
            "bare_soil_patch_count": self._models["regions.bare_soil_patch_count"].get_value_as_int(),
        })
        zones = regions.setdefault("zones", [])
        for index, models in enumerate(self._zone_models):
            zone = zones[index]
            zone.update({
                "name": models["name"].get_value_as_string(),
                "kind": models["kind"].get_value_as_string(),
                "shape": models["shape"].get_value_as_string(),
                "center_m": [models["center"][0].get_value_as_float(), models["center"][1].get_value_as_float()],
                "radius_m": models["radius"].get_value_as_float(),
                "size_m": [models["size"][0].get_value_as_float(), models["size"][1].get_value_as_float()],
            })

        for index, models in enumerate(self._asset_models):
            asset = value["assets"][index]
            asset.update({
                "density_per_100m2": models["density"].get_value_as_float(),
                "scale_min": models["scale_min"].get_value_as_float(),
                "scale_max": models["scale_max"].get_value_as_float(),
                "clearance_m": models["clearance"].get_value_as_float(),
                "allowed_masks": self._csv(models["allowed"].get_value_as_string()),
                "forbidden_masks": self._csv(models["forbidden"].get_value_as_string()),
                "semantic_label": models["semantic"].get_value_as_string(),
                "lidar_visible": models["lidar"].get_value_as_bool(),
                "align_to_surface": models["align"].get_value_as_bool(),
            })

        surface = value.setdefault("surface", {})
        for name in ("diffuse_texture", "normal_texture", "roughness_texture", "ao_texture"):
            surface[name] = self._models[f"surface.{name}"].get_value_as_string()
        surface["project_uvw"] = self._models["surface.project_uvw"].get_value_as_bool()
        surface["texture_scale"] = self._models["surface.texture_scale"].get_value_as_float()
        surface["diffuse_color"] = [self._models[f"surface.diffuse_color.{i}"].get_value_as_float() for i in range(3)]

        lighting = value.setdefault("lighting", {})
        lighting.update({
            "enabled": self._models["lighting.enabled"].get_value_as_bool(),
            "hdri_path": self._models["lighting.hdri_path"].get_value_as_string(),
            "dome_intensity": self._models["lighting.dome_intensity"].get_value_as_float(),
            "dome_rotation_deg": [self._models[f"lighting.dome_rotation_deg.{i}"].get_value_as_float() for i in range(3)],
            "sun_intensity": self._models["lighting.sun_intensity"].get_value_as_float(),
            "sun_angle_deg": self._models["lighting.sun_angle_deg"].get_value_as_float(),
            "sun_rotation_deg": [self._models[f"lighting.sun_rotation_deg.{i}"].get_value_as_float() for i in range(3)],
            "color_temperature_k": self._models["lighting.color_temperature_k"].get_value_as_float(),
        })
        value.setdefault("output", {})["truth_directory"] = self._models["output.truth_directory"].get_value_as_string()
        return value

    def _runtime_config(self):
        mapping = self._collect_mapping()
        config_path = Path(self._config_model.get_value_as_string()).resolve()
        suffix = ".json" if config_path.suffix.lower() == ".json" else ".yaml"
        temporary = tempfile.NamedTemporaryFile(
            mode="w", suffix=suffix, prefix=".lawn_ui_", dir=config_path.parent, delete=False, encoding="utf-8"
        )
        temporary_path = Path(temporary.name)
        temporary.close()
        try:
            self._write_mapping(temporary_path, mapping)
            return load_config(temporary_path)
        finally:
            temporary_path.unlink(missing_ok=True)

    def destroy(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        if self._path_picker is not None:
            self._path_picker.destroy()
            self._path_picker = None
        self._task = None
        if self._window is not None:
            self._window.destroy()
        self._window = None
        self._models.clear()
        self._asset_models.clear()
        self._zone_models.clear()

    def _visibility_changed(self, visible: bool) -> None:
        # Retain ownership while hidden so reopening reuses this window and
        # extension shutdown can always destroy it explicitly.
        pass

    def _reload(self) -> None:
        try:
            self._config_path = Path(self._config_model.get_value_as_string()).resolve()
            self._read_mapping(self._config_path)
            self._window.destroy()
            self._window = None
            self.show()
            self._status.text = f"Loaded: {self._config_path.name}"
        except Exception as exc:
            self._status.text = f"Load error: {exc}"

    def _save(self) -> None:
        try:
            target = Path(self._config_model.get_value_as_string()).resolve()
            target.parent.mkdir(parents=True, exist_ok=True)
            mapping = self._collect_mapping()
            # Validate through the same production loader before replacing the file.
            suffix = ".json" if target.suffix.lower() == ".json" else ".yaml"
            with tempfile.NamedTemporaryFile(mode="w", suffix=suffix, dir=target.parent, delete=False) as stream:
                validation_path = Path(stream.name)
            try:
                self._write_mapping(validation_path, mapping)
                load_config(validation_path)
            finally:
                validation_path.unlink(missing_ok=True)
            self._write_mapping(target, mapping)
            self._raw = mapping
            self._config_path = target
            self._status.text = f"Saved: {target}"
        except Exception as exc:
            self._status.text = f"Save error: {exc}"

    def _start_generate(self) -> None:
        if self._task and not self._task.done():
            return
        self._task = asyncio.ensure_future(self._generate())

    async def _generate(self) -> None:
        try:
            config = self._runtime_config()
            controller = self._controller_getter(config.asset_root)
            self._status.text = "Computing terrain and placements..."
            loop = asyncio.get_running_loop()
            arrays = await loop.run_in_executor(None, controller.compute, config)
            await omni.kit.app.get_app().next_update_async()
            self._status.text = "Applying USD on the Kit main thread..."
            counts = controller.apply(config, arrays)
            if config.output.truth_directory:
                from .truth import export_truth

                self._status.text = "Exporting navigation truth..."
                await loop.run_in_executor(None, export_truth, config.output.truth_directory, config, arrays, controller.assets)
            self._status.text = f"Generated with seed={config.seed}: {counts}"
        except Exception as exc:
            carb.log_error(f"[{EXTENSION_NAME}] generation failed: {exc}")
            if self._status:
                self._status.text = f"Error: {exc}"

    def _clear(self) -> None:
        try:
            config = self._runtime_config()
            self._controller_getter(config.asset_root).clear(config.parent_path)
            self._status.text = f"Cleared: {config.parent_path}"
        except Exception as exc:
            self._status.text = f"Error: {exc}"
