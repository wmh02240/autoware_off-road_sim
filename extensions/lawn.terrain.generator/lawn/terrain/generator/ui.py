"""Small Isaac Sim UI; generation logic remains usable without it."""

from __future__ import annotations

import asyncio
from pathlib import Path

import carb
import omni.kit.app
import omni.ui as ui

from .config import load_config
from .constants import EXTENSION_NAME


class GeneratorWindow:
    def __init__(self, extension_path: str, controller_getter, asset_root: str):
        self._extension_path = extension_path
        self._controller_getter = controller_getter
        self._asset_root = asset_root
        self._window = None
        self._task = None
        self._status = None
        self._config_model = None

    def show(self) -> None:
        if self._window is not None:
            self._window.visible = True
            return
        # Extension Manager may report the registration symlink under
        # release/exts. Resolve it before deriving the repository root.
        extension_path = Path(self._extension_path).resolve()
        default_config = extension_path.parents[1] / "scripts/configs/lawn_generator_stage3_offroad.yaml"
        self._window = ui.Window(EXTENSION_NAME, width=520, height=220)
        self._window.set_visibility_changed_fn(self._visibility_changed)
        with self._window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Config-driven Isaac Sim 6.x generator")
                with ui.HStack(height=24):
                    ui.Label("Config", width=80)
                    self._config_model = ui.SimpleStringModel(str(default_config))
                    ui.StringField(self._config_model)
                ui.Label(f"Asset root: {self._asset_root}", word_wrap=True)
                with ui.HStack(height=30):
                    ui.Button("Generate", clicked_fn=self._start_generate)
                    ui.Button("Clear", clicked_fn=self._clear)
                self._status = ui.Label("Ready")

    def destroy(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None
        if self._window is not None:
            self._window.destroy()
        self._window = None

    def _visibility_changed(self, visible: bool) -> None:
        # Retain ownership while hidden so reopening reuses this window and
        # extension shutdown can always destroy it explicitly.
        pass

    def _start_generate(self) -> None:
        if self._task and not self._task.done():
            return
        self._task = asyncio.ensure_future(self._generate())

    async def _generate(self) -> None:
        try:
            config = load_config(self._config_model.get_value_as_string())
            controller = self._controller_getter(config.asset_root)
            self._status.text = "Computing..."
            loop = asyncio.get_running_loop()
            arrays = await loop.run_in_executor(None, controller.compute, config)
            await omni.kit.app.get_app().next_update_async()
            counts = controller.apply(config, arrays)
            self._status.text = f"Generated: {counts}"
        except Exception as exc:
            carb.log_error(f"[{EXTENSION_NAME}] generation failed: {exc}")
            if self._status:
                self._status.text = f"Error: {exc}"

    def _clear(self) -> None:
        try:
            config = load_config(self._config_model.get_value_as_string())
            self._controller_getter(config.asset_root).clear(config.parent_path)
            self._status.text = "Cleared"
        except Exception as exc:
            self._status.text = f"Error: {exc}"
