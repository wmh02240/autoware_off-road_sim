"""Kit extension lifecycle and Window menu integration."""

from __future__ import annotations

import carb
import carb.eventdispatcher
import omni.ext
import omni.kit.app
import omni.usd
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from .assets import audit_assets, discover_asset_root
from .constants import ASSET_ROOT_SETTING, EXTENSION_NAME
from .controller import GeneratorController
from .ui import GeneratorWindow


class LawnTerrainGeneratorExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._ext_id = ext_id
        manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = manager.get_extension_path(ext_id)
        configured = carb.settings.get_settings().get_as_string(ASSET_ROOT_SETTING)
        self._asset_root = discover_asset_root(self._extension_path, configured)
        self._context = omni.usd.get_context()
        self._window = GeneratorWindow(
            self._extension_path, self._get_controller, str(self._asset_root)
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_NAME, onclick_fn=self._window.show)
        ]
        add_menu_items(self._menu_items, "Window")
        self._stage_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._context.stage_event_name(omni.usd.StageEventType.CLOSED),
            on_event=self._on_stage_closed,
            observer_name=f"{EXTENSION_NAME}.on_stage_closed",
        )
        missing = [name for name, item in audit_assets(self._asset_root).items() if not item.available]
        carb.log_info(f"[{EXTENSION_NAME}] enabled; asset root: {self._asset_root}")
        if missing:
            carb.log_warn(
                f"[{EXTENSION_NAME}] disabled missing assets: {', '.join(missing)}. "
                "Switchgrass and Container are absent from the pinned public checkout."
            )

    def on_shutdown(self) -> None:
        subscription = getattr(self, "_stage_subscription", None)
        if subscription is not None:
            subscription.reset()
        self._stage_subscription = None
        if getattr(self, "_menu_items", None):
            remove_menu_items(self._menu_items, "Window")
        self._menu_items = []
        if getattr(self, "_window", None):
            self._window.destroy()
        self._window = None
        self._context = None
        carb.log_info(f"[{EXTENSION_NAME}] disabled")

    def _get_controller(self, config_asset_root: str = "") -> GeneratorController:
        stage = self._context.get_stage()
        if stage is None:
            raise RuntimeError("open or create a USD stage before generating")
        root = discover_asset_root(self._extension_path, config_asset_root or str(self._asset_root))
        return GeneratorController(stage, str(root))

    def _on_stage_closed(self, _event: carb.eventdispatcher.Event) -> None:
        if self._window:
            self._window.destroy()
