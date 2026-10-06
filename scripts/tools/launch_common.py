"""两个 Isaac Sim 启动器共用的启动引导逻辑。

场景加载、车辆模型修复和仿真循环仍由各自的入口脚本负责；本模块只处理
两者完全一致的启动前工作，避免 ``launch_sim.py`` 和
``launch_lawn_mower_sim.py`` 长期复制同一段初始化代码。
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
from typing import Any, Dict

def _resolve_network_ip(interface: str) -> str:
    """将网卡名转换为 IPv4；传入 IP 或解析失败时返回原值。"""
    if "." in interface:
        return interface
    try:
        import fcntl

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packed = fcntl.ioctl(
            sock.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack("256s", interface[:15].encode()),
        )
        return socket.inet_ntoa(packed[20:24])
    except Exception:
        return interface


def _configure_ros_environment(config: Dict[str, Any]) -> float:
    """设置 Isaac Sim Python 进程使用的 ROS 2/DDS 环境变量。"""
    network_setup = config.get("network_setup", {})
    ros2_domain_id = network_setup.get("ros2_domain_id", 0)
    network_interface = network_setup.get("network_interface", "auto")
    ros2_cmd_timeout_s = float(network_setup.get("ros2_cmd_timeout_s", 1.5))

    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(ros2_domain_id)

    if network_interface != "auto":
        lan_ip = _resolve_network_ip(network_interface)
        cyclone_xml = (
            '<?xml version="1.0" encoding="UTF-8" ?>\n'
            "<CycloneDDS><Domain><General>\n"
            f"  <NetworkInterfaceAddress>{lan_ip}</NetworkInterfaceAddress>\n"
            "</General></Domain></CycloneDDS>\n"
        )
        cyclone_xml_path = "/tmp/cyclone_hil.xml"
        with open(cyclone_xml_path, "w", encoding="utf-8") as stream:
            stream.write(cyclone_xml)
        os.environ["CYCLONEDDS_URI"] = f"file://{cyclone_xml_path}"
        print(f"[ROS2] CycloneDDS pinned to interface: {lan_ip}")
        print(f"[ROS2] CYCLONEDDS_URI = {os.environ['CYCLONEDDS_URI']}")
    else:
        print("[ROS2] CycloneDDS using automatic interface/multicast discovery")

    print(
        "[ROS2] RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
        f" ROS_DOMAIN_ID={ros2_domain_id}"
    )
    return ros2_cmd_timeout_s


def _experience_path() -> str:
    """查找完整 Isaac Sim Experience 文件。"""
    carb_app_path = os.environ.get("CARB_APP_PATH", "")
    release_dir = os.path.dirname(carb_app_path)
    experience = os.path.join(release_dir, "apps", "isaacsim.exp.full.kit")
    if os.path.exists(experience):
        return experience

    apps_dir = os.path.join(release_dir, "apps")
    candidates = (
        [name for name in os.listdir(apps_dir) if "full" in name and name.endswith(".kit")]
        if os.path.isdir(apps_dir)
        else []
    )
    if candidates:
        experience = os.path.join(apps_dir, sorted(candidates)[0])
        print(f"[SimApp] Using experience: {experience}")
        return experience

    print(
        f"[SimApp] Warning: isaacsim.exp.full.kit not found in {apps_dir}, "
        "falling back to default"
    )
    return ""


def _append_tracy_args(config: Dict[str, Any]) -> bool:
    """在 SimulationApp 启动前加入可选 Tracy 参数。"""
    tracy_cfg = config.get("profiling", {}).get("tracy", {})
    if not bool(tracy_cfg.get("enabled", False)):
        return False

    tracy_gpu = bool(tracy_cfg.get("gpu", True))
    tracy_args = [
        "--enable",
        "omni.kit.profiler.tracy",
        "--/profiler/enabled=true",
        "--/app/profilerBackend=tracy",
        "--/app/profileFromStart=true",
    ]
    if tracy_gpu:
        tracy_args.extend([
            "--/profiler/gpu=true",
            "--/profiler/gpu/tracyInject/enabled=true",
        ])
    for argument in tracy_args:
        if argument not in sys.argv:
            sys.argv.append(argument)
    print(
        f"[Profile] Tracy enabled (GPU trace: {tracy_gpu}). "
        "Open the bundled Tracy UI and click Connect."
    )
    return True


def _configure_viewport(carb_settings: Any, config: Dict[str, Any], headless: bool) -> None:
    """应用窗口、渲染器、DLSS 和可选视口模式设置。"""
    viewport_cfg = config.get("graphics_settings", {})
    render_res = viewport_cfg.get("render_resolution", [2560, 1440])
    if not headless:
        carb_settings.set_int("/app/renderer/resolution/width", render_res[0])
        carb_settings.set_int("/app/renderer/resolution/height", render_res[1])
        carb_settings.set_bool("/app/window/showFps", True)
        carb_settings.set_bool("/app/viewport/showFps", True)
        carb_settings.set_bool("/exts/omni.kit.viewport.window/fps", True)

        if viewport_cfg.get("enable_DLSS_FPS_Multiplier_x2", False):
            carb_settings.set_int("/rtx/post/aa/op", 3)
            carb_settings.set_int("/rtx/post/dlss/execMode", 0)
            carb_settings.set_bool("/rtx-transient/dlssg/enabled", True)
            print("[Renderer] DLSS Performance + FPS Multiplier x2 (DLSS-G) enabled.")
        if viewport_cfg.get("disable_shadows", False):
            carb_settings.set_bool("/rtx/shadows/enabled", False)
            print("[Renderer] Shadows disabled.")
        if viewport_cfg.get("disable_ambient_occlusion", False):
            carb_settings.set_bool("/rtx/ambientOcclusion/enabled", False)
            print("[Renderer] Ambient occlusion disabled.")
        if viewport_cfg.get("disable_reflections", False):
            carb_settings.set_bool("/rtx/reflections/enabled", False)
            print("[Renderer] Reflections disabled.")
        print(
            f"Configured Viewport Render Resolution to {render_res[0]}x{render_res[1]} "
            "with FPS counter"
        )

        if bool(config.get("user_interface", {}).get("viewport_mode", False)):
            try:
                import omni.ui

                panel_names = [
                    "Stage", "Content", "Console", "Property", "Layer",
                    "Semantics Schema Editor", "Action Graph", "Render Settings",
                    "Statistics", "Profiler", "Animation Graph", "Physics",
                    "Replicator", "Material Graph", "Robot Inspector",
                ]
                for panel_name in panel_names:
                    try:
                        omni.ui.Workspace.show_window(panel_name, False)
                    except Exception:
                        pass
                carb_settings.set_bool("/app/window/showMenu", False)
                print("[UI] Viewport mode: panels suppressed at startup.")
            except Exception as error:
                print(f"[UI] Viewport mode startup error: {error}")


def bootstrap(default_config: str, description: str) -> Dict[str, Any]:
    """完成两个启动器共有的参数、ROS、Isaac Sim 和 UI 初始化。"""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", type=str, default=default_config, help="Path to configuration file")
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run without a display window. All vehicles default to ROS2_CONTROL mode.",
    )
    args, _kit_extra_args = parser.parse_known_args()
    headless_mode = args.headless
    config_path = os.path.abspath(args.config)
    print(f"Loading configuration from: {config_path}")

    import yaml

    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    # 本模块位于 scripts/tools/；仓库根目录需要向上返回两级。
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    ros2_cmd_timeout_s = _configure_ros_environment(config)
    viewport_cfg = config.get("graphics_settings", {})
    render_res = viewport_cfg.get("render_resolution", [2560, 1440])
    tracy_enabled = _append_tracy_args(config)

    # 必须在导入其他 omni 模块之前创建 SimulationApp。
    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp

    sim_config = {
        "headless": headless_mode,
        "experience": _experience_path(),
    }
    if tracy_enabled:
        sim_config["profiler_backend"] = ["tracy"]
    if not headless_mode:
        sim_config.update({
            "width": render_res[0],
            "height": render_res[1],
            "display_options": 3287,
        })
    simulation_app = SimulationApp(sim_config)

    import carb

    carb_settings = carb.settings.get_settings()
    _configure_viewport(carb_settings, config, headless_mode)
    carb_settings.set_bool("/app/stage/generateDefaultLight", False)

    ui_cfg = config.get("user_interface", {})
    return {
        "args": args,
        "config": config,
        "config_path": config_path,
        "repo_root": repo_root,
        "headless_mode": headless_mode,
        "ros2_cmd_timeout_s": ros2_cmd_timeout_s,
        "simulation_app": simulation_app,
        "carb_settings": carb_settings,
        "viewport_opts": viewport_cfg,
        "render_res": render_res,
        "ui_viewport_mode": not headless_mode and bool(ui_cfg.get("viewport_mode", False)),
        "ui_split_screen": not headless_mode and bool(ui_cfg.get("split_screen", False)),
        "ui_default_ctrl_mode": str(ui_cfg.get("default_control_mode", "KEYBOARD")).strip().upper(),
    }
