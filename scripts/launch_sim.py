import os
import argparse
import collections
import time
import subprocess
import sys
import yaml
try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

def main():
    parser = argparse.ArgumentParser(description="Launch IsaacSim with specified assets")
    parser.add_argument(
        "--config",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "configs", "pumptrack_simple.yaml"),
        help="Path to configuration file"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run without a display window. All vehicles default to ROS2_CONTROL mode.",
    )
    # 在解析本启动器自身选项的同时，为 SimulationApp 保留 Kit/Carbonite 参数
    #（例如 Tracy 分析器开关）。
    args, _kit_extra_args = parser.parse_known_args()
    headless_mode = args.headless

    config_path = os.path.abspath(args.config)
    print(f"Loading configuration from: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 以仓库根目录为基准解析路径
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    
    env_asset_path = os.path.join(repo_root, config.get("environment_asset", ""))
    print(f"Environment asset: {env_asset_path}")

    # 解析网络配置
    network_setup = config.get("network_setup", {})
    ros2_domain_id = network_setup.get("ros2_domain_id", 0)
    network_interface = network_setup.get("network_interface", "auto")
    ros2_cmd_timeout_s = float(network_setup.get("ros2_cmd_timeout_s", 1.5))

    ui_cfg = config.get("user_interface", {})
    _ui_viewport_mode = not headless_mode and bool(ui_cfg.get("viewport_mode", False))
    _ui_split_screen  = not headless_mode and bool(ui_cfg.get("split_screen", False))
    # 车辆启动时的默认控制模式（非无头模式）。"KEYBOARD" → KEYBOARD_CONTROL，
    # "ROS2_CONTROL" → ROS2_CONTROL。无头模式下由于没有键盘，
    # 下方逻辑始终会覆盖为 ROS2_CONTROL。
    _ui_default_ctrl_mode = str(ui_cfg.get("default_control_mode", "KEYBOARD")).strip().upper()

    # ── CycloneDDS 设置 ────────────────────────────────────────────────────
    # 为稳妥起见显式设置 RMW，以防 Isaac Sim 自带解释器启动的 Python
    # 没有继承 Docker 环境变量。
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(ros2_domain_id)

    if network_interface != "auto":
        # 将显式 IP 或网卡名称解析为 IP 地址
        def _resolve_ip(iface):
            if "." in iface:          # 已经是 IP 字符串
                return iface
            try:                      # 通过 fcntl SIOCGIFADDR 将网卡名称转换为 IP
                import socket, fcntl, struct
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                packed = fcntl.ioctl(s.fileno(), 0x8915,
                                     struct.pack("256s", iface[:15].encode()))
                return socket.inet_ntoa(packed[20:24])
            except Exception:
                return iface          # 后备方案：原样使用该字符串

        lan_ip = _resolve_ip(network_interface)
        _cyclone_xml = (
            '<?xml version="1.0" encoding="UTF-8" ?>\n'
            '<CycloneDDS><Domain><General>\n'
            f'  <NetworkInterfaceAddress>{lan_ip}</NetworkInterfaceAddress>\n'
            '</General></Domain></CycloneDDS>\n'
        )
        _cyclone_xml_path = "/tmp/cyclone_hil.xml"
        with open(_cyclone_xml_path, "w") as _f:
            _f.write(_cyclone_xml)
        os.environ["CYCLONEDDS_URI"] = f"file://{_cyclone_xml_path}"
        print(f"[ROS2] CycloneDDS pinned to interface: {lan_ip}")
        print(f"[ROS2] CYCLONEDDS_URI = {os.environ['CYCLONEDDS_URI']}")
    else:
        print("[ROS2] CycloneDDS using automatic interface/multicast discovery")

    print(f"[ROS2] RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ROS_DOMAIN_ID={ros2_domain_id}")

    # 初始化仿真应用
    viewport_opts = config.get("graphics_settings", {})
    render_res = viewport_opts.get("render_resolution", [2560, 1440])

    # 此操作必须在导入其他 omni 模块之前完成。
    # 加载完整的 Isaac Sim Experience（与 isaac-sim.sh 使用相同 Kit 配置），
    # 以提供全部面板、编辑器和扩展，而不是最精简的 Python 独立界面。
    # CARB_APP_PATH 指向 .../release/kit；Experience 的 Kit 文件位于同级的
    # .../release/apps/ 目录，因此需要向上返回一级。
    _carb_app_path = os.environ.get("CARB_APP_PATH", "")
    _release_dir = os.path.dirname(_carb_app_path)  # 对应路径：.../release
    _exp_full = os.path.join(_release_dir, "apps", "isaacsim.exp.full.kit")
    if not os.path.exists(_exp_full):
        # 后备方案：在 apps/ 中查找任意 Isaac Sim 完整版 Kit 文件
        _apps_dir = os.path.join(_release_dir, "apps")
        _candidates = [f for f in os.listdir(_apps_dir) if "full" in f and f.endswith(".kit")] if os.path.isdir(_apps_dir) else []
        if _candidates:
            _exp_full = os.path.join(_apps_dir, sorted(_candidates)[0])
            print(f"[SimApp] Using experience: {_exp_full}")
        else:
            print(f"[SimApp] Warning: isaacsim.exp.full.kit not found in {_apps_dir}, falling back to default")
            _exp_full = ""
    _sim_cfg = {"headless": headless_mode, "experience": _exp_full}

    # Tracy 必须在 SimulationApp 启动前配置。YAML 开关可避免常规运行承担
    # 分析器开销，而上面的 parse_known_args 仍允许从命令行手动传入 Kit 分析参数。
    _profiling_cfg = config.get("profiling", {})
    _tracy_cfg = _profiling_cfg.get("tracy", {})
    _tracy_enabled = bool(_tracy_cfg.get("enabled", False))
    if _tracy_enabled:
        _sim_cfg["profiler_backend"] = ["tracy"]
        _tracy_gpu = bool(_tracy_cfg.get("gpu", True))
        _tracy_args = [
            "--enable", "omni.kit.profiler.tracy",
            "--/profiler/enabled=true",
            "--/app/profilerBackend=tracy",
            "--/app/profileFromStart=true",
        ]
        if _tracy_gpu:
            _tracy_args.extend([
                "--/profiler/gpu=true",
                "--/profiler/gpu/tracyInject/enabled=true",
            ])
        for _tracy_arg in _tracy_args:
            if _tracy_arg not in sys.argv:
                sys.argv.append(_tracy_arg)
        print(f"[Profile] Tracy enabled (GPU trace: {_tracy_gpu}). "
              "Open the bundled Tracy UI and click Connect.")
    if not headless_mode:
        _sim_cfg["width"]           = render_res[0]
        _sim_cfg["height"]          = render_res[1]
        _sim_cfg["display_options"] = 3287  # 3286（默认值）| 1（显示 FPS）
    simulation_app = SimulationApp(_sim_cfg)
    
    # 单独强制设置输出渲染器分辨率（与窗口大小解耦时很有用）
    import carb
    carb_settings = carb.settings.get_settings()

    if not headless_mode:
        carb_settings.set_int("/app/renderer/resolution/width", render_res[0])
        carb_settings.set_int("/app/renderer/resolution/height", render_res[1])
        # 切换视口 FPS 显示
        carb_settings.set_bool("/app/window/showFps", True)
        carb_settings.set_bool("/app/viewport/showFps", True)
        carb_settings.set_bool("/exts/omni.kit.viewport.window/fps", True)
        # DLSS 超分辨率 + 帧生成（FPS 倍增器 x2）
        # /rtx/post/aa/op：0=无，1=TAA，2=FXAA，3=DLSS，4=DLAA
        # /rtx/post/dlss/execMode：0=性能（约 2x），1=平衡，2=质量，3=超高性能
        # /rtx-transient/dlssg/enabled：DLSS-G 帧生成，即界面中的 FPS 倍增器
        if viewport_opts.get("enable_DLSS_FPS_Multiplier_x2", False):
            carb_settings.set_int("/rtx/post/aa/op", 3)
            carb_settings.set_int("/rtx/post/dlss/execMode", 0)
            carb_settings.set_bool("/rtx-transient/dlssg/enabled", True)
            print("[Renderer] DLSS Performance + FPS Multiplier x2 (DLSS-G) enabled.")
        if viewport_opts.get("disable_shadows", False):
            carb_settings.set_bool("/rtx/shadows/enabled", False)
            print("[Renderer] Shadows disabled.")
        if viewport_opts.get("disable_ambient_occlusion", False):
            carb_settings.set_bool("/rtx/ambientOcclusion/enabled", False)
            print("[Renderer] Ambient occlusion disabled.")
        if viewport_opts.get("disable_reflections", False):
            carb_settings.set_bool("/rtx/reflections/enabled", False)
            print("[Renderer] Reflections disabled.")
        print(f"Configured Viewport Render Resolution to {render_res[0]}x{render_res[1]} with FPS counter")

        # 视口模式：启动时隐藏视口以外的所有面板，使它们从一开始就不向用户显示，
        # 而不是显示后再关闭。
        if _ui_viewport_mode:
            try:
                import omni.ui as _ui_startup
                for _pname in ["Stage", "Content", "Console", "Property", "Layer",
                               "Semantics Schema Editor", "Action Graph",
                               "Render Settings", "Statistics", "Profiler",
                               "Animation Graph", "Physics", "Replicator",
                               "Material Graph", "Robot Inspector"]:
                    try:
                        _ui_startup.Workspace.show_window(_pname, False)
                    except Exception:
                        pass
                carb_settings.set_bool("/app/window/showMenu", False)
                print("[UI] Viewport mode: panels suppressed at startup.")
            except Exception as _vms_err:
                print(f"[UI] Viewport mode startup error: {_vms_err}")

    # 阻止 Isaac Sim Full 自动向 Stage 添加 defaultLight。
    # 环境 USD 已包含 DomeLight，第二盏灯会改变场景效果。
    carb_settings.set_bool("/app/stage/generateDefaultLight", False)

    import omni.usd
    from pxr import UsdGeom, Gf, Sdf
    import omni.ext

    # 确保已有 Stage 打开
    context = omni.usd.get_context()
    if not context.get_stage():
        context.new_stage()
    stage = context.get_stage()
    
    def strip_embed_physics_scenes(filepath):
        from pxr import Usd, UsdPhysics
        try:
            temp_stage = Usd.Stage.Open(filepath)
            scenes = [p.GetPath() for p in temp_stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            if scenes:
                for path in scenes:
                    temp_stage.RemovePrim(path)
                temp_stage.Save()
                print(f"[Physics] Autocleaned embedded PhysicsScene natively from {filepath}")
        except Exception:
            pass
            
    # 加载环境
    env_config = config.get("environment", {})
    if isinstance(env_config, str):
        env_asset_path = env_config
        env_scale = [1.0, 1.0, 1.0]
        env_rot = [0.0, 0.0, 0.0]
        env_pos = [0.0, 0.0, 0.0]
    else:
        env_asset_path = env_config.get("asset", config.get("environment_asset", "assets/environments/pumptrack_simple.usd"))
        env_scale = env_config.get("scale", [1.0, 1.0, 1.0])
        env_rot = env_config.get("rotation_euler", [0.0, 0.0, 0.0])
        env_pos = env_config.get("translation", [0.0, 0.0, 0.0])
        
    env_asset_path = os.path.abspath(os.path.join(repo_root, env_asset_path))
    
    if os.path.exists(env_asset_path):
        strip_embed_physics_scenes(env_asset_path)
        env_prim = stage.DefinePrim("/World/Environment", "Xform")
        
        # 使用 UsdGeom Xformable API 安全地应用变换
        xformable = UsdGeom.Xformable(env_prim)
        xformable.AddScaleOp().Set(Gf.Vec3d(*env_scale))
        xformable.AddRotateXYZOp().Set(Gf.Vec3d(*env_rot))
        xformable.AddTranslateOp().Set(Gf.Vec3d(*env_pos))
        
        env_prim.GetReferences().AddReference(env_asset_path)
        print(f"Successfully added environment reference to stage with Scale {env_scale} and Rotation {env_rot}.")
    else:
        print(f"Warning: Environment asset not found at {env_asset_path}")

    # 加载车辆
    vehicles = config.get("vehicles", [])
    for veh in vehicles:
        if not veh.get("enabled", True):
            print(f"Skipping disabled vehicle: {veh.get('name', 'Unknown')}")
            continue
            
        veh_name = veh.get("name", "Vehicle")
        veh_asset = veh.get("asset", "")
        veh_asset_path = os.path.join(repo_root, veh_asset)
        
        veh_scale = veh.get("scale", [1.0, 1.0, 1.0])
        veh_rot = veh.get("spawn_orientation", veh.get("rotation_euler", [0.0, 0.0, 0.0]))
        veh_pos = veh.get("spawn_position", [0.0, 0.0, 0.0])
        
        prim_path = f"/World/{veh_name}"
        
        if os.path.exists(veh_asset_path):
            strip_embed_physics_scenes(veh_asset_path)
            veh_prim = stage.DefinePrim(prim_path, "Xform")
            
            # 应用变换
            xformable = UsdGeom.Xformable(veh_prim)
            xformable.AddScaleOp().Set(Gf.Vec3d(*veh_scale))
            xformable.AddRotateXYZOp().Set(Gf.Vec3d(*veh_rot))
            xformable.AddTranslateOp().Set(Gf.Vec3d(*veh_pos))
            
            veh_prim.GetReferences().AddReference(veh_asset_path)
            print(f"Successfully added vehicle '{veh_name}' reference to stage at '{prim_path}'")
        else:
            print(f"Warning: Vehicle asset not found at {veh_asset_path}")

    # ── 车辆 USD 兼容性修复：IMU 固定关节 ───────────────────────────────────
    # roboracer_max.usd 包含名为 ``Sensors/IMU`` 的过期刚体目标。
    # 资源中实际的刚体 Prim 是底盘 base_link 下的 ``Imu_Sensor``。
    # 无法解析的刚体关系会使 PhysX 丢弃整个固定关节。这里不修改二进制 USD Crate，
    # 而是在组合 Stage 中写入覆盖，并且只替换这一确切的过期目标，
    # 从而保证该修复对其他车辆资源和配置是安全的。
    for _imu_joint in stage.Traverse():
        _imu_joint_path = _imu_joint.GetPath().pathString
        if not _imu_joint_path.endswith("/Joints/IMUFixedJoint"):
            continue

        _vehicle_root = _imu_joint_path.removesuffix("/Joints/IMUFixedJoint")
        _imu_target = Sdf.Path(
            f"{_vehicle_root}/Rigid_Bodies/Chassis/base_link/Imu_Sensor")
        if not stage.GetPrimAtPath(_imu_target).IsValid():
            print(f"[Physics] IMU joint repair skipped: expected sensor prim is missing: {_imu_target}")
            continue

        _repaired_rels = []
        for _rel in _imu_joint.GetRelationships():
            _targets = _rel.GetTargets()
            _new_targets = []
            _changed = False
            for _target in _targets:
                # 不要改写任意缺失目标：此修复刻意只作用于已知的旧 IMU 位置。
                if str(_target).endswith("/Sensors/IMU"):
                    _new_targets.append(_imu_target)
                    _changed = True
                else:
                    _new_targets.append(_target)
            if _changed:
                _rel.SetTargets(_new_targets)
                _repaired_rels.append(_rel.GetName())

        if _repaired_rels:
            print(f"[Physics] Repaired IMUFixedJoint at {_imu_joint_path}: "
                  f"{', '.join(_repaired_rels)} -> {_imu_target}")

    # 源车辆 USD 中有一个悬架关节，其写入的两个局部坐标系会解析到不同的世界坐标系。
    # 因此开始播放时，PhysX 会将连接部件突然吸合。在 PhysX 读取 USD 前，
    # 保留 body0 已写入的锚点，并由此推导 body1 的局部坐标系。
    for _susp_joint in stage.Traverse():
        _susp_path = _susp_joint.GetPath().pathString
        if not _susp_path.endswith("/Joints/Chassis__Arm_Rear_Upper_Right"):
            continue

        _body0_targets = _susp_joint.GetRelationship("physics:body0").GetTargets()
        _body1_targets = _susp_joint.GetRelationship("physics:body1").GetTargets()
        if len(_body0_targets) != 1 or len(_body1_targets) != 1:
            print(f"[Physics] Suspension joint repair skipped: expected one body0 and one body1 target at {_susp_path}")
            continue

        _body0 = stage.GetPrimAtPath(_body0_targets[0])
        _body1 = stage.GetPrimAtPath(_body1_targets[0])
        _pos0_attr = _susp_joint.GetAttribute("physics:localPos0")
        _rot0_attr = _susp_joint.GetAttribute("physics:localRot0")
        _pos1_attr = _susp_joint.GetAttribute("physics:localPos1")
        _rot1_attr = _susp_joint.GetAttribute("physics:localRot1")
        _pos0, _rot0 = _pos0_attr.Get(), _rot0_attr.Get()

        if (not _body0.IsValid() or not _body1.IsValid() or _pos0 is None or _rot0 is None
                or not _pos1_attr or not _rot1_attr):
            print(f"[Physics] Suspension joint repair skipped: incomplete joint data at {_susp_path}")
            continue

        try:
            def _quatd(_q):
                _im = _q.GetImaginary()
                return Gf.Quatd(float(_q.GetReal()), Gf.Vec3d(float(_im[0]), float(_im[1]), float(_im[2])))

            # USD 使用行向量变换：local_frame * body_world。
            _local0 = Gf.Matrix4d(1.0)
            _local0.SetRotate(Gf.Rotation(_quatd(_rot0)))
            _local0.SetTranslateOnly(Gf.Vec3d(float(_pos0[0]), float(_pos0[1]), float(_pos0[2])))
            _body0_world = omni.usd.get_world_transform_matrix(_body0)
            _body1_world = omni.usd.get_world_transform_matrix(_body1)
            _joint_world = _local0 * _body0_world
            _local1 = _joint_world * _body1_world.GetInverse()

            _new_pos1 = _local1.ExtractTranslation()
            _new_rot1 = _local1.ExtractRotationQuat()
            _new_im = _new_rot1.GetImaginary()
            _pos1_attr.Set(Gf.Vec3f(float(_new_pos1[0]), float(_new_pos1[1]), float(_new_pos1[2])))
            _rot1_attr.Set(Gf.Quatf(
                float(_new_rot1.GetReal()),
                Gf.Vec3f(float(_new_im[0]), float(_new_im[1]), float(_new_im[2])),
            ))

            _anchor_error = (
                _joint_world.ExtractTranslation()
                - (_local1 * _body1_world).ExtractTranslation()
            ).GetLength()
            print(f"[Physics] Repaired suspension joint at {_susp_path}: "
                  f"body1 local frame aligned (anchor error={_anchor_error:.6g} m)")
        except Exception as _susp_repair_err:
            print(f"[Physics] Suspension joint repair failed at {_susp_path}: {_susp_repair_err}")

    # Imu_Sensor 是由上述固定关节连接的刚体，但源 USD 使用负质量哨兵值，
    # 且没有可供 PhysX 推导质量属性的碰撞体。为这个小型 IMU（30 mm、10 g）
    # 显式设置物理有效的质量和长方体惯量，避免采用 PhysX 不稳定的小球后备值。
    # 此处不要修改 LiDAR；那是另一个资源问题，将单独修复。
    for _imu_sensor in stage.Traverse():
        _imu_sensor_path = _imu_sensor.GetPath().pathString
        if not _imu_sensor_path.endswith("/Rigid_Bodies/Chassis/base_link/Imu_Sensor"):
            continue
        try:
            from pxr import UsdPhysics
            _imu_mass_api = UsdPhysics.MassAPI.Apply(_imu_sensor)
            _imu_mass_api.CreateMassAttr(0.01)  # 单位：kg
            # 对边长 30 mm、质量 10 g 的立方体，I = m * side^2 / 6。
            _imu_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(1.5e-6, 1.5e-6, 1.5e-6))
            _imu_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            print(f"[Physics] Repaired IMU mass properties at {_imu_sensor_path}: "
                  "mass=0.01 kg, diagonalInertia=(1.5e-06, 1.5e-06, 1.5e-06) kg·m²")
        except Exception as _imu_mass_err:
            print(f"[Physics] IMU mass-property repair failed at {_imu_sensor_path}: {_imu_mass_err}")

    # OS2 RTX LiDAR 同样被写成使用负质量哨兵值且没有碰撞体的刚体。
    # 将其近似为 0.40 kg 的圆柱体（直径 90 mm、高 80 mm），以获得稳定的显式属性。
    # 此操作有意与上面的 IMU 修复相互独立。
    for _lidar_sensor in stage.Traverse():
        _lidar_sensor_path = _lidar_sensor.GetPath().pathString
        if not _lidar_sensor_path.endswith("/Sensors/OS2/sensor"):
            continue
        try:
            from pxr import UsdPhysics
            _lidar_mass_api = UsdPhysics.MassAPI.Apply(_lidar_sensor)
            _lidar_mass_api.CreateMassAttr(0.40)  # 单位：kg
            # 实心圆柱体惯量：Ixx=Iyy≈4.16e-4，Izz≈4.05e-4 kg·m²。
            _lidar_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(4.16e-4, 4.16e-4, 4.05e-4))
            _lidar_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            print(f"[Physics] Repaired OS2 LiDAR mass properties at {_lidar_sensor_path}: "
                  "mass=0.4 kg, diagonalInertia=(0.000416, 0.000416, 0.000405) kg·m²")
        except Exception as _lidar_mass_err:
            print(f"[Physics] OS2 LiDAR mass-property repair failed at {_lidar_sensor_path}: {_lidar_mass_err}")

    # 应用配置中的 PhysicsScene 覆盖
    physics_opts = config.get("physics_settings", {})
    if physics_opts:
        from pxr import UsdPhysics, Sdf
        solver_type = "PGS"  # TGS 对轮式车辆不稳定
        time_steps = float(physics_opts.get("time_steps_per_second", 120.0))
        enable_gpu  = False  # 禁用 GPU 动力学（与当前 USD 设置不兼容）
        bp_type     = "SAP"
        pos_iters   = int(physics_opts.get("solver_position_iterations", 16))
        vel_iters   = int(physics_opts.get("solver_velocity_iterations", 4))

        scenes_updated = False
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                attr_solver = prim.GetAttribute("physxScene:solverType")
                if attr_solver: attr_solver.Set(solver_type)
                else: prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set(solver_type)

                attr_ts = prim.GetAttribute("physxScene:timeStepsPerSecond")
                if attr_ts: attr_ts.Set(time_steps)
                else: prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(time_steps)

                attr_gpu = prim.GetAttribute("physxScene:enableGPUDynamics")
                if attr_gpu: attr_gpu.Set(enable_gpu)
                else: prim.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(enable_gpu)

                attr_bp = prim.GetAttribute("physxScene:broadphaseType")
                if attr_bp: attr_bp.Set(bp_type)
                else: prim.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set(bp_type)

                if scenes_updated:
                    # 在车辆或环境引用中发现了嵌入的重复 PhysicsScene！
                    # 这会彻底破坏 PhysX 时间线同步，并产生重叠读取警告。
                    # 必须从组合后的 USD Stage 中原生、彻底地清除所有多余场景。
                    print(f"[Physics] Warning: Eradicating duplicate PhysicsScene from composed stage at '{prim.GetPath()}'")
                    stage.RemovePrim(prim.GetPath())
                    continue

                print(f"[Physics] Configured Primary PhysicsScene at '{prim.GetPath()}' -> Solver: {solver_type}, Steps: {time_steps}, GPU: {enable_gpu}, BP: {bp_type}")
                scenes_updated = True

        if not scenes_updated:
            print("[Physics] Emitting new singleton PhysicsScene since no assets contained one mapping.")
            scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
            prim = scene.GetPrim()
            prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set(solver_type)
            prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(time_steps)
            prim.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(enable_gpu)
            prim.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set(bp_type)
            print(f"[Physics] Force-configured Singleton PhysicsScene at '/physicsScene' -> Solver: {solver_type}, Steps: {time_steps}, GPU: {enable_gpu}, BP: {bp_type}")

        # 将每个 Articulation 的求解器迭代次数应用到所有 Articulation 根节点
        try:
            from pxr import PhysxSchema
            for prim in stage.Traverse():
                api = PhysxSchema.PhysxArticulationAPI(prim)
                if api:
                    attr_pi = prim.GetAttribute("physxArticulation:solverPositionIterationCount")
                    if attr_pi: attr_pi.Set(pos_iters)
                    else: prim.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.UInt).Set(pos_iters)
                    attr_vi = prim.GetAttribute("physxArticulation:solverVelocityIterationCount")
                    if attr_vi: attr_vi.Set(vel_iters)
                    else: prim.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.UInt).Set(vel_iters)
            print(f"[Physics] Articulation solver iterations -> pos={pos_iters}, vel={vel_iters}")
        except Exception as _e:
            print(f"[Physics] Warning: Could not set articulation iterations: {_e}")

    # 应用配置中的摩擦力覆盖
    friction_cfgs = config.get("environment", {}).get("frictions", [])
    if friction_cfgs:
        try:
            from pxr import UsdPhysics
            _stage = omni.usd.get_context().get_stage()
            for fc in friction_cfgs:
                mat_name = fc.get("name", "")
                dyn = fc.get("dynamic_friction")
                sta = fc.get("static_friction")
                for prim in _stage.Traverse():
                    if prim.GetName() == mat_name:
                        mat_api = UsdPhysics.MaterialAPI(prim)
                        if not mat_api:
                            mat_api = UsdPhysics.MaterialAPI.Apply(prim)
                        if dyn is not None:
                            attr = prim.GetAttribute("physics:dynamicFriction")
                            if attr: attr.Set(float(dyn))
                            else: prim.CreateAttribute("physics:dynamicFriction", Sdf.ValueTypeNames.Float).Set(float(dyn))
                        if sta is not None:
                            attr = prim.GetAttribute("physics:staticFriction")
                            if attr: attr.Set(float(sta))
                            else: prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(float(sta))
                        print(f"[Friction] {mat_name} -> dynamic={dyn}, static={sta}")
                        break
        except Exception as _fe:
            print(f"[Friction] Warning: Could not apply friction overrides: {_fe}")

    # 通过防御式发现启用 ROS 2 和 Core 节点，以避免版本冲突
    import omni.kit.app
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    
    # 预扫描可用扩展
    available_exts = [e.get("id") for e in ext_manager.get_extensions()]
    def enable_preferred(variants):
        found = False
        for v in variants:
            if any(v in ex for ex in available_exts):
                if ext_manager.set_extension_enabled_immediate(v, True):
                    print(f"[Extensions] Enabled compatible variant: {v}")
                    found = True
                    # 在 6.0+ 等部分版本中，不同节点类型可能需要所有可用变体，
                    # 遇到这种情况时继续处理。
        return found

    # 为桥接和 Core 节点启用所有可用扩展。
    enable_preferred(["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge", "isaacsim.ros2.nodes", "isaacsim.core_nodes", "omni.isaac.core_nodes"])
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # ── 传感器话题重映射 ───────────────────────────────────────────────────────
    # 将 USD 中固化的所有传感器话题重映射为带车辆命名空间的话题。
    sensor_topics  = config.get("topics_to_remap", [])
    frame_ids      = config.get("frame_ids_to_remap", [])
    _frame_ids_set = set(frame_ids)

    for veh in vehicles:
        if not veh.get("enabled", True):
            continue
            
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        topic_prefix = veh.get("topic_prefix", f"/{veh_name.lower()}")
        topic_prefix = "/" + topic_prefix.strip("/") # 确保采用 /ego 格式
        ackermann_topic = topic_prefix + "/drive"
        
        remapped_count = 0
        veh_prim = stage.GetPrimAtPath(veh_prim_path)
        if not veh_prim.IsValid():
            continue
            
        # 仅遍历该车辆的 Prim 层级
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path):
                continue
            
            # 专门检查用于驾驶控制的 ROS 2 订阅节点
            if prim.GetTypeName() == "OmniGraphNode":
                try:
                    node_type = prim.GetAttribute("node:type").Get()
                    if node_type and "SubscribeAckermannDrive" in node_type:
                        topic_attr = prim.GetAttribute("inputs:topicName")
                        if topic_attr:
                            topic_attr.Set(ackermann_topic)
                            print(f"[ROS2 setup] {veh_name}: Drive topic -> '{ackermann_topic}'")
                except Exception: pass

            # ── 传感器话题重映射（穷举扫描）────────────────────────────────────
            # 重映射与已知传感器话题（imu、rgb、tf 等）匹配的所有字符串属性
            for attr in prim.GetAttributes():
                attr_name = attr.GetName().lower()
                # 跳过用于指定数据类型或配置常量的属性
                if "type" in attr_name or "format" in attr_name:
                    continue
                    
                val = attr.Get()
                if isinstance(val, str) and val.strip():
                    normalized = "/" + val.strip("/")
                    if normalized in sensor_topics:
                        new_val = topic_prefix + normalized
                        attr.Set(new_val)
                        remapped_count += 1
                        print(f"[ROS2 remap] {veh_name}: attr '{attr.GetName()}' | '{val}' -> '{new_val}'")
                    elif val in _frame_ids_set and "frameid" in attr_name:
                        new_val = topic_prefix.strip("/") + "/" + val
                        attr.Set(new_val)
                        remapped_count += 1
                        print(f"[ROS2 remap] {veh_name}: frame_id '{attr.GetName()}' | '{val}' -> '{new_val}'")
            
            # 重映射 ConstantString 的 node_namespace（ROS 2 节点将其用作 /tf 命名空间）
            # 这些节点通常具有一个作为命名空间字符串使用的输出 ``value``。
            ns_attr = prim.GetAttribute("inputs:value")
            if ns_attr:
                ns_val = ns_attr.Get()
                if isinstance(ns_val, str) and ns_val.strip("/") in [t.strip("/") for t in sensor_topics]:
                    new_ns = topic_prefix + "/" + ns_val.strip("/")
                    ns_attr.Set(new_ns)
                    remapped_count += 1
                    print(f"[ROS2 remap] {veh_name}: namespace '{ns_val}' -> '{new_ns}'")

            # ── 无人机相机/跟随路径修补 ─────────────────────────────────────
            # 如果 USD 中的跟随行为使用目标 Prim 的绝对路径，加入命名空间后路径会失效，
            # 因此在路径前添加前缀。
            for attr_name in ("inputs:target_prim", "inputs:targetPrim", "inputs:target"):
                target_attr = prim.GetAttribute(attr_name)
                if target_attr:
                    target_val = target_attr.Get()
                    # OmniGraph 目标属性通常是字符串或 Sdf.Path
                    if target_val and str(target_val).startswith("/") and not str(target_val).startswith(veh_prim_path):
                        new_target = f"{veh_prim_path}{str(target_val)}"
                        target_attr.Set(new_target if isinstance(target_val, str) else Sdf.Path(new_target))
                        remapped_count += 1
                        print(f"[Drone fix] {veh_name}: path '{target_val}' -> '{new_target}'")
        
        print(f"[ROS2 remap] {veh_name}: {remapped_count} sensor topic(s) remapped with prefix '{topic_prefix}'")

        # ── 跟随相机的创建现已推迟到下方的 ``BaseLink`` 发现阶段 ──


    # 强制同步执行物理计算，防止 PhysX 异步仿真时 ROS 2 OmniGraph 对严格物理变量
    #（如 getLinearVelocity）进行重叠读取。
    
    # 获取 Core 上下文前，让扩展和图完成初始化
    simulation_app.update()

    # Isaac Sim Full 启动时会自动添加 /Environment/defaultLight。
    # 将其移除，以确保仅启用环境 USD 中固化的 DomeLight。
    _stage_now = omni.usd.get_context().get_stage()
    for _dl_path in ("/Environment/defaultLight", "/World/Environment/defaultLight"):
        _dl_prim = _stage_now.GetPrimAtPath(_dl_path)
        if _dl_prim.IsValid():
            _stage_now.RemovePrim(_dl_path)
            print(f"[Stage] Removed auto-added defaultLight at {_dl_path}")
    
    # 通过底层 C++ Carb 配置无条件强制 PhysX 使用同步模式，
    # 从根本上避免 ROS 2 ActionGraph 节点计算速度时发生竞态冲突。
    import carb
    carb_settings = carb.settings.get_settings()
    carb_settings.set_bool("/physics/asyncFastSimulation", False)
    carb_settings.set_bool("/physics/updateToUsd", True)
    
    # 强制严格对齐步数限制，使物理 Tick 与渲染 Tick 完全同步
    app_freq = int(float(config.get("physics_settings", {}).get("time_steps_per_second", 60.0)))
    carb_settings.set_int("/persistent/simulation/minFrameRate", app_freq)
    carb_settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
    carb_settings.set_int("/app/runLoops/main/rateLimitFrequency", app_freq)

    # 抑制 ROS 2 TF 发布器产生的大量 [PoseTree] ``parent getObjectType eInvalid`` 警告。
    # Carb 日志在主循环上同步执行，因此失败的 TF 节点按 Tick 频率（×N 个目标坐标系）
    # 写日志会引入变化的逐步延迟，使实时因子不稳定。将该通道降为 Error，避免警告
    # 占用步进预算。注意：这只隐藏刷屏信息；底层 TF 树故障仍由下方 PoseTree 诊断流程
    # 报告，并且必须修复才能形成完整的 map->odom->base_link 链。
    carb_settings.set("/log/channels/isaacsim.ros2.nodes", "Error")

    # 此构建中 omni.isaac.core 已弃用，因此使用标准时间线执行循环
    import omni.timeline
    import omni.physx
    
    # 开始播放前强制 PhysX 引擎刷新所有待处理的异步更改
    omni.physx.get_physx_interface().force_load_physics_from_usd()
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # 使用经 ActionGraph 动态映射的原始 Carbon 输入层建立原生 UI 硬件遥控绑定
    #（不会产生 C 扩展与 Python 3.10 的冲突！）
    import carb.input
    import omni.appwindow
    import omni.graph.core as og
    import omni.timeline
    
    vehicle_teleop_publishers = {}  # 保存每辆车的 (ctrl_node, pub_node) 元数据
    
    # 从系统中识别合适的 ROS 2 发布器节点类型
    # 新版 Isaac Sim 使用 isaacsim.ros2.bridge，旧版/Omni 版本使用 omni.isaac.ros2_bridge
    pub_node_type = "isaacsim.ros2.bridge.ROS2PublishAckermannDrive"
    if og.get_node_type(pub_node_type) is None:
        pub_node_type = "omni.isaac.ros2_bridge.ROS2PublishAckermannDrive"
    
    print(f"[Teleop] Using ROS2 Publisher Node Type: {pub_node_type}")

    # 拦截每辆车的 ActionGraph，注入视口环回
    # 策略：找到现有 AckermannController 节点并直接驱动它，同时注入连接到现有
    # ros2_context 的 ROS 2 发布器，以便观察状态。
    for i, veh in enumerate(vehicles):
        veh_name = veh.get("name", f"Vehicle_{i}")
        _veh_prefix = "/" + veh.get("topic_prefix", f"/vehicle_{i}").strip("/")
        veh_topic = _veh_prefix + "/drive"
        if not veh.get("enabled", True): continue
            
        print(f"[Teleop] Processing {veh_name} for Loopback on {veh_topic}...")
        
        ackermann_ctrl_node = None
        ackermann_ctrl_path = ""
        ros2_context_path = ""
        subscribe_node_path = ""

        # 1. 发现阶段：查找 AckermannController、ros2_context 和订阅器节点。
        #    优先使用 OG 运行时，失败时回退到 USD Prim 属性；这样即使第二辆车的图
        #    在启动时尚未完整注册，也能找到两辆车。
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(f"/World/{veh_name}"): continue
            if prim.GetTypeName() != "OmniGraphNode": continue

            # 解析节点类型：先尝试 OG 运行时，再以 USD Prim 属性作为后备
            n_type = ""
            _og_node = None
            try:
                _og_node = og.get_node_by_path(p_path)
                if _og_node and _og_node.is_valid():
                    n_type = _og_node.get_node_type().get_node_type()
            except Exception: pass
            if not n_type:
                try: n_type = prim.GetAttribute("node:type").Get() or ""
                except Exception: pass

            if "AckermannController" in n_type:
                ackermann_ctrl_node = _og_node if (_og_node and _og_node.is_valid()) else og.get_node_by_path(p_path)
                ackermann_ctrl_path = p_path
            elif "ROS2Context" in n_type:
                ros2_context_path = p_path
            elif "SubscribeAckermannDrive" in n_type:
                subscribe_node_path = p_path

        # 通过遍历 USD 属性连接，查找驱动订阅器的 OnPlaybackTick（或 Gate）节点。
        # 该方法比 OG 运行时 API 更可靠，因为即使第二辆车的 OmniGraph 在启动时
        # 尚未注册到运行时中，它仍然有效。
        sub_tick_path = ""
        if subscribe_node_path:
            try:
                _sub_prim = stage.GetPrimAtPath(subscribe_node_path)
                _exec_in_attr = _sub_prim.GetAttribute("inputs:execIn")
                if _exec_in_attr:
                    for _src_path in _exec_in_attr.GetConnections():
                        _src_prim_path = _src_path.GetPrimPath()
                        _src_prim = stage.GetPrimAtPath(_src_prim_path)
                        if _src_prim.IsValid():
                            _src_ntype = _src_prim.GetAttribute("node:type").Get() or ""
                            if any(k in _src_ntype for k in ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")):
                                sub_tick_path = str(_src_prim_path)
                                print(f"[Teleop] {veh_name}: subscriber tick node found: {sub_tick_path}")
                                break
            except Exception as _ste:
                print(f"[Teleop] {veh_name}: could not resolve subscriber tick: {_ste}")

        if not ackermann_ctrl_node:
            print(f"[Teleop] WARNING: No AckermannController for {veh_name}. Skipping teleop setup.")
            continue

        try:
            graph_obj = ackermann_ctrl_node.get_graph()
            ackermann_graph_path = graph_obj.get_path_to_graph()
            ctrl_node_name = ackermann_ctrl_path.split("/")[-1]
            
            # 即将注入的节点名称
            tick_node_name = f"TeleopTick_{veh_name}_{i}"
            pub_node_name  = f"ViewportPublisher_{veh_name}_{i}"
            pub_tick_name  = f"PubTick_{veh_name}_{i}"

            # 2. 注入节点
            # 尝试查找可用的 ReadSimulationTime 类型
            time_node_type = "omni.isaac.core_nodes.IsaacReadSimulationTime"
            # 在某些 5.x 版本中，它可能是 isaacsim.core_nodes.IsaacReadSimulationTime
            # 尝试创建图节点，并妥善处理失败情况
            
            create_cmds = [
                (tick_node_name, "omni.graph.action.OnPlaybackTick"),
                (pub_tick_name, "omni.graph.action.OnPlaybackTick"),
                (pub_node_name, pub_node_type),
            ]
            set_cmds = [
                (f"{pub_node_name}.inputs:topicName", veh_topic),
                (f"{pub_node_name}.inputs:frameId", "base_link"),
            ]
            connect_cmds = [
                (f"{tick_node_name}.outputs:tick", f"{ctrl_node_name}.inputs:execIn"),
                (f"{pub_tick_name}.outputs:tick", f"{pub_node_name}.inputs:execIn"),
            ]
            if ros2_context_path:
                ctx_node_name = ros2_context_path.split("/")[-1]
                connect_cmds.append((f"{ctx_node_name}.outputs:context", f"{pub_node_name}.inputs:context"))

            # 3. 首先保存控制元数据——即使下方节点注入失败，此步骤也必须成功，
            #    以便车辆进入控制循环，并允许 og.Controller.set() 覆盖订阅器。
            _monitor_topic = veh_topic.rstrip("/") + "_teleop"
            _muted_topic   = veh_topic.rstrip("/") + "_muted"
            # 查找订阅器→控制器的 OmniGraph 数据连接。这些连接携带最后收到的
            # ROS 2 值，即使订阅器的 Tick 已禁用，也始终会覆盖 og.Controller.set()
            # 写入的值。唯一可靠的修复是在进入 TELEOP 时断开连接，切回 ROS 2 时重连。
            _sub_ctrl_conns = []  # (src_attr_path, dst_attr_path) 列表
            if subscribe_node_path and ackermann_ctrl_path:
                _ctrl_prim = stage.GetPrimAtPath(ackermann_ctrl_path)
                for _inp in ("inputs:speed", "inputs:steeringAngle"):
                    _usd_attr = _ctrl_prim.GetAttribute(_inp)
                    if not _usd_attr:
                        continue
                    for _src in _usd_attr.GetConnections():
                        if str(_src.GetPrimPath()) == subscribe_node_path:
                            _sub_ctrl_conns.append((str(_src), f"{ackermann_ctrl_path}.{_inp}"))
            if _sub_ctrl_conns:
                print(f"[Teleop] {veh_name}: found {len(_sub_ctrl_conns)} subscriber→controller connection(s) to manage")

            vehicle_teleop_publishers[veh_name] = {
                "ctrl_attr_speed": f"{ackermann_graph_path}/{ctrl_node_name}.inputs:speed",
                "ctrl_attr_steer": f"{ackermann_graph_path}/{ctrl_node_name}.inputs:steeringAngle",
                "pub_node_path":  f"{ackermann_graph_path}/{pub_node_name}",
                "pub_topic_attr": f"{ackermann_graph_path}/{pub_node_name}.inputs:topicName",
                "drive_topic":    veh_topic,
                "monitor_topic":  _monitor_topic,
                "sub_node_path":  subscribe_node_path,
                "sub_tick_path":  sub_tick_path,
                "sub_topic_attr": f"{subscribe_node_path}.inputs:topicName" if subscribe_node_path else None,
                "muted_topic":    _muted_topic,
                "sub_ctrl_connections": _sub_ctrl_conns,
            }
            print(f"[Teleop] Control paths registered for {veh_name} (graph: {ackermann_graph_path})")

            # 4. 注入用于观测的发布器节点——这不是关键路径；即使失败也不会阻止遥控，
            #    因为车辆已经进入控制循环。
            try:
                og.Controller.edit(graph_obj, {
                    og.Controller.Keys.CREATE_NODES: create_cmds,
                    og.Controller.Keys.SET_VALUES: set_cmds,
                })

                # 可选的仿真时间节点，用于生成准确的消息时间戳
                time_node_name = f"ReadSimTime_{veh_name}_{i}"
                try:
                    og.Controller.edit(graph_obj, {
                        og.Controller.Keys.CREATE_NODES: [(time_node_name, time_node_type)],
                    })
                    connect_cmds.append((f"{time_node_name}.outputs:simulationTime", f"{pub_node_name}.inputs:timeStamp"))
                except Exception:
                    try:
                        og.Controller.edit(graph_obj, {
                            og.Controller.Keys.CREATE_NODES: [(time_node_name, "isaacsim.core_nodes.IsaacReadSimulationTime")],
                        })
                        connect_cmds.append((f"{time_node_name}.outputs:simulationTime", f"{pub_node_name}.inputs:timeStamp"))
                    except Exception:
                        print(f"[Teleop] Warning: Could not create IsaacReadSimulationTime for {veh_name}.")

                # 连线（单独执行以处理连接冲突）
                for src, dst in connect_cmds:
                    try:
                        og.Controller.connect(og.Controller.attribute(f"{ackermann_graph_path}/{src}"),
                                              og.Controller.attribute(f"{ackermann_graph_path}/{dst}"))
                    except Exception:
                        if "execIn" in dst: pass
                        else: print(f"[Teleop] Warning: Could not connect {src} -> {dst}")

                # 将发布器路由到监控话题，使 drive_topic 可供外部 ROS 2 使用
                try:
                    og.Controller.set(
                        og.Controller.attribute(f"{ackermann_graph_path}/{pub_node_name}.inputs:topicName"),
                        _monitor_topic,
                    )
                    print(f"[Teleop] Publisher routed to '{_monitor_topic}' for {veh_name}.")
                except Exception as _rr_e:
                    print(f"[Teleop] Warning: Could not set publisher topic for {veh_name}: {_rr_e}")

            except Exception as _inj_e:
                print(f"[Teleop] Warning: Publisher node injection failed for {veh_name}: {_inj_e}")
                print(f"[Teleop] Control (speed/steer override) still active for {veh_name}.")

        except Exception as e:
            print(f"[Teleop] ERROR during setup for {veh_name}: {e}")

    if not vehicle_teleop_publishers:
        print(f"[Teleop] CRITICAL ERROR: No control nodes registered!")
    else:
        print(f"[Teleop] Successfully initialized {len(vehicle_teleop_publishers)} vehicle control path(s).")

    # ── 传感器图门控 ────────────────────────────────────────────────────────────
    # 阻止每辆车中已禁用的传感器发布 ROS 2 数据。
    # 策略：在执行源节点（OnPlaybackTick / IsaacSimulationGate）上设置
    # inputs:enabled = False 以停止整个图，或在各个相机/LiDAR 发布器节点上设置，
    # 以实现局部禁用。不使用 USD SetActive()——OmniGraph 运行时会在加载后缓存图，
    # 并忽略运行时对 Prim 激活状态的更改。
    _CAM_NODE_KW  = ("CameraHelper", "PublishImage", "PublishRgb", "RgbSensor")
    _LID_NODE_KW  = ("Lidar", "RTXLidar", "PointCloud", "PublishPointCloud", "LaserScan")
    _TICK_NODE_KW = ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")

    def _sg_set_enabled(node_path, enabled):
        """设置 OmniGraph 节点的 inputs:enabled；成功时返回 True。"""
        try:
            og.Controller.set(og.Controller.attribute(f"{node_path}.inputs:enabled"), enabled)
            return True
        except Exception:
            return False

    for _sg_veh in vehicles:
        if not _sg_veh.get("enabled", True): continue
        _sg_name   = _sg_veh.get("name", "")
        _sg_en_cam = _sg_veh.get("enable_camera", True)
        _sg_en_lid = _sg_veh.get("enable_lidar",  True)
        if _sg_en_cam and _sg_en_lid:
            continue  # 两者都已启用，无需门控

        _sg_base = f"/World/{_sg_name}"
        for _sg_prim in stage.Traverse():
            _sg_pp = _sg_prim.GetPath().pathString
            if not _sg_pp.startswith(_sg_base):
                continue
            if _sg_prim.GetTypeName() != "OmniGraphNode":
                continue
            _sg_node_name = _sg_prim.GetName().lower()
            if not _sg_en_cam and "camera_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled camera helper '{_sg_pp}'")
            elif not _sg_en_lid and "lidar_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled lidar helper '{_sg_pp}'")

    # ── ROS 2 驾驶桥接（AckermannDriveStamped + autoware_control_msgs/Control）──
    # Isaac Sim 使用 Python 3.12，但 ROS Humble 中的 rclpy 为 Python 3.10 编译。
    # 按照 gnss_bridge.py 的相同模式，将订阅委托给 Python 3.10 子进程
    #（drive_bridge.py）。指令通过子进程标准输出到达并存入 _drive_cmds，供物理循环使用；
    # 场景预热后，地图和 TF 数据通过标准输入发送给子进程。
    _ros_bridge_enabled = False
    _drive_proc         = None   # subprocess.Popen 句柄
    _drive_cmds         = {}     # 映射：veh_name -> {speed, steer, stamp, source}
    _drive_log          = None
    try:
        import subprocess  as _drv_sub
        import os          as _drv_os
        import time        as _drv_time
        import threading   as _drv_threading

        _drv_script = _drv_os.path.join(
            _drv_os.path.dirname(_drv_os.path.abspath(__file__)),
            "drive_bridge.py")
        _drv_env = {
            "HOME":               _drv_os.environ.get("HOME", "/root"),
            "USER":               _drv_os.environ.get("USER", "root"),
            "PATH":               "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
            "PYTHONPATH":         ("/opt/ros/humble/lib/python3.10/site-packages:"
                                   "/opt/ros/humble/local/lib/python3.10/dist-packages"),
            "AMENT_PREFIX_PATH":  "/opt/ros/humble",
            "LD_LIBRARY_PATH":    "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu",
            "ROS_DOMAIN_ID":      _drv_os.environ.get("ROS_DOMAIN_ID", "0"),
            # 从父进程继承 RMW，使 drive_bridge 与仿真器宿主使用相同的中间件。
            # 正常启动路径中父进程会设置该值，此时默认采用 rmw_cyclonedds_cpp。
            **({"RMW_IMPLEMENTATION": _drv_os.environ["RMW_IMPLEMENTATION"]}
               if "RMW_IMPLEMENTATION" in _drv_os.environ else {}),
            # 若已设置网卡绑定，则将其传递给子进程（适用于多网卡主机）
            **({"CYCLONEDDS_URI": _drv_os.environ["CYCLONEDDS_URI"]}
               if "CYCLONEDDS_URI" in _drv_os.environ else {}),
        }

        _drv_log_path = _drv_os.path.join(
            _drv_os.path.dirname(_drv_os.path.abspath(__file__)),
            "drive_bridge.log")
        _drive_log = open(_drv_log_path, "w")
        _drive_proc = _drv_sub.Popen(
            ["/usr/bin/python3.10", _drv_script],
            stdin=_drv_sub.PIPE,
            stdout=_drv_sub.PIPE,
            stderr=_drive_log,
            text=True,
            env=_drv_env,
        )

        # 发送车辆订阅注册信息，并保存记录以供重启使用。
        _drv_sub_cmds = []
        for _veh in vehicles:
            if not _veh.get("enabled", True): continue
            _vn = _veh.get("name", "Vehicle")
            _vp = "/" + _veh.get("topic_prefix", f"/{_vn.lower()}").strip("/")
            _cmd = f"sub\t{_vn}\t{_vp}/drive\t{_vp}/control\n"
            _drv_sub_cmds.append(_cmd)
            _drive_proc.stdin.write(_cmd)
        _drive_proc.stdin.write("start\n")
        _drive_proc.stdin.flush()

        # 等待桥接程序发出就绪信号。
        _drv_ready = _drive_proc.stdout.readline().strip()
        _drive_log.flush()
        if _drv_ready != "ready":
            raise RuntimeError(
                f"drive_bridge.py did not start correctly (got: {_drv_ready!r}); "
                f"see {_drv_log_path} for details")

        # 后台线程：从子进程标准输出读取驾驶指令。将进程句柄作为参数传入，
        # 使线程始终绑定到启动它的特定进程，从而可以安全重启桥接程序。
        def _drive_reader(_proc):
            for _dl in _proc.stdout:
                _dl = _dl.strip()
                if not _dl.startswith("cmd\t"):
                    continue
                try:
                    _, _dvn, _spd, _str, _src = _dl.split("\t")
                    _drive_cmds[_dvn] = {
                        "speed":  float(_spd),
                        "steer":  float(_str),
                        "stamp":  _drv_time.monotonic(),
                        "source": _src,
                    }
                except Exception:
                    pass

        _drv_threading.Thread(target=_drive_reader, args=(_drive_proc,), daemon=True).start()
        _ros_bridge_enabled = True
        _drv_post_cmds = []   # 地图和静态 TF 命令，重启时重新发送给新桥接进程
        print("[ROS2 Bridge] Drive bridge ready — AckermannDriveStamped on /drive, "
              "autoware_control_msgs/Control on /control")
    except Exception as _bridge_err:
        print(f"[ROS2 Bridge] Could not initialize drive bridge: {_bridge_err}")

    # ── 全局时间戳修复 ─────────────────────────────────────────────────────────
    # 在 timeline.play() 之后运行，确保 OmniGraph 运行时已完全初始化。
    # 使用 node.get_attributes() 安全枚举；直接对不存在的属性调用
    # node.get_attribute() 会在日志中触发 C++ 插件错误。
    #
    # 即使 USD 中固化的实例能在加载时正常工作，IsaacReadSimulationTime 扩展也可能
    # 无法通过带硬编码类型名的 CREATE_NODES 加载，因为扩展未注册为可动态创建。
    # 解决方案：从现有活动实例中发现确切的已注册类型。
    _sim_time_node_type = None
    for _p in stage.Traverse():
        if _p.GetTypeName() != "OmniGraphNode":
            continue
        try:
            _n = og.get_node_by_path(_p.GetPath().pathString)
            if not _n.is_valid():
                continue
            _ntype = _n.get_node_type().get_node_type()
            if "ReadSimulationTime" in _ntype:
                _sim_time_node_type = _ntype
                print(f"[Timestamp Fix] Discovered sim time node type: {_ntype}")
                break
        except Exception:
            pass
    if _sim_time_node_type is None:
        print("[Timestamp Fix] WARNING: Could not discover IsaacReadSimulationTime type — timestamps may remain 0")

    _graphs_needing_fix = {}  # 映射：graph_path_str -> (graph_obj, [node_local.inputs:timeStamp, ...])
    for prim in stage.Traverse():
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        try:
            node = og.get_node_by_path(prim.GetPath().pathString)
            if not node.is_valid():
                continue
            # 安全枚举属性，避免 C++ 的“找不到属性”错误
            attr_names = {a.get_name() for a in node.get_attributes()}
            if "inputs:timeStamp" not in attr_names:
                continue
            ts_attr = node.get_attribute("inputs:timeStamp")
            if ts_attr.get_upstream_connection_count() > 0:
                continue  # 已连接驱动源
            graph_obj = node.get_graph()
            graph_path = graph_obj.get_path_to_graph()
            node_local = prim.GetPath().pathString.split("/")[-1]
            if graph_path not in _graphs_needing_fix:
                _graphs_needing_fix[graph_path] = (graph_obj, [])
            _graphs_needing_fix[graph_path][1].append(f"{node_local}.inputs:timeStamp")
            print(f"[Timestamp Fix] Found undriven timeStamp on: {prim.GetPath().pathString}")
        except Exception:
            pass

    _ts_fix_node_name = "GlobalSimTimeReader"
    for graph_path, (graph_obj, dst_attrs) in _graphs_needing_fix.items():
        # 步骤 1：查找图中已有的 IsaacReadSimulationTime 节点，例如遥控功能注入的
        # ReadSimTime_*_* 或已固化的实例。优先复用而不是新建，因为某些构建中的
        # 动态 CREATE_NODES 可能失败。
        time_src_node = None
        for _p in stage.Traverse():
            if not _p.GetPath().pathString.startswith(graph_path + "/"):
                continue
            if _p.GetTypeName() != "OmniGraphNode":
                continue
            try:
                _n = og.get_node_by_path(_p.GetPath().pathString)
                if _n.is_valid() and "ReadSimulationTime" in _n.get_node_type().get_node_type():
                    time_src_node = _n
                    print(f"[Timestamp Fix] Using existing sim time node: {_p.GetPath().pathString}")
                    break
            except Exception:
                pass

        # 步骤 2：如果没有现有节点，尝试创建一个；失败时记录错误
        if time_src_node is None:
            for _type in (_sim_time_node_type, "isaacsim.core_nodes.IsaacReadSimulationTime",
                          "omni.isaac.core_nodes.IsaacReadSimulationTime"):
                try:
                    og.Controller.edit(graph_obj, {
                        og.Controller.Keys.CREATE_NODES: [(_ts_fix_node_name, _type)],
                    })
                    _candidate = og.get_node_by_path(f"{graph_path}/{_ts_fix_node_name}")
                    if _candidate and _candidate.is_valid():
                        time_src_node = _candidate
                        print(f"[Timestamp Fix] Created sim time node ({_type}) in {graph_path}")
                        break
                except Exception as _ce:
                    print(f"[Timestamp Fix] CREATE_NODES({_type}) failed for {graph_path}: {_ce}")

        if time_src_node is None or not time_src_node.is_valid():
            print(f"[Timestamp Fix] No sim time source available for {graph_path} — skipping")
            continue

        # 步骤 3：通过直接节点/属性引用建立连接，避免使用 og.Controller.attribute
        # 的路径字符串解析；若上方节点未成功创建，该解析会失败
        src_attr = time_src_node.get_attribute("outputs:simulationTime")
        if not src_attr.is_valid():
            print(f"[Timestamp Fix] outputs:simulationTime not found on time node in {graph_path}")
            continue

        for dst in dst_attrs:
            node_local = dst.split(".")[0]
            try:
                dst_node = og.get_node_by_path(f"{graph_path}/{node_local}")
                if not dst_node.is_valid():
                    continue
                dst_attr = dst_node.get_attribute("inputs:timeStamp")
                if not dst_attr.is_valid():
                    continue
                og.Controller.connect(src_attr, dst_attr)
                print(f"[Timestamp Fix] Connected sim time -> {graph_path}/{dst}")
            except Exception as _e:
                print(f"[Timestamp Fix] Warning: Could not connect -> {graph_path}/{dst}: {_e}")

    # ── OmniGraph 传感器话题重映射（播放后阶段）────────────────────────────────
    # 播放前的 USD 扫描只能捕获已写入的属性（prim.GetAttributes()）。
    # 若 OmniGraph 节点的 inputs:topicName 从未在 USD 中显式设置，它会使用默认值
    #（如 "imu"、"rgb"），而 USD 扫描无法看到这些值。timeline.play() 之后，
    # OmniGraph 运行时已激活，node.get_attributes() 会暴露包括默认值在内的所有输入，
    # 因此本阶段可捕获之前遗漏的属性。
    _sensor_topics_set = set(sensor_topics)
    # 收集坐标系 ID 重映射的 (node_path, attr_name, value) 元组，以便每个 Tick
    # 重新应用。图重新求值或建立新连接等事件后，OmniGraph 节点可能把已写入属性
    # 重置为默认值，因此仅在播放后写入一次并不可靠。
    _og_frame_id_remaps = []   # 元组列表：[(node_path_str, og_attr_name_str, new_val_str), ...]
    for veh in vehicles:
        if not veh.get("enabled", True):
            continue
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        topic_prefix = "/" + veh.get("topic_prefix", f"/{veh_name.lower()}").strip("/")

        _og_remap_count = 0
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                continue
            try:
                node = og.get_node_by_path(p_path)
                if not node.is_valid():
                    continue
                og_attr_names = {a.get_name() for a in node.get_attributes()}
                for candidate in ("inputs:topicName", "inputs:nodeNamespace"):
                    if candidate not in og_attr_names:
                        continue
                    if "type" in candidate.lower() or "format" in candidate.lower():
                        continue
                    # 先尝试 OG attr.get()，失败时回退到 USD Prim 属性
                    val = None
                    og_attr = node.get_attribute(candidate)
                    for _getter in (lambda: og_attr.get(),
                                    lambda: prim.GetAttribute(candidate).Get()):
                        try:
                            v = _getter()
                            if isinstance(v, str) and v.strip():
                                val = v
                                break
                        except Exception:
                            pass
                    if not isinstance(val, str) or not val.strip():
                        continue
                    normalized = "/" + val.strip("/")
                    if normalized not in _sensor_topics_set:
                        continue
                    if val.startswith(topic_prefix):
                        continue  # 已由播放前流程完成重映射
                    new_val = topic_prefix + normalized
                    # 通过可靠的 USD Prim API 写入，同时也尝试写入 OG 属性
                    prim.GetAttribute(candidate).Set(new_val)
                    try:
                        og_attr.set(new_val)
                    except Exception:
                        pass
                    _og_remap_count += 1
                    print(f"[ROS2 remap OG] {veh_name}: {prim.GetPath().GetName()}.{candidate} "
                          f"'{val}' -> '{new_val}'")
            except Exception:
                pass
        if _og_remap_count > 0:
            print(f"[ROS2 remap OG] {veh_name}: {_og_remap_count} additional topic(s) remapped post-play")

        # ── 坐标系 ID 重映射（播放后的 OG 阶段）───────────────────────────────
        # 扫描名称中包含 "FrameId" 的所有 OmniGraph 节点属性，将基础值
        #（"odom"、"base_link"）重映射为带命名空间的值（"ego/odom"）。
        if _frame_ids_set:
            _pfx_ns = topic_prefix.strip("/")  # 例如 "ego"
            _fid_count = 0
            for prim in stage.Traverse():
                p_path = prim.GetPath().pathString
                if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                    continue
                try:
                    node = og.get_node_by_path(p_path)
                    if not node.is_valid(): continue
                    for _oa in node.get_attributes():
                        _aname = _oa.get_name()
                        if "frameid" not in _aname.lower(): continue
                        try:
                            _fval = _oa.get()
                            if not isinstance(_fval, str) or not _fval.strip(): continue
                            if _fval not in _frame_ids_set: continue
                            if _fval.startswith(_pfx_ns + "/"): continue
                            _new_fval = _pfx_ns + "/" + _fval
                            prim.GetAttribute(_aname).Set(_new_fval)
                            try: _oa.set(_new_fval)
                            except Exception: pass
                            _fid_count += 1
                            # 保存记录以便每个 Tick 重新应用，因为图事件后 OG 节点
                            # 可能把已写入值重置为默认值。
                            _og_frame_id_remaps.append((p_path, _aname, _new_fval))
                            print(f"[ROS2 remap OG] {veh_name}: {prim.GetPath().GetName()}.{_aname} "
                                  f"'{_fval}' -> '{_new_fval}'")
                        except Exception: pass
                except Exception: pass
            if _fid_count > 0:
                print(f"[ROS2 remap OG] {veh_name}: {_fid_count} frame ID(s) remapped post-play")

        # ── PoseTree（TF 发布器）parentPrim 有效性诊断 ─────────────────────────
        # 大量出现 ``[PoseTree] parent getObjectType eInvalid``，表示
        # ROS2PublishTransformTree 节点在求值时无法解析 parentPrim/targetPrims，
        # 因而不会发布任何 base_link->传感器变换，下游 TF 链
        #（map->odom->base_link->传感器）会断开。常见原因是 Prim 位于可实例化引用内部
        #（内部 Prim 无法解析），或它不是 Xformable。报告确切 Prim 类型和可实例化状态，
        # 以便修复该节点。
        def _pt_prim_report(_pp):
            _pp = str(_pp)
            _pr = stage.GetPrimAtPath(_pp)
            if not _pr or not _pr.IsValid():
                return f"{_pp}  ->  MISSING (no prim at path)"
            # 向上遍历祖先节点，标记会对 Fabric/PhysX 隐藏内部 Prim 并产生 eInvalid
            # 的所有可实例化父节点。
            _anc = _pr.GetParent()
            _anc_inst = None
            while _anc and _anc.IsValid() and _anc.GetPath().pathString not in ("/", ""):
                if _anc.IsInstanceable():
                    _anc_inst = _anc.GetPath().pathString
                    break
                _anc = _anc.GetParent()
            return (f"{_pp}  type={_pr.GetTypeName() or '<none>'}  "
                    f"instanceable={_pr.IsInstanceable()}  "
                    f"instanceProxy={_pr.IsInstanceProxy()}  "
                    f"instanceableAncestor={_anc_inst or 'none'}")

        def _pt_rel_targets(_prim, _name):
            _r = _prim.GetRelationship(_name)
            if _r:
                _tg = _r.GetTargets()
                if _tg:
                    return list(_tg)
            _a = _prim.GetAttribute(_name)
            if _a:
                try:
                    _v = _a.Get()
                    if _v:
                        return list(_v) if isinstance(_v, (list, tuple)) else [_v]
                except Exception:
                    pass
            return []

        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                continue
            _nt_attr = prim.GetAttribute("node:type")
            _ntype = _nt_attr.Get() if _nt_attr else ""
            if "PublishTransformTree" not in (_ntype or ""):
                continue
            print(f"[PoseTree diag] {veh_name}: node {p_path} (type={_ntype})")
            _parents = _pt_rel_targets(prim, "inputs:parentPrim")
            if not _parents:
                print(f"[PoseTree diag]   parentPrim: <none set>")
            for _pp in _parents:
                print(f"[PoseTree diag]   parentPrim: {_pt_prim_report(_pp)}")
            _tgts = _pt_rel_targets(prim, "inputs:targetPrims")
            print(f"[PoseTree diag]   {len(_tgts)} targetPrim(s)")
            for _pp in _tgts:
                print(f"[PoseTree diag]   target: {_pt_prim_report(_pp)}")

    # ── 播放后禁用传感器辅助节点（此时 OG 运行时已完全初始化）────────────────
    for _sg_veh in vehicles:
        if not _sg_veh.get("enabled", True): continue
        _sg_name   = _sg_veh.get("name", "")
        _sg_en_cam = _sg_veh.get("enable_camera", True)
        _sg_en_lid = _sg_veh.get("enable_lidar",  True)
        if _sg_en_cam and _sg_en_lid:
            continue
        _sg_base = f"/World/{_sg_name}"
        for _sg_prim in stage.Traverse():
            _sg_pp = _sg_prim.GetPath().pathString
            if not _sg_pp.startswith(_sg_base):
                continue
            if _sg_prim.GetTypeName() != "OmniGraphNode":
                continue
            _sg_node_name = _sg_prim.GetName().lower()
            if not _sg_en_cam and "camera_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled camera helper '{_sg_pp}' (post-play)")
            elif not _sg_en_lid and "lidar_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled lidar helper '{_sg_pp}' (post-play)")

    # ── 地图生成与发布 ─────────────────────────────────────────────────────────
    # 渲染一帧俯视正交语义分割图，根据环境网格构建 nav_msgs/OccupancyGrid，
    # 然后使用 TRANSIENT_LOCAL（锁存）QoS 发布到 /map，使延迟加入的订阅器也能收到。
    # 同时发布静态 map→odom 单位变换，使 TF 树完整：map → odom → base_link。
    _map_cfg     = config.get("map_server", {})
    _map_enabled = _map_cfg.get("enabled", False)

    if _map_enabled and _ros_bridge_enabled and _drive_proc:
        try:
            import numpy as _map_np
            import omni.replicator.core as _map_rep
            import struct  as _map_struct
            import base64  as _map_b64
            from pxr import UsdGeom as _MapUG, Gf as _MapGf, Usd as _MapUsd

            _map_res   = float(_map_cfg.get("resolution", 0.05))
            _map_stage = omni.usd.get_context().get_stage()

            # 若尚未分配，则为环境网格指定语义标签。seg_id_keywords、seg_default_id
            # 和 seg_enabled 在后面的分割设置块中定义；使用 locals().get() 处理
            # 地图部分先于该设置块运行的情况。
            _map_id_kws = locals().get("seg_id_keywords") or {0: ["default"], 1: ["track"]}
            _map_def_id = locals().get("seg_default_id", 0)

            def _map_assign_label(prim, label_str):
                try:
                    from omni.isaac.core.utils.semantics import add_update_semantics
                    add_update_semantics(prim, label_str, type_label="class"); return
                except Exception: pass
                try:
                    from pxr import Semantics as _SA
                    _s = _SA.SemanticsAPI.Apply(prim, "Semantics")
                    _s.CreateSemanticTypeAttr().Set("class")
                    _s.CreateSemanticDataAttr().Set(label_str)
                except Exception: pass

            if not locals().get("seg_enabled", False):
                _lbl_n = 0
                for _mp in _map_stage.Traverse():
                    if _mp.GetTypeName() != "Mesh": continue
                    if not _mp.GetPath().pathString.startswith("/World/Environment"): continue
                    _mn = _mp.GetName().lower()
                    _aid = _map_def_id
                    for _mid, _mkws in _map_id_kws.items():
                        if "default" in _mkws: continue
                        if any(_kw.lower() in _mn for _kw in _mkws):
                            _aid = _mid; break
                    _map_assign_label(_mp, str(_aid))
                    _lbl_n += 1
                if _lbl_n:
                    print(f"[Map] Assigned semantic labels to {_lbl_n} environment meshes")

            # 计算环境包围盒（世界坐标，保留 5% 边距）。
            _map_env = _map_stage.GetPrimAtPath("/World/Environment")
            _map_bbc = _MapUG.BBoxCache(_MapUsd.TimeCode.Default(), ["default", "render"])
            _map_br  = _map_bbc.ComputeWorldBound(_map_env).GetRange()
            _map_mn_pt, _map_mx_pt = _map_br.GetMin(), _map_br.GetMax()
            _map_ex  = (_map_mx_pt[0] - _map_mn_pt[0]) * 1.05   # X 方向范围加边距
            _map_ey  = (_map_mx_pt[1] - _map_mn_pt[1]) * 1.05   # Y 方向范围加边距
            _map_cx  = (_map_mn_pt[0] + _map_mx_pt[0]) / 2
            _map_cy  = (_map_mn_pt[1] + _map_mx_pt[1]) / 2
            _map_cz  = float(_map_mx_pt[2]) + 20.0               # 位于最高点上方 20 米
            _map_orig_x = _map_cx - _map_ex / 2
            _map_orig_y = _map_cy - _map_ey / 2
            _map_pw  = min(4096, max(64, int(_map_ex / _map_res)))
            _map_ph  = min(4096, max(64, int(_map_ey / _map_res)))

            # 创建俯视正交相机。
            # aperture 的单位是“场景单位的十分之一”（Stage 单位为米时即厘米）。
            _map_cp  = "/World/_MapCamera"
            _map_cam = _MapUG.Camera.Define(_map_stage, _map_cp)
            _map_cam.GetProjectionAttr().Set(_MapUG.Tokens.orthographic)
            _map_cam.GetHorizontalApertureAttr().Set(_map_ex * 10.0)
            _map_cam.GetVerticalApertureAttr().Set(_map_ey * 10.0)
            _map_cam.GetClippingRangeAttr().Set(
                _MapGf.Vec2f(0.1, _map_cz + abs(float(_map_mn_pt[2])) + 10.0))
            _map_xf = _MapUG.Xformable(_map_cam)
            _map_xf.ClearXformOpOrder()
            # 单位旋转：相机朝向 -Z（在 Z 轴向上的世界中垂直向下）。
            _map_xf.AddTranslateOp().Set(_MapGf.Vec3d(_map_cx, _map_cy, _map_cz))

            # 挂载语义标注器，预热渲染器，然后采集。
            _map_rp     = _map_rep.create.render_product(_map_cp, (_map_pw, _map_ph))
            _map_annot  = _map_rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"colorize": False})
            _map_annot.attach(_map_rp)
            print(f"[Map] Top-down camera ({_map_cx:.1f},{_map_cy:.1f},{_map_cz:.1f}), "
                  f"coverage {_map_ex:.1f}×{_map_ey:.1f} m → {_map_pw}×{_map_ph} px")
            for _ in range(10):
                simulation_app.update()

            _map_sd  = _map_annot.get_data()
            _map_i2l = _map_sd.get("info", {}).get("idToLabels", {})
            _map_arr = _map_sd.get("data")   # 形状为 (H, W) 的 uint32 数组

            if _map_arr is not None and _map_arr.size > 0:
                # 将 Replicator ID 映射为占据值（0=空闲，100=占据，-1=未知）。
                _map_l2v = {}
                for _rid, _li in _map_i2l.items():
                    try:
                        _oc = int(_li.get("class", str(_map_def_id)))
                        _map_l2v[int(_rid)] = 0 if _oc != _map_def_id else 100
                    except (ValueError, TypeError):
                        _map_l2v[int(_rid)] = -1
                _map_occ = _map_np.vectorize(
                    lambda v: _map_l2v.get(int(v), -1), otypes=[_map_np.int8])(_map_arr)
                # 图像第 0 行对应世界 max_Y，而 OccupancyGrid 第 0 行对应世界 min_Y，因此需翻转。
                _map_occ = _map_np.flipud(_map_occ)

                # 将占据数据序列化为 Base64 打包的有符号字节，并发送给 drive_bridge.py
                # 子进程，由其通过 rclpy 发布。
                _map_flat = _map_occ.flatten().tolist()
                _map_raw  = _map_struct.pack(f"{len(_map_flat)}b", *_map_flat)
                _map_db64 = _map_b64.b64encode(_map_raw).decode("ascii")
                _map_cmd = (f"map\t{int(_map_pw)}\t{int(_map_ph)}\t{_map_res}"
                            f"\t{float(_map_orig_x)}\t{float(_map_orig_y)}\t{_map_db64}\n")
                _drive_proc.stdin.write(_map_cmd)
                _drive_proc.stdin.flush()
                _drv_post_cmds.append(_map_cmd)
                print(f"[Map] Sent /map to bridge: {_map_pw}×{_map_ph} cells @ {_map_res} m/cell")

                # 为每辆已启用车辆发布一个静态 map→{prefix}/odom 单位变换。
                _odom_frames = []
                for _mv in vehicles:
                    if not _mv.get("enabled", True): continue
                    _mvpfx = "/" + _mv.get("topic_prefix",
                                          f"/{_mv.get('name','').lower()}").strip("/")
                    _mv_odom_frame = _mvpfx.strip("/") + "/odom"
                    _tf_cmd = f"tf\tmap\t{_mv_odom_frame}\n"
                    _drive_proc.stdin.write(_tf_cmd)
                    _drv_post_cmds.append(_tf_cmd)
                    _odom_frames.append(_mv_odom_frame)
                _drive_proc.stdin.flush()
                print(f"[Map] Sent static map→odom TFs to bridge: {', '.join(_odom_frames)}")
            else:
                print("[Map] WARNING: semantic annotator returned no data — /map not published")

            try: _map_annot.detach(_map_rp)
            except Exception: pass
            try: _map_rp.destroy()
            except Exception: pass
            try: _map_stage.RemovePrim(_map_cp)
            except Exception: pass
        except Exception as _map_err:
            import traceback as _map_tb
            print(f"[Map] Error during map generation: {_map_err}")
            _map_tb.print_exc()

    # 预先初始化 GNSS 重启变量，使主循环即使在 GNSS 已禁用或其设置块尚未运行时，
    # 也能安全检查这些变量。
    _gnss_proc   = None
    _gnss_script = None
    _gnss_env    = None
    _gnss_log    = None

    # ── GNSS 传感器设置 ────────────────────────────────────────────────────────
    # 每帧从 USD 读取各车辆 base_link 的世界位置，以配置的地图原点为基准，
    # 通过等距圆柱投影转换为 WGS-84 坐标，加入可配置的高斯噪声，
    # 并按要求的频率在 /{topic_prefix}/gnss 上发布 sensor_msgs/NavSatFix。
    _gnss_cfg         = config.get("gnss", {})
    _gnss_enabled     = _gnss_cfg.get("enabled", False)
    _gnss_publishers  = {}   # 映射：veh_name -> rclpy Publisher<NavSatFix>
    _gnss_bridge      = None  # 用于创建发布器和提供时钟的 rclpy 节点
    _gnss_frame_skip  = 1
    _gnss_lat0 = _gnss_lon0 = _gnss_alt0 = 0.0
    _gnss_h_std = _gnss_v_std = 0.0
    _gnss_cos_lat0 = 1.0
    _gnss_hud_data = {}  # veh_name -> (lat, lon)，用于 HUD 显示
    _R_EARTH = 6_371_000.0   # 地球平均半径，单位：米

    if _gnss_enabled:
        try:
            import math   as _gnss_math
            import random as _gnss_random

            _gnss_origin     = _gnss_cfg.get("map_origin", {})
            _gnss_lat0       = float(_gnss_origin.get("latitude",  0.0))
            _gnss_lon0       = float(_gnss_origin.get("longitude", 0.0))
            _gnss_alt0       = float(_gnss_origin.get("altitude",  0.0))
            _gnss_noise      = _gnss_cfg.get("noise", {})
            _gnss_h_std      = float(_gnss_noise.get("horizontal_stddev_m", 0.5))
            _gnss_v_std      = float(_gnss_noise.get("altitude_stddev_m",   1.0))
            _gnss_rate       = float(_gnss_cfg.get("publish_rate_hz", 10.0))
            _gnss_frame_skip = max(1, int(round(app_freq / _gnss_rate)))
            _gnss_cos_lat0   = _gnss_math.cos(_gnss_math.radians(_gnss_lat0))

            # Isaac Sim 的 Python 3.12 无法导入为 Python 3.10 编译的 rclpy。
            # 将发布任务委托给 Python 3.10 子进程 gnss_bridge.py；它从标准输入读取
            # 制表符分隔的 GNSS 记录，并通过 rclpy 发布 sensor_msgs/NavSatFix。
            import subprocess as _gnss_subprocess
            import os as _gnss_os
            import time as _gnss_time

            _gnss_script = _gnss_os.path.join(
                _gnss_os.path.dirname(_gnss_os.path.abspath(__file__)),
                "gnss_bridge.py")
            # 使用最小化环境，避免 Isaac Sim 的 LD_LIBRARY_PATH 和 PYTHONPATH
            # 污染 Python 3.10 子进程。
            _gnss_env = {
                "HOME":             _gnss_os.environ.get("HOME", "/root"),
                "USER":             _gnss_os.environ.get("USER", "root"),
                "PATH":             "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
                "PYTHONPATH":       ("/opt/ros/humble/lib/python3.10/site-packages:"
                                     "/opt/ros/humble/local/lib/python3.10/dist-packages"),
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "LD_LIBRARY_PATH":   "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu",
                "ROS_DOMAIN_ID":     _gnss_os.environ.get("ROS_DOMAIN_ID", "0"),
                # 与父进程的 RMW 保持一致，使所有节点使用相同中间件。
                **({"RMW_IMPLEMENTATION": _gnss_os.environ["RMW_IMPLEMENTATION"]}
                   if "RMW_IMPLEMENTATION" in _gnss_os.environ else {}),
                # 传递网卡绑定设置，使 GNSS 节点绑定到相同接口。
                **({"CYCLONEDDS_URI": _gnss_os.environ["CYCLONEDDS_URI"]}
                   if "CYCLONEDDS_URI" in _gnss_os.environ else {}),
            }

            _gnss_log_path = _gnss_os.path.join(
                _gnss_os.path.dirname(_gnss_os.path.abspath(__file__)),
                "gnss_bridge.log")
            _gnss_log = open(_gnss_log_path, "w")
            _gnss_proc = _gnss_subprocess.Popen(
                ["/usr/bin/python3.10", _gnss_script],
                stdin=_gnss_subprocess.PIPE,
                stdout=_gnss_subprocess.PIPE,
                stderr=_gnss_log,
                text=True,
                env=_gnss_env,
            )
            # 阻塞等待桥接程序发出就绪信号，或因启动失败而退出。
            _gnss_ready = _gnss_proc.stdout.readline().strip()
            _gnss_log.flush()
            if _gnss_ready != "ready":
                raise RuntimeError(
                    f"gnss_bridge.py did not start correctly (got: {_gnss_ready!r}); "
                    f"see {_gnss_log_path} for details")
            _gnss_bridge = _gnss_proc

            for _gv in vehicles:
                if not _gv.get("enabled", True):     continue
                if not _gv.get("enable_gnss", True): continue
                _gvname = _gv.get("name", "")
                _gvpfx  = "/" + _gv.get("topic_prefix", f"/{_gvname.lower()}").strip("/")
                _gtopic = _gvpfx + "/gnss"
                _gfid   = _gvpfx.strip("/") + "/base_link"

                _gnss_publishers[_gvname] = {"topic": _gtopic, "frame_id": _gfid}
                print(f"[GNSS] {_gvname}: NavSatFix publisher on '{_gtopic}' "
                      f"at {_gnss_rate:.0f} Hz (every {_gnss_frame_skip} frames)")

            print(f"[GNSS] Map origin: lat={_gnss_lat0:.6f}°  lon={_gnss_lon0:.6f}°  "
                  f"alt={_gnss_alt0:.1f} m  |  noise H={_gnss_h_std} m  V={_gnss_v_std} m")
        except Exception as _gnss_init_err:
            import traceback as _gnss_tb
            print(f"[GNSS] Setup error: {_gnss_init_err}")
            _gnss_tb.print_exc()
            _gnss_enabled = False
    else:
        print("[GNSS] Disabled via config.")

    # ── 查找 IMU 和里程计 OmniGraph 节点，供 HUD 回读 ────────────────────────
    vehicle_sensor_nodes = {}  # 映射：veh_name -> {"imu_path": str|None, "odom_path": str|None}
    for veh in vehicles:
        if not veh.get("enabled", True): continue
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        imu_path = None
        odom_path = None

        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                continue
            try:
                node = og.get_node_by_path(p_path)
                if not node.is_valid(): continue
                n_type = node.get_node_type().get_node_type()
                if any(x in n_type for x in ["IsaacImuSensor", "ReadIMU"]):
                    imu_path = p_path
                elif any(x in n_type for x in ["ComputeOdometry", "IsaacComputeOdometry"]):
                    odom_path = p_path
            except Exception:
                continue
        vehicle_sensor_nodes[veh_name] = {"imu_path": imu_path, "odom_path": odom_path}
        print(f"[HUD] {veh_name}: odom_path={odom_path}, imu_path={imu_path}")
    
    # ── 跟随相机持久化配置 ───────────────────────────────────────────────────
    veh_follow_configs = {} # 映射：veh_name -> {base_path, front_path, rear_path, dist, height}
    fc_cfg = config.get("follow_camera", {})
    for veh in vehicles:
        if not veh.get("enabled", True): continue
        # 现在对所有活动车辆始终启用跟随相机
        
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        
        # 启动时只查找一次底盘和标记点
        # 查找 "Chassis" 或 "base_link"（不区分大小写）
        chassis_prim = None
        front_p = None
        rear_p = None
        
        for p in stage.Traverse():
            path_str = p.GetPath().pathString
            if not path_str.startswith(veh_prim_path): continue
            
            p_name = p.GetName().lower()
            if p.GetTypeName() != "OmniGraphNode":
                # 优先级：先 "chassis"，再 "base_link"
                if "chassis" in p_name and not chassis_prim:
                    chassis_prim = p
                elif "base_link" in p_name and not chassis_prim:
                    chassis_prim = p
                
                # 前向轴标记点
                if "front" in p_name and not front_p: front_p = p
                if ("rear" in p_name or "back" in p_name) and not rear_p: rear_p = p
            
        if chassis_prim and chassis_prim.IsValid():
            _cam_name_map = {
                "Ego_Vehicle": "EgoFollowCamera",
                "Opponent_Vehicle": "OpponentFollowCamera",
            }
            veh_follow_configs[veh_name] = {
                "base_path": chassis_prim.GetPath().pathString,
                "front_path": front_p.GetPath().pathString if front_p else None,
                "rear_path": rear_p.GetPath().pathString if rear_p else None,
                "dist": float(fc_cfg.get("distance", 5.0)),
                "height": float(fc_cfg.get("height", 2.0)),
                "focus_height": float(fc_cfg.get("focus_height", 0.5)),
                "cam_name": _cam_name_map.get(veh_name, "FollowCamera"),
            }

    follow_cam_handles = {} # 仿真循环的内部跟踪状态

    # ── 视口 HUD 叠加层 ───────────────────────────────────────────────────────
    HUD_ENABLED = False
    hud_labels  = {}
    ip_bar_window = None
    opp_hud_window = None
    _hud_all_windows = []  # 所有 HUD 窗口；按 / 键统一切换
    _split_active = False  # 在 HUD 代码块内正确设置；无头模式下为 False
    try:
        if headless_mode:
            raise RuntimeError("headless – skipping HUD")
        import omni.ui as ui

        HUD_ENABLED = True
        
        # ── 颜色常量（omni.ui = 0xAABBGGRR）──────────────────────────────────
        C_BLACK  = 0xFF000000   # 完全不透明的黑色
        C_CYAN   = 0xFFFFD400   # 亮青色（R=0x00 G=0xD4 B=0xFF A=0xFF）
        C_WHITE  = 0xFFFFFFFF   # 白色
        C_GREY   = 0xFF999999   # 柔和灰色
        C_ORANGE = 0xFF4488FF   # 橙色（R=0xFF G=0x88 B=0x44 A=0xFF）
        C_BLUE   = 0xFFFF8800   # 道奇蓝（R=0x00 G=0x88 B=0xFF A=0xFF）
        
        CARD_W = 228
        CARD_H = 516
        GAP    = 19

        enabled_vehicles = [v for v in vehicles if v.get("enabled", True)]
        # 分屏模式下，主（左侧）窗口只显示自车。
        _split_active = _ui_split_screen and len(enabled_vehicles) >= 2
        _main_vehs = [v for v in enabled_vehicles
                      if not _split_active or "Ego" in v.get("name", "")]
        _opp_vehs  = [v for v in enabled_vehicles
                      if _split_active and "Ego" not in v.get("name", "")]
        _n_veh = max(1, len(_main_vehs))

        # 缩放卡片高度，确保组合后的 HUD 不超过 Isaac Sim 窗口高度。
        # position_y=100 会占用顶部 100 像素，底部再预留 20 像素。
        _app_win    = omni.appwindow.get_default_app_window()
        _win_h      = _app_win.get_height() if _app_win else render_res[1]
        _avail_h    = _win_h - 120
        _card_h_fit = int((_avail_h - 16) / _n_veh - GAP)
        CARD_H = int(min(CARD_H, max(100, _card_h_fit)) * 0.7)
        _sc = CARD_H / 430  # 相对默认卡片高度的缩放系数

        CARD_W      = max(120, int(190 * _sc))
        FONT_HEADER = max(9,   int(22  * _sc))
        FONT_ROW    = max(8,   int(18  * _sc))
        FONT_CTRL   = max(6,   int(11  * _sc))
        CARD_MARGIN = max(4,   int(10  * _sc))
        INDENT_W    = max(4,   int(20  * _sc))
        LABEL_W     = max(40,  int(75  * _sc))
        SPACER_SM   = max(1,   int(4   * _sc))

        total_w = CARD_W + 16
        total_h = CARD_H * _n_veh + GAP * (_n_veh - 1) + 16

        hud_window = ui.Window(
            "VehicleStatusHUD",
            width=total_w,
            height=total_h,
            position_x=70,
            position_y=95,
            flags=(
                ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_MOVE
            ),
        )
        # 使用透明窗口边框——卡片自身带有背景
        hud_window.frame.set_style({"background_color": 0x00000000})

        hud_labels = {}

        def _label(text, color, size, w=0, bold=False):
            style = {"color": color, "font_size": size}
            if bold: style["font_style"] = "Bold"
            lbl = ui.Label(text, style=style, width=w if w > 0 else ui.Fraction(1),
                           alignment=ui.Alignment.LEFT_CENTER)
            return lbl

        with hud_window.frame:
            with ui.VStack(spacing=GAP, width=total_w):
                # 排序以确保自车位于顶部
                sorted_vehs = sorted(_main_vehs, key=lambda x: "Ego" not in x["name"])
                for veh in sorted_vehs:
                    veh_name = veh.get("name", "Vehicle")
                    models = {}
                    with ui.ZStack(width=CARD_W, height=CARD_H):
                        # 背景——半透明深灰色（0xAABBGGRR：A=0x55，RGB=0x383838）
                        ui.Rectangle(style={"background_color": 0x55383838, "border_radius": 8})
                        with ui.VStack(spacing=2, margin=CARD_MARGIN):
                            # 标题栏
                            is_ego = "Ego" in veh_name
                            h_color = C_CYAN if is_ego else C_WHITE
                            header_label = ui.Label(veh_name.replace("_", " "),
                                                    alignment=ui.Alignment.CENTER,
                                                    style={"color": h_color, "font_size": FONT_HEADER, "font_style": "Bold"})
                            models["header"] = header_label

                            def _row(label_text):
                                with ui.HStack():
                                    ui.Spacer(width=INDENT_W)
                                    ui.Label(f"{label_text}:", style={"color": C_WHITE, "font_size": FONT_ROW}, width=LABEL_W)
                                    val_label = ui.Label("--", style={"color": C_WHITE, "font_size": FONT_ROW}, width=ui.Fraction(1))
                                    return val_label

                            ui.Spacer(height=SPACER_SM)
                            ctrl_src_label = _label("CONTROL: KEYBOARD", C_WHITE, FONT_ROW, bold=True)
                            models["ctrl_src"] = ctrl_src_label
                            models["speed"] = _row("Speed")
                            models["steer"] = _row("Steer")

                            ui.Spacer(height=SPACER_SM)
                            _label("ODOMETRY", C_WHITE, FONT_ROW, bold=True)
                            models["pos_x"] = _row("Pos X")
                            models["pos_y"] = _row("Pos Y")
                            models["pos_z"] = _row("Pos Z")
                            models["lin_vel"] = _row("Lin Vel")
                            models["ang_vel"] = _row("Ang Vel")

                            ui.Spacer(height=SPACER_SM)
                            _label("IMU", C_WHITE, FONT_ROW, bold=True)
                            models["lin_acc"] = _row("Lin Acc")
                            models["ang_acc"] = _row("Ang Acc")

                            ui.Spacer(height=SPACER_SM)
                            _label("GNSS", C_WHITE, FONT_ROW, bold=True)
                            models["gnss_lat"] = _row("Lat")
                            models["gnss_lon"] = _row("Lon")

                    hud_labels[veh_name] = models

        print("[HUD] Viewport overlay window created.")

        # ── 对手车 HUD 窗口（分屏右侧）───────────────────────────────────────
        opp_hud_window = None
        _hud_all_windows = [hud_window]
        if _split_active and _opp_vehs:
            try:
                _opp_n = len(_opp_vehs)
                _opp_total_h = (CARD_H + GAP) * _opp_n + 16
                _win_w = _app_win.get_width() if _app_win else render_res[0]
                _opp_x = _win_w // 2 + 10
                opp_hud_window = ui.Window(
                    "OpponentStatusHUD",
                    width=total_w,
                    height=_opp_total_h,
                    position_x=_opp_x,
                    position_y=95,
                    flags=(
                        ui.WINDOW_FLAGS_NO_TITLE_BAR
                        | ui.WINDOW_FLAGS_NO_SCROLLBAR
                        | ui.WINDOW_FLAGS_NO_RESIZE
                        | ui.WINDOW_FLAGS_NO_MOVE
                    ),
                )
                opp_hud_window.frame.set_style({"background_color": 0x00000000})
                with opp_hud_window.frame:
                    with ui.VStack(spacing=GAP, width=total_w):
                        for _ov in _opp_vehs:
                            _ovn = _ov.get("name", "Vehicle")
                            _omodels = {}
                            with ui.ZStack(width=CARD_W, height=CARD_H):
                                ui.Rectangle(style={"background_color": 0x55383838, "border_radius": 8})
                                with ui.VStack(spacing=2, margin=CARD_MARGIN):
                                    _oh_color = C_CYAN if "Ego" in _ovn else C_WHITE
                                    _ohdr = ui.Label(_ovn.replace("_", " "),
                                                     alignment=ui.Alignment.CENTER,
                                                     style={"color": _oh_color,
                                                            "font_size": FONT_HEADER,
                                                            "font_style": "Bold"})
                                    _omodels["header"] = _ohdr

                                    def _orow(label_text):
                                        with ui.HStack():
                                            ui.Spacer(width=INDENT_W)
                                            ui.Label(f"{label_text}:",
                                                     style={"color": C_WHITE, "font_size": FONT_ROW},
                                                     width=LABEL_W)
                                            _vl = ui.Label("--",
                                                           style={"color": C_WHITE, "font_size": FONT_ROW},
                                                           width=ui.Fraction(1))
                                            return _vl

                                    ui.Spacer(height=SPACER_SM)
                                    _omodels["ctrl_src"] = _label("CONTROL: KEYBOARD", C_WHITE, FONT_ROW, bold=True)
                                    _omodels["speed"]    = _orow("Speed")
                                    _omodels["steer"]    = _orow("Steer")
                                    ui.Spacer(height=SPACER_SM)
                                    _label("ODOMETRY", C_WHITE, FONT_ROW, bold=True)
                                    _omodels["pos_x"]   = _orow("Pos X")
                                    _omodels["pos_y"]   = _orow("Pos Y")
                                    _omodels["pos_z"]   = _orow("Pos Z")
                                    _omodels["lin_vel"] = _orow("Lin Vel")
                                    _omodels["ang_vel"] = _orow("Ang Vel")
                                    ui.Spacer(height=SPACER_SM)
                                    _label("IMU", C_WHITE, FONT_ROW, bold=True)
                                    _omodels["lin_acc"] = _orow("Lin Acc")
                                    _omodels["ang_acc"] = _orow("Ang Acc")
                                    ui.Spacer(height=SPACER_SM)
                                    _label("GNSS", C_WHITE, FONT_ROW, bold=True)
                                    _omodels["gnss_lat"] = _orow("Lat")
                                    _omodels["gnss_lon"] = _orow("Lon")
                            hud_labels[_ovn] = _omodels
                _hud_all_windows.append(opp_hud_window)
                print(f"[HUD] Opponent HUD window created at x={_opp_x}.")
            except Exception as _opp_err:
                print(f"[HUD] Opponent HUD creation error: {_opp_err}")

    except Exception as e:
        HUD_ENABLED = False
        hud_labels = {}
        print(f"[HUD] Initialization Error: {e}")

    # 防御式查找键盘常量
    def get_key(cand_list):
        for c in cand_list:
            if hasattr(carb.input.KeyboardInput, c):
                return getattr(carb.input.KeyboardInput, c)
        return None

    K_1         = get_key(["ONE", "_1", "KEY_1", "DIGIT_1"])
    K_2         = get_key(["TWO", "_2", "KEY_2", "DIGIT_2"])
    K_F1        = get_key(["F1"])
    K_F2        = get_key(["F2"])
    K_SLASH     = get_key(["SLASH", "KEY_SLASH", "FORWARD_SLASH"])
    K_R         = get_key(["R", "KEY_R"])
    K_BACKSPACE = get_key(["BACKSPACE", "BACK_SPACE", "DELETE", "BS"])
    K_GRAVE     = get_key(["GRAVE", "BACK_QUOTE", "BACKQUOTE", "TILDE", "ACCENT_GRAVE",
                            "GRAVE_ACCENT", "OEM_3", "SECTION"])
    if K_GRAVE is None:
        # 动态后备方案：扫描每个 KeyboardInput 属性，查找重音符/反引号对应项
        for _kname in dir(carb.input.KeyboardInput):
            if any(s in _kname.upper() for s in ("GRAVE", "BACKTICK", "BACK_QUOTE", "TILDE")):
                K_GRAVE = getattr(carb.input.KeyboardInput, _kname)
                print(f"[Input] Found grave/backtick key via scan: {_kname}")
                break

    # 启动时记录所有已发现的按键绑定
    print(f"[Input] Key bindings: 1={K_1}, 2={K_2}, F1={K_F1}, F2={K_F2}, /={K_SLASH}, R={K_R}, Backspace={K_BACKSPACE}, `={K_GRAVE}")
    if K_1 is None: print("[Input] Warning: Could not find key for '1'.")
    if K_2 is None: print("[Input] Warning: Could not find key for '2'.")
    if K_F1 is None: print("[Input] Warning: Could not find key for F1.")
    if K_F2 is None: print("[Input] Warning: Could not find key for F2.")
    if K_GRAVE is None: print("[Input] Warning: Could not find backtick/grave key — dumping all KeyboardInput names:")
    if K_GRAVE is None:
        print("  " + ", ".join(n for n in dir(carb.input.KeyboardInput) if not n.startswith("_")))

    # ── 仿真选择与交互状态 ─────────────────────────────────────────────────────
    selected_vehicle_name = "Ego_Vehicle"
    _vp2_api     = None              # 视口 2 的 viewport_api，在分屏设置时保存
    _vp1_showing = "Ego_Vehicle"     # 当前显示在视口 1 中的车辆相机
    _vp2_showing = "Opponent_Vehicle"  # 当前显示在视口 2 中的车辆相机
    slash_pressed_last = False
    r_pressed_last = False
    grave_pressed_last = False
    backspace_pressed_last = False
    choice_keys_pressed_last = {}
    if K_1: choice_keys_pressed_last[K_1] = False
    if K_2: choice_keys_pressed_last[K_2] = False
    key1_hold_start = None   # 首次按下按键 1 时的 time.monotonic()
    key2_hold_start = None   # 首次按下按键 2 时的 time.monotonic()
    key1_toggle_fired = False  # 长按切换触发后为 True，松开前保持抑制
    key2_toggle_fired = False

    CTRL_HOLD_DURATION = 1.0  # 切换控制模式所需的长按秒数

    # 每辆车的显式控制模式："KEYBOARD_CONTROL" 或 "ROS2_CONTROL"，长按 1/2 切换。
    # 无头模式下没有键盘，所有车辆默认使用 ROS2_CONTROL；否则默认值来自配置中的
    # user_interface.default_control_mode。
    if headless_mode:
        _default_ctrl = "ROS2_CONTROL"
    else:
        _default_ctrl = "ROS2_CONTROL" if _ui_default_ctrl_mode == "ROS2_CONTROL" else "KEYBOARD_CONTROL"
    veh_ctrl_mode = {veh["name"]: _default_ctrl for veh in vehicles if veh.get("enabled", True)}
    # 记录每辆车 OmniGraph 发布器上次路由到的模式，只有 topicName 实际变化时
    # 才调用 og.Controller.set()。
    _pub_routed_mode = {}  # veh_name -> "KEYBOARD_CONTROL" 或 "ROS2_CONTROL"
    _last_n_status_lines = 0  # 上次打印的状态行数（用于光标上移）

    is_recording = False
    seg_capture_index = 0

    def _set_viewport_camera(path):
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp: vp.camera_path = path
        except Exception: pass

    def switch_selection(new_name):
        nonlocal selected_vehicle_name, _vp1_showing, _vp2_showing
        # 始终更新跟随相机，即使该车辆已经被选中
        if new_name in veh_follow_configs:
            _cfg = veh_follow_configs[new_name]
            # 记录接收此相机的视口，用于键盘路由和 HUD 内容
            if _split_active and _vp2_api is not None:
                try:
                    from omni.kit.viewport.utility import get_active_viewport as _gav
                    _active_is_vp2 = (_gav() is _vp2_api)
                    if _active_is_vp2:
                        _vp2_showing = new_name
                    else:
                        _vp1_showing = new_name
                except Exception:
                    pass
            _set_viewport_camera(f"/World/{_cfg['cam_name']}")

        # 仅在车辆确实发生变化时更新选择状态和 HUD
        if new_name == selected_vehicle_name: return
        if new_name not in vehicle_teleop_publishers: return

        print(f"[Selection] Switching to {new_name}")
        selected_vehicle_name = new_name

        if not _split_active:
            for v_name, models in hud_labels.items():
                if "header" in models:
                    color = C_BLUE if (new_name == v_name) else C_WHITE
                    models["header"].style = {"color": color, "font_size": FONT_HEADER, "font_style": "Bold", "alignment": ui.Alignment.CENTER}

    # ── 分割数据集设置 ─────────────────────────────────────────────────────────
    seg_cfg = config.get("semantic_segmentation", {})
    seg_enabled = bool(seg_cfg) and bool(seg_cfg.get("enabled", True))
    seg_rgb_annot = None
    seg_mask_annot = None
    seg_id_keywords = {}
    seg_color_map = {}
    seg_images_dir = ""
    seg_gt_masks_dir = ""
    seg_overwrite = True
    seg_capture_freq = 1
    seg_default_id = 0

    if seg_enabled:
        try:
            import numpy as _np
            from PIL import Image as _PILImage
            import omni.replicator.core as _rep
            from pxr import Sdf as _Sdf

            seg_id_keywords  = {int(k): v for k, v in seg_cfg.get("id_keywords", {}).items()}
            seg_color_map    = {int(k): tuple(v) for k, v in seg_cfg.get("color_map", {}).items()}
            # capture_frequency 的单位为 Hz，将其转换为帧间隔
            seg_capture_freq = max(1, int(app_freq / max(1, float(seg_cfg.get("capture_frequency", 1)))))
            _seg_res         = seg_cfg.get("image_resolution", [1280, 720])
            seg_images_dir   = os.path.join(repo_root, seg_cfg.get("images_dir", "data/segmentation/images"))
            seg_gt_masks_dir = os.path.join(repo_root, seg_cfg.get("gt_masks_dir", "data/segmentation/masks"))
            seg_overwrite    = bool(seg_cfg.get("overwrite_existing", True))

            _old_umask = os.umask(0)
            try:
                os.makedirs(seg_images_dir,   mode=0o777, exist_ok=True)
                os.makedirs(seg_gt_masks_dir, mode=0o777, exist_ok=True)
            finally:
                os.umask(_old_umask)
            # 从叶目录向上，对每个中间目录执行 chmod，但不包括 repo_root
            for _leaf in [seg_images_dir, seg_gt_masks_dir]:
                _d = _leaf
                while _d and _d != repo_root:
                    try:
                        os.chmod(_d, 0o777)
                    except Exception:
                        pass
                    _parent = os.path.dirname(_d)
                    if _parent == _d:
                        break
                    _d = _parent

            # 确定默认（兜底）标签 ID
            for _id, _kws in seg_id_keywords.items():
                if "default" in _kws:
                    seg_default_id = _id
                    break

            # 仅为 Mesh Prim 分配语义标签；跳过关节、物理 Prim、OmniGraph 节点等，
            # 以免破坏非可视 Stage 元素。
            def _assign_semantic_label(prim, label_str):
                try:
                    from omni.isaac.core.utils.semantics import add_update_semantics
                    add_update_semantics(prim, label_str, type_label="class")
                    return
                except Exception:
                    pass
                try:
                    from pxr import Semantics as _SemAPI
                    _sem = _SemAPI.SemanticsAPI.Apply(prim, "Semantics")
                    _sem.CreateSemanticTypeAttr().Set("class")
                    _sem.CreateSemanticDataAttr().Set(label_str)
                except Exception:
                    pass

            _seg_stage = omni.usd.get_context().get_stage()
            _labeled = 0
            for _prim in _seg_stage.Traverse():
                if not _prim.IsValid() or _prim.GetTypeName() != "Mesh":
                    continue
                # 只标注环境 Prim，绝不修改车辆 Prim，以免破坏车辆网格可见性或渲染状态。
                if not _prim.GetPath().pathString.startswith("/World/Environment"):
                    continue
                _name_lower = _prim.GetName().lower()
                _assigned   = seg_default_id
                for _id, _kws in seg_id_keywords.items():
                    if "default" in _kws:
                        continue
                    for _kw in _kws:
                        if _kw.lower() in _name_lower:
                            _assigned = _id
                            break
                    if _assigned != seg_default_id:
                        break
                _assign_semantic_label(_prim, str(_assigned))
                _labeled += 1
            print(f"[Segmentation] Assigned semantic labels to {_labeled} Mesh prims.")

            # 在 /World/Ego_Vehicle 下查找 RGB 相机。若配置中设置了 camera_prim，
            # 则查找路径以该值结尾的第一台相机；否则优先选择名称包含 "color"/"rgb"
            # 的相机，并以任意非深度相机作为后备。
            _seg_cam_prim  = seg_cfg.get("camera_prim", None)
            _ego_cam_path  = None
            _ego_cam_fallback = None
            for _prim in _seg_stage.Traverse():
                _ps = _prim.GetPath().pathString
                if not _ps.startswith("/World/Ego_Vehicle") or not _prim.IsA(UsdGeom.Camera):
                    continue
                if _seg_cam_prim:
                    if _ps.endswith(_seg_cam_prim) or _prim.GetName() == _seg_cam_prim:
                        _ego_cam_path = _ps
                        break
                    continue
                _cam_name_lower = _prim.GetName().lower()
                if "depth" in _cam_name_lower:
                    continue  # 跳过深度相机
                if "color" in _cam_name_lower or "rgb" in _cam_name_lower:
                    _ego_cam_path = _ps
                    break  # 最佳匹配
                if _ego_cam_fallback is None:
                    _ego_cam_fallback = _ps  # 以后备方式选择任意非深度相机
            if _ego_cam_path is None:
                _ego_cam_path = _ego_cam_fallback

            if _ego_cam_path:
                _seg_rp = _rep.create.render_product(_ego_cam_path, tuple(_seg_res))
                seg_rgb_annot  = _rep.AnnotatorRegistry.get_annotator("rgb")
                seg_mask_annot = _rep.AnnotatorRegistry.get_annotator(
                    "semantic_segmentation",
                    init_params={"colorize": False},
                )
                seg_rgb_annot.attach(_seg_rp)
                seg_mask_annot.attach(_seg_rp)
                print(f"[Segmentation] Camera: {_ego_cam_path} | Resolution: {_seg_res[0]}x{_seg_res[1]}")
                print(f"[Segmentation] images_dir : {seg_images_dir}")
                print(f"[Segmentation] gt_masks_dir: {seg_gt_masks_dir}")
                print(f"[Segmentation] Capture: {seg_cfg.get('capture_frequency', 1)} Hz "
                      f"→ every {seg_capture_freq} frames at {app_freq} Hz loop")
                print(f"[Segmentation] Press R to start/stop recording.")
            else:
                print("[Segmentation] WARNING: No UsdGeom.Camera found under /World/Ego_Vehicle — disabled.")
                seg_enabled = False

        except Exception as _seg_err:
            print(f"[Segmentation] Setup error: {_seg_err}")
            seg_enabled = False

    def _rviz_reset():
        """如果 RViz2 已打开，则通过 xdotool 触发“文件 > 重置”。
        清除 RViz2 内部 TF 缓存，避免仿真重启前的陈旧坐标系在桥接程序重启后
        引发 TF_OLD_DATA 警告。
        """
        import os as _rv_os, shutil as _rv_sh, subprocess as _rv_sp, time as _rv_t
        try:
            if not _rv_os.environ.get("DISPLAY"):
                return
            if not _rv_sh.which("xdotool"):
                return
            _r = _rv_sp.run(["pgrep", "-x", "rviz2"], capture_output=True, timeout=1)
            if _r.returncode != 0:
                return
            _r = _rv_sp.run(["xdotool", "search", "--classname", "rviz2"],
                            capture_output=True, text=True, timeout=2)
            if not _r.stdout.strip():
                _r = _rv_sp.run(["xdotool", "search", "--name", "RViz"],
                                capture_output=True, text=True, timeout=2)
            _wids = _r.stdout.strip().split()
            if not _wids:
                return
            _wid = _wids[0]
            _rv_sp.run(["xdotool", "windowfocus", "--sync", _wid], capture_output=True, timeout=2)
            _rv_t.sleep(0.1)
            _rv_sp.run(["xdotool", "key", "--window", _wid, "--clearmodifiers", "alt+F"],
                       capture_output=True, timeout=2)
            _rv_t.sleep(0.15)
            _rv_sp.run(["xdotool", "key", "--window", _wid, "--clearmodifiers", "r"],
                       capture_output=True, timeout=2)
            print("[RViz] Reset triggered.")
        except Exception:
            pass

    # ── 仿真循环 ───────────────────────────────────────────────────────────────
    ts = config.get("keyboard_control_settings", {})
    MAX_SPEED = float(ts.get("max_speed_m_s", 15.0))
    MAX_STEER = float(ts.get("max_steer_rad", 0.52))
    ACCEL = float(ts.get("acceleration_m_s2", 2.0))
    DECEL = float(ts.get("deceleration_m_s2", ACCEL * 2.0))
    STEER_SPEED = float(ts.get("steering_speed_rad_s", 1.5))
    
    current_speed = 0.0
    current_steer = 0.0
    opp_speed = 0.0
    opp_steer = 0.0
    dt_teleop = 1.0 / float(app_freq)
    
    input_iface = carb.input.acquire_input_interface()
    keyboard = None

    # Kit 将空格键绑定到工具栏的播放/暂停操作。原始键盘事件回调执行得太晚，
    # 无法覆盖该快捷键，因此从 Kit 的进程内快捷键注册表中移除工具栏操作。
    # 这不会影响时间线 UI 按钮，也不会修改用户持久化的快捷键设置。
    try:
        import omni.kit.hotkeys.core as _hotkeys_core

        _hotkey_registry = _hotkeys_core.get_hotkey_registry()
        _removed_space_hotkeys = 0
        for _hotkey in list(_hotkey_registry.get_all_hotkeys_for_key("SPACE")):
            _hotkey_desc = " ".join(
                str(getattr(_hotkey, _name, ""))
                for _name in ("id", "module", "action", "action_text")
            ).lower()
            if "toolbar" in _hotkey_desc and "play" in _hotkey_desc:
                if _hotkey_registry.deregister_hotkey(_hotkey):
                    _removed_space_hotkeys += 1
        if _removed_space_hotkeys:
            print(f"[Input] Reserved Space for brake; removed {_removed_space_hotkeys} Timeline Play/Pause hotkey(s).")
        else:
            print("[Input] No Timeline Space hotkey found; Space remains available for brake.")
    except Exception as _space_hotkey_err:
        print(f"[Input] Could not unregister Timeline Space hotkey: {_space_hotkey_err}")

    iteration = 0
    _restart_pending = False  # stop→play 过渡稳定期间有一帧为 True
    _soft_restart_time = 0.0  # 上次触发重启时的仿真时钟值
    _ui_setup_done = False    # 首次跟随相机初始化后执行一次视口/面板设置
    _dock_vp2_iteration = -1  # 延迟数帧停靠视口，确保 UI 已就绪
    _rt_sim_dt = 1.0 / float(app_freq)  # 每个 Tick 的仿真秒数
    _rt_wall_last = time.monotonic()   # 上次实时因子采样时的墙上时间（按区间计算）
    _rt_iter_last = 0                  # 上次实时因子采样时的迭代次数
    _ctrl_mode_last_pub = 0.0  # 上次周期性发布 control_mode 的墙上时间

    if headless_mode:
        _veh_names = list(veh_ctrl_mode.keys())
        print(f"\n[Headless] Running without display. Control mode: ROS2_CONTROL for {_veh_names}")
        print(f"[Headless] Subscribe to drive commands via /ego/drive (AckermannDriveStamped) or /ego/control (autoware_control_msgs/Control)")
    print(f"\n[Simulator] Entering Main Loop ({app_freq}Hz)...")

    # 预先导入热点路径模块，使每个 Tick 的查找都能低成本命中缓存。
    from omni.usd import get_world_transform_matrix as _get_wtm
    from pxr import Gf, UsdGeom
    _FC_UP = Gf.Vec3d(0, 0, 1)          # 跟随相机计算使用的恒定上方向向量
    def _avg(b): return sum(b) / len(b)  # 跟随相机缓冲区的平滑辅助函数
    _gnss_base_prims = {}  # veh_name -> 缓存的 Usd.Prim，避免每 Tick 调用 GetPrimAtPath

    # 刻意保持此分析器轻量：它将 Kit/PhysX/OmniGraph 的耗时与本启动器的 Python
    # 工作耗时分开。当实时因子较低、但 CPU/GPU 总体利用率也较低时，首先需要这种拆分。
    _profile_cfg = config.get("profiling", {})
    _profile_enabled = bool(_profile_cfg.get("enabled", True))
    _profile_interval_s = max(0.25, float(_profile_cfg.get("interval_s", 1.0)))
    _profile_window_start = time.perf_counter()
    _profile_update_ms = []
    _profile_post_ms = []
    if _profile_enabled:
        print(f"[Profile] Enabled: reporting update/post-loop timing every "
              f"{_profile_interval_s:g}s.")

    while simulation_app.is_running():
        _profile_tick_start = time.perf_counter() if _profile_enabled else 0.0
        simulation_app.update()
        _profile_after_update = time.perf_counter() if _profile_enabled else 0.0
        iteration += 1

        # 以 10 Hz（每 100 ms）发布 control_mode 状态。
        _now_wall = time.monotonic()
        if _ros_bridge_enabled and _drive_proc is not None and (_now_wall - _ctrl_mode_last_pub) >= 0.1:
            try:
                for _ivn, _imode in veh_ctrl_mode.items():
                    _ival = 1 if _imode == "ROS2_CONTROL" else 0
                    _drive_proc.stdin.write(f"ctrl_mode\t{_ivn}\t{_ival}\n")
                _drive_proc.stdin.flush()
                _ctrl_mode_last_pub = _now_wall
            except Exception:
                pass

        # 延迟重启：在 stop 后的下一帧才调用 play，确保 PhysX 张量视图
        #（里程计/IMU getVelocities）在 OmniGraph 节点再次执行前已完全销毁。
        # 本帧跳过所有 OmniGraph 读取。
        if _restart_pending:
            # ── 重启 ROS 桥接程序 ─────────────────────────────────────────────
            # 终止并重新生成桥接子进程，会让两个 rclpy 节点获得全新的 TF 缓存
            #（cache_time = 2 s），其中不含上次运行的陈旧条目。这是解决
            # TF_OLD_DATA/LiDAR 数据空窗的根本办法：新的 rclpy 节点意味着新的
            # /tf 发布器，RViz 会立即重置该连接的 TF 缓存。
            if _ros_bridge_enabled and _drive_proc is not None:
                try:
                    _drive_proc.stdin.close()
                except Exception:
                    pass
                try:
                    _drive_proc.terminate()
                    _drive_proc.wait(timeout=2.0)
                except Exception:
                    pass
                try:
                    _drive_proc = _drv_sub.Popen(
                        ["/usr/bin/python3.10", _drv_script],
                        stdin=_drv_sub.PIPE, stdout=_drv_sub.PIPE,
                        stderr=_drive_log, text=True, env=_drv_env)
                    for _rc in _drv_sub_cmds:
                        _drive_proc.stdin.write(_rc)
                    _drive_proc.stdin.write("start\n")
                    _drive_proc.stdin.flush()
                    _drv_rdy = _drive_proc.stdout.readline().strip()
                    if _drv_rdy == "ready":
                        _drv_threading.Thread(
                            target=_drive_reader, args=(_drive_proc,),
                            daemon=True).start()
                        for _pc in _drv_post_cmds:
                            _drive_proc.stdin.write(_pc)
                        for _rvn, _rmode in veh_ctrl_mode.items():
                            _rval = 1 if _rmode == "ROS2_CONTROL" else 0
                            _drive_proc.stdin.write(f"ctrl_mode\t{_rvn}\t{_rval}\n")
                        _drive_proc.stdin.flush()
                        print("[ROS2 Bridge] Drive bridge restarted.")
                    else:
                        print(f"[ROS2 Bridge] Drive bridge restart failed: {_drv_rdy!r}")
                except Exception as _drv_rerr:
                    print(f"[ROS2 Bridge] Drive bridge restart error: {_drv_rerr}")

            if _gnss_enabled and _gnss_proc is not None and _gnss_script is not None:
                try:
                    _gnss_proc.stdin.close()
                except Exception:
                    pass
                try:
                    _gnss_proc.terminate()
                    _gnss_proc.wait(timeout=2.0)
                except Exception:
                    pass
                try:
                    _gnss_proc = subprocess.Popen(
                        ["/usr/bin/python3.10", _gnss_script],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=_gnss_log, text=True, env=_gnss_env)
                    _gnss_rdy = _gnss_proc.stdout.readline().strip()
                    if _gnss_rdy == "ready":
                        _gnss_bridge = _gnss_proc
                        print("[GNSS] GNSS bridge restarted.")
                    else:
                        print(f"[GNSS] GNSS bridge restart failed: {_gnss_rdy!r}")
                except Exception as _gnss_rerr:
                    print(f"[GNSS] GNSS bridge restart error: {_gnss_rerr}")

            # ── 恢复仿真 ─────────────────────────────────────────────────────
            _rviz_reset()
            timeline.set_current_time(_soft_restart_time + 3.0)
            timeline.play()
            _restart_pending = False
            iteration = 0
            _rt_wall_last = time.monotonic()
            _rt_iter_last = 0
            _last_n_status_lines = 0
            print("[Sim] Simulation restarted.")
            continue

        # ── 周期性重新应用坐标系 ID ───────────────────────────────────────────
        # 图重新求值后，OmniGraph 节点可能把已写入的属性值重置为默认值。
        # 每 60 个 Tick（60 Hz 时约 1 秒）重新应用，可使 TF 发布器坐标系 ID
        # 保持命名空间，从而让 ego/base_link → ego/odom → map TF 链在 RViz 中保持完整。
        if _og_frame_id_remaps and iteration % 60 == 1:
            for _fir_path, _fir_attr, _fir_val in _og_frame_id_remaps:
                try:
                    og.Controller.set(og.Controller.attribute(f"{_fir_path}.{_fir_attr}"), _fir_val)
                except Exception:
                    pass

        if not keyboard:
            appwindow = omni.appwindow.get_default_app_window()
            if appwindow: keyboard = appwindow.get_keyboard()
        
        val_w = val_s = val_a = val_d = val_up = val_down = val_left = val_right = val_space = 0.0
        val_1 = val_2 = val_slash = val_f1 = val_f2 = 0.0

        if keyboard:
            val_w = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.W)
            val_s = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.S)
            val_a = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.A)
            val_d = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.D)
            val_up = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.UP)
            val_down = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.DOWN)
            val_left = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.LEFT)
            val_right = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.RIGHT)
            val_space = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.SPACE)
            val_1         = input_iface.get_keyboard_value(keyboard, K_1)         if K_1         else 0.0
            val_2         = input_iface.get_keyboard_value(keyboard, K_2)         if K_2         else 0.0
            val_f1        = input_iface.get_keyboard_value(keyboard, K_F1)        if K_F1        else 0.0
            val_f2        = input_iface.get_keyboard_value(keyboard, K_F2)        if K_F2        else 0.0
            val_slash     = input_iface.get_keyboard_value(keyboard, K_SLASH)     if K_SLASH     else 0.0
            val_r         = input_iface.get_keyboard_value(keyboard, K_R)         if K_R         else 0.0
            val_backspace = input_iface.get_keyboard_value(keyboard, K_BACKSPACE) if K_BACKSPACE else 0.0
            val_grave     = input_iface.get_keyboard_value(keyboard, K_GRAVE)     if K_GRAVE     else 0.0

            # ` ——切换到透视相机
            if val_grave > 0.1 and not grave_pressed_last:
                _set_viewport_camera("/OmniverseKit_Persp")
                print("[Camera] Switched to Perspective")
            grave_pressed_last = (val_grave > 0.1)

            # 退格键——重启仿真（立即停止，延迟一帧后播放）
            if val_backspace > 0.1 and not backspace_pressed_last:
                print("[Sim] Restarting simulation...")
                _soft_restart_time = float(iteration) * _rt_sim_dt  # 停止时的仿真时钟
                timeline.stop()
                current_speed = 0.0
                current_steer = 0.0
                opp_speed = 0.0
                opp_steer = 0.0
                _restart_pending = True
            backspace_pressed_last = (val_backspace > 0.1)

            # 1 / 2——短按选择车辆；长按超过 1.5 秒切换控制模式
            _now = time.monotonic()

            if K_1:
                key1_down = (val_1 > 0.1)
                key1_was_down = choice_keys_pressed_last.get(K_1, False)
                if key1_down:
                    if key1_hold_start is None:
                        key1_hold_start = _now
                        key1_toggle_fired = False
                    elif not key1_toggle_fired and (_now - key1_hold_start) >= CTRL_HOLD_DURATION:
                        _new = "ROS2_CONTROL" if veh_ctrl_mode.get("Ego_Vehicle") == "KEYBOARD_CONTROL" else "KEYBOARD_CONTROL"
                        veh_ctrl_mode["Ego_Vehicle"] = _new
                        print(f"\n[Control] Ego_Vehicle → {_new}")
                        key1_toggle_fired = True
                        if _ros_bridge_enabled and _drive_proc is not None:
                            try:
                                _drive_proc.stdin.write(f"ctrl_mode\tEgo_Vehicle\t{1 if _new == 'ROS2_CONTROL' else 0}\n")
                                _drive_proc.stdin.flush()
                            except Exception:
                                pass
                else:
                    if key1_was_down and not key1_toggle_fired:
                        # 未达到长按阈值便松开——松开时切换相机
                        switch_selection("Ego_Vehicle")
                    key1_hold_start = None
                    key1_toggle_fired = False
                choice_keys_pressed_last[K_1] = key1_down

            if K_2:
                key2_down = (val_2 > 0.1)
                key2_was_down = choice_keys_pressed_last.get(K_2, False)
                if key2_down:
                    if key2_hold_start is None:
                        key2_hold_start = _now
                        key2_toggle_fired = False
                    elif not key2_toggle_fired and (_now - key2_hold_start) >= CTRL_HOLD_DURATION:
                        _new = "ROS2_CONTROL" if veh_ctrl_mode.get("Opponent_Vehicle") == "KEYBOARD_CONTROL" else "KEYBOARD_CONTROL"
                        veh_ctrl_mode["Opponent_Vehicle"] = _new
                        print(f"\n[Control] Opponent_Vehicle → {_new}")
                        key2_toggle_fired = True
                        if _ros_bridge_enabled and _drive_proc is not None:
                            try:
                                _drive_proc.stdin.write(f"ctrl_mode\tOpponent_Vehicle\t{1 if _new == 'ROS2_CONTROL' else 0}\n")
                                _drive_proc.stdin.flush()
                            except Exception:
                                pass
                else:
                    if key2_was_down and not key2_toggle_fired:
                        # 未达到长按阈值便松开——松开时切换相机
                        switch_selection("Opponent_Vehicle")
                    key2_hold_start = None
                    key2_toggle_fired = False
                choice_keys_pressed_last[K_2] = key2_down

            # HUD 开关（同时切换主窗口和对手车窗口）
            if val_slash > 0.1 and not slash_pressed_last:
                _hud_vis = not hud_window.visible
                for _hw in _hud_all_windows:
                    _hw.visible = _hud_vis
            slash_pressed_last = (val_slash > 0.1)

            # R——切换分割数据录制
            if val_r > 0.1 and not r_pressed_last:
                is_recording = not is_recording
                if is_recording:
                    seg_capture_index = 0
                    print("\n[Record] Recording STARTED — saving segmentation data.")
                    print(f"[Record] seg_enabled={seg_enabled} | "
                          f"rgb_annot={seg_rgb_annot is not None} | "
                          f"mask_annot={seg_mask_annot is not None}")
                else:
                    print(f"\n[Record] Recording STOPPED — {seg_capture_index} frame(s) saved.")
            r_pressed_last = (val_r > 0.1)
        
        # 遥控逻辑
        f_dt = float(dt_teleop)
        if _split_active:
            # 分屏：WASD 驾驶自车，方向键独立驾驶对手车。
            pressed_throttle_ego = (val_w > 0.0 or val_s > 0.0 or val_space > 0.0)
            pressed_steer_ego    = (val_a > 0.0 or val_d > 0.0 or val_space > 0.0)
            pressed_throttle_opp = (val_up > 0.0 or val_down > 0.0 or val_space > 0.0)
            pressed_steer_opp    = (val_left > 0.0 or val_right > 0.0 or val_space > 0.0)
            pressed_throttle = pressed_throttle_ego or pressed_throttle_opp
            pressed_steer    = pressed_steer_ego or pressed_steer_opp

            if val_w > 0.0:    current_speed += ACCEL * f_dt
            elif val_s > 0.0:  current_speed -= ACCEL * f_dt
            if val_a > 0.0:    current_steer += STEER_SPEED * f_dt
            elif val_d > 0.0:  current_steer -= STEER_SPEED * f_dt
            if val_space > 0.0: current_speed = current_steer = 0.0

            if val_up > 0.0:    opp_speed += ACCEL * f_dt
            elif val_down > 0.0: opp_speed -= ACCEL * f_dt
            if val_left > 0.0:  opp_steer += STEER_SPEED * f_dt
            elif val_right > 0.0: opp_steer -= STEER_SPEED * f_dt
            if val_space > 0.0: opp_speed = opp_steer = 0.0

            # 自车自动回正
            if not pressed_throttle_ego:
                if current_speed > 0.0: current_speed = max(0.0, current_speed - DECEL * f_dt)
                elif current_speed < 0.0: current_speed = min(0.0, current_speed + DECEL * f_dt)
            if not pressed_steer_ego:
                if current_steer > 0.0: current_steer = max(0.0, current_steer - (STEER_SPEED * 2.0) * f_dt)
                elif current_steer < 0.0: current_steer = min(0.0, current_steer + (STEER_SPEED * 2.0) * f_dt)
            current_speed = max(min(current_speed, MAX_SPEED), -MAX_SPEED)
            current_steer = max(min(current_steer, MAX_STEER), -MAX_STEER)

            # 对手车自动回正
            if not pressed_throttle_opp:
                if opp_speed > 0.0: opp_speed = max(0.0, opp_speed - DECEL * f_dt)
                elif opp_speed < 0.0: opp_speed = min(0.0, opp_speed + DECEL * f_dt)
            if not pressed_steer_opp:
                if opp_steer > 0.0: opp_steer = max(0.0, opp_steer - (STEER_SPEED * 2.0) * f_dt)
                elif opp_steer < 0.0: opp_steer = min(0.0, opp_steer + (STEER_SPEED * 2.0) * f_dt)
            opp_speed = max(min(opp_speed, MAX_SPEED), -MAX_SPEED)
            opp_steer = max(min(opp_steer, MAX_STEER), -MAX_STEER)
        else:
            # 单屏：WASD 和方向键都用于驾驶当前选中的车辆。
            pressed_throttle = (val_w > 0.0 or val_up > 0.0 or val_s > 0.0 or val_down > 0.0 or val_space > 0.0)
            pressed_steer = (val_a > 0.0 or val_left > 0.0 or val_d > 0.0 or val_right > 0.0 or val_space > 0.0)
            if val_w > 0.0 or val_up > 0.0:    current_speed += ACCEL * f_dt
            elif val_s > 0.0 or val_down > 0.0: current_speed -= ACCEL * f_dt
            if val_a > 0.0 or val_left > 0.0:  current_steer += STEER_SPEED * f_dt
            elif val_d > 0.0 or val_right > 0.0: current_steer -= STEER_SPEED * f_dt
            if val_space > 0.0: current_speed = current_steer = 0.0

            # 自动回正
            if not pressed_throttle:
                if current_speed > 0.0: current_speed = max(0.0, current_speed - DECEL * f_dt)
                elif current_speed < 0.0: current_speed = min(0.0, current_speed + DECEL * f_dt)
            if not pressed_steer:
                if current_steer > 0.0: current_steer = max(0.0, current_steer - (STEER_SPEED * 2.0) * f_dt)
                elif current_steer < 0.0: current_steer = min(0.0, current_steer + (STEER_SPEED * 2.0) * f_dt)
            current_speed = max(min(current_speed, MAX_SPEED), -MAX_SPEED)
            current_steer = max(min(current_steer, MAX_STEER), -MAX_STEER)
        
        # 驾驶指令通过 _drive_reader 后台线程到达（drive_bridge.py 子进程）。

        # 应用控制——每辆车都有通过 F1/F2 切换的显式模式。
        # TELEOP：键盘驾驶选中的车辆，未选车辆保持空闲。
        # ROS2：由外部 ROS 2 指令驾驶车辆，忽略键盘输入。
        #       显示内容回读 OmniGraph 控制器状态，因此反映车辆内置 ROS 2 订阅器
        #       实际应用的值，而不只是 rclpy 桥接程序收到的值。
        def _read_og_ctrl(meta):
            """直接从 OmniGraph AckermannController 输入读取速度和转向值。"""
            try:
                _rs = og.Controller.get(og.Controller.attribute(meta["ctrl_attr_speed"]))
                _ra = og.Controller.get(og.Controller.attribute(meta["ctrl_attr_steer"]))
                return (float(_rs) if _rs is not None else 0.0,
                        float(_ra) if _ra is not None else 0.0)
            except Exception:
                return (0.0, 0.0)

        # ── 延迟查找订阅器 Tick ────────────────────────────────────────────────
        # 启动时，对手车的 OmniGraph 可能尚未完整注册到 OG 运行时，导致
        # og.get_node_by_path() 返回无效节点，并且 USD GetConnections() 可能无法
        # 返回订阅器 execIn 的连接。前 300 帧内持续重试，待图完全初始化后找到
        # Tick 路径，并立即应用 TELEOP 门控。
        if iteration < 300:
            for _lz_name, _lz_meta in vehicle_teleop_publishers.items():
                if _lz_meta.get("sub_tick_path"):
                    continue  # 已找到
                _lz_sub = _lz_meta.get("sub_node_path", "")
                if not _lz_sub:
                    # 同时重试查找订阅器节点本身
                    for _lz_prim in stage.Traverse():
                        _lz_pp = _lz_prim.GetPath().pathString
                        if not _lz_pp.startswith(f"/World/{_lz_name}"): continue
                        if _lz_prim.GetTypeName() != "OmniGraphNode": continue
                        _lz_nt = ""
                        try:
                            _lz_n = og.get_node_by_path(_lz_pp)
                            if _lz_n and _lz_n.is_valid():
                                _lz_nt = _lz_n.get_node_type().get_node_type()
                        except Exception: pass
                        if not _lz_nt:
                            try: _lz_nt = _lz_prim.GetAttribute("node:type").Get() or ""
                            except Exception: pass
                        if "SubscribeAckermannDrive" in _lz_nt:
                            _lz_meta["sub_node_path"] = _lz_pp
                            _lz_meta["sub_topic_attr"] = f"{_lz_pp}.inputs:topicName"
                            _lz_sub = _lz_pp
                            print(f"\n[Teleop] Lazy: found subscriber for {_lz_name}: {_lz_pp}")
                            _pub_routed_mode.pop(_lz_name, None)
                            break
                if _lz_sub:
                    # 尝试通过 inputs:execIn 上的 USD 连接查找驱动 Tick
                    try:
                        _lz_sp = stage.GetPrimAtPath(_lz_sub)
                        _lz_ei = _lz_sp.GetAttribute("inputs:execIn")
                        if _lz_ei:
                            for _lz_src in _lz_ei.GetConnections():
                                _lz_pp2 = _lz_src.GetPrimPath()
                                _lz_tp = stage.GetPrimAtPath(_lz_pp2)
                                if _lz_tp.IsValid():
                                    _lz_tnt = _lz_tp.GetAttribute("node:type").Get() or ""
                                    if any(k in _lz_tnt for k in ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")):
                                        _lz_meta["sub_tick_path"] = str(_lz_pp2)
                                        print(f"\n[Teleop] Lazy: found sub_tick for {_lz_name}: {_lz_pp2}")
                                        # 如果当前处于 TELEOP 模式则立即禁用
                                        if veh_ctrl_mode.get(_lz_name, "KEYBOARD_CONTROL") == "KEYBOARD_CONTROL":
                                            try:
                                                og.Controller.set(og.Controller.attribute(f"{_lz_pp2}.inputs:enabled"), False)
                                                print(f"\n[Teleop] Lazy: sub_tick DISABLED for {_lz_name}")
                                            except Exception: pass
                                        _pub_routed_mode.pop(_lz_name, None)
                                        break
                    except Exception: pass

        # 键盘路由：WASD → VP1 车辆，方向键 → VP2 车辆。
        if _split_active:
            _kbd_vp1_veh = _vp1_showing
            _kbd_vp2_veh = _vp2_showing
        else:
            _kbd_vp1_veh = selected_vehicle_name
            _kbd_vp2_veh = selected_vehicle_name

        _hud_cmds = {}  # veh_name -> 本 Tick 实际应用的 (speed, steer)
        for _ctrl_veh, _meta in vehicle_teleop_publishers.items():
            _mode = veh_ctrl_mode.get(_ctrl_veh, "KEYBOARD_CONTROL")
            _just_switched_to_ros2 = False
            # TELEOP 模式下完全不查询 ROS 2 桥接程序，只处理键盘输入。
            if _mode == "ROS2_CONTROL":
                _drv = _drive_cmds.get(_ctrl_veh)
                _ext = _drv if (_drv and (time.monotonic() - float(_drv["stamp"])) < ros2_cmd_timeout_s) else None
            else:
                _ext = None
            if _ctrl_veh == selected_vehicle_name:
                if _mode == "KEYBOARD_CONTROL":
                    if _split_active:
                        if _ctrl_veh == _kbd_vp1_veh and _ctrl_veh == _kbd_vp2_veh:
                            _ks = current_speed if abs(current_speed) >= abs(opp_speed) else opp_speed
                            _kstr = current_steer if abs(current_steer) >= abs(opp_steer) else opp_steer
                            _apply_speed, _apply_steer = _ks, _kstr
                        elif _ctrl_veh == _kbd_vp1_veh:
                            _apply_speed, _apply_steer = current_speed, current_steer
                        elif _ctrl_veh == _kbd_vp2_veh:
                            _apply_speed, _apply_steer = opp_speed, opp_steer
                        else:
                            _apply_speed, _apply_steer = 0.0, 0.0
                    else:
                        _apply_speed, _apply_steer = current_speed, current_steer
                else:  # ROS2 模式
                    if _ext is not None:
                        _apply_speed, _apply_steer = _ext["speed"], _ext["steer"]
                    else:
                        # 桥接程序没有新指令——读取实际 OmniGraph 控制器状态，
                        # 使显示内容反映车辆内置 ROS 2 订阅器（若存在）所应用的指令。
                        _apply_speed, _apply_steer = _read_og_ctrl(_meta)
            else:
                # 模式变化时重路由订阅器/发布器话题（未选中的车辆）。
                # 必须在 continue 前完成，使 TELEOP 模式能够屏蔽外部 ROS 2 指令。
                if _pub_routed_mode.get(_ctrl_veh) != _mode:
                    _new_pub    = _meta["drive_topic"] if _mode == "KEYBOARD_CONTROL" else _meta["monitor_topic"]
                    _new_sub    = _meta["muted_topic"]
                    # 无条件更新保护状态；即使下方 og.Controller.set() 调用抛出异常，
                    # 也不会再次触发重路由。
                    _pub_routed_mode[_ctrl_veh] = _mode
                    if _mode == "ROS2_CONTROL":
                        _just_switched_to_ros2 = True
                    try:
                        og.Controller.set(og.Controller.attribute(_meta["pub_topic_attr"]), _new_pub)
                        # 无论处于何种模式，都始终禁用订阅器 Tick。物理循环每个 Tick
                        # 都会通过 og.Controller.set() 使用 _drive_cmds 缓存驱动控制器。
                        # 在 ROS2 模式启用订阅器 Tick 会导致 SubscribeAckermannDrive 节点
                        # 在没有新消息的每个 Tick 将控制器输入重置为 0（控制器以 10 Hz
                        # 发送、仿真为 60 Hz 时，即每 6 个 Tick 中有 5 个），从而覆盖
                        # og.Controller.set() 写入的值。
                        if _meta.get("sub_node_path"):
                            try:
                                og.Controller.set(og.Controller.attribute(f"{_meta['sub_node_path']}.inputs:enabled"), False)
                            except Exception: pass
                        if _meta.get("sub_tick_path"):
                            try:
                                og.Controller.set(og.Controller.attribute(f"{_meta['sub_tick_path']}.inputs:enabled"), False)
                            except Exception: pass
                        for _sc_src, _sc_dst in (_meta.get("sub_ctrl_connections") or []):
                            try:
                                og.Controller.disconnect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                            except Exception: pass
                        if _meta.get("sub_topic_attr"):
                            og.Controller.set(og.Controller.attribute(_meta["sub_topic_attr"]), _new_sub)
                        print(f"\n[Control] {_ctrl_veh}: routed to {_mode}")
                    except Exception as _rr_e:
                        print(f"\n[Teleop] {_ctrl_veh}: failed to reroute: {_rr_e}")
                if _mode == "KEYBOARD_CONTROL":
                    # 每个 Tick 都必须继续执行到 og.Controller.set()，
                    # 以覆盖控制器上任何陈旧的订阅器 ROS 2 值。
                    if _ctrl_veh == _kbd_vp1_veh and _ctrl_veh == _kbd_vp2_veh:
                        _ks = current_speed if abs(current_speed) >= abs(opp_speed) else opp_speed
                        _kstr = current_steer if abs(current_steer) >= abs(opp_steer) else opp_steer
                        _apply_speed, _apply_steer = _ks, _kstr
                    elif _ctrl_veh == _kbd_vp1_veh:
                        _apply_speed, _apply_steer = current_speed, current_steer
                    elif _ctrl_veh == _kbd_vp2_veh:
                        _apply_speed, _apply_steer = opp_speed, opp_steer
                    else:
                        _apply_speed, _apply_steer = 0.0, 0.0
                elif _ext is not None:
                    _apply_speed, _apply_steer = _ext["speed"], _ext["steer"]
                else:
                    _apply_speed, _apply_steer = _read_og_ctrl(_meta)
            _hud_cmds[_ctrl_veh] = (_apply_speed, _apply_steer)

            # 模式变化时重路由发布器话题；订阅器始终保持禁用。
            if _pub_routed_mode.get(_ctrl_veh) != _mode:
                _new_pub = _meta["drive_topic"] if _mode == "KEYBOARD_CONTROL" else _meta["monitor_topic"]
                # 无条件更新保护状态；即使下方 og.Controller.set() 调用抛出异常，
                # 也不会再次触发重路由。
                _pub_routed_mode[_ctrl_veh] = _mode
                if _mode == "ROS2_CONTROL":
                    _just_switched_to_ros2 = True
                try:
                    og.Controller.set(og.Controller.attribute(_meta["pub_topic_attr"]), _new_pub)
                    if _meta.get("sub_node_path"):
                        try:
                            og.Controller.set(og.Controller.attribute(f"{_meta['sub_node_path']}.inputs:enabled"), False)
                        except Exception: pass
                    if _meta.get("sub_tick_path"):
                        try:
                            og.Controller.set(og.Controller.attribute(f"{_meta['sub_tick_path']}.inputs:enabled"), False)
                        except Exception: pass
                    for _sc_src, _sc_dst in (_meta.get("sub_ctrl_connections") or []):
                        try:
                            og.Controller.disconnect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                        except Exception: pass
                    print(f"\n[Control] {_ctrl_veh}: routed to {_mode}")
                except Exception as _rr_e:
                    print(f"\n[Teleop] {_ctrl_veh}: failed to reroute: {_rr_e}")

            try:
                if _mode == "KEYBOARD_CONTROL":
                    # TELEOP：使用键盘驱动控制器，并发布数据以便观察。
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), _apply_speed)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), _apply_steer)
                elif _just_switched_to_ros2:
                    # 模式切换时执行一次性复位：清除锁存的键盘指令。
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), 0.0)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), 0.0)
                    _sub_np = _meta.get("sub_node_path")
                    if _sub_np:
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:speed"), 0.0)
                        except Exception: pass
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:steeringAngle"), 0.0)
                        except Exception: pass
                elif _ext is not None:
                    # ROS2 模式下收到新指令：应用该指令。同时写入控制器输入
                    #（订阅器→控制器连接不存在或已断开时有效）和订阅器输出属性；
                    # 这样任何尚未断开的订阅器→控制器连接也会携带我们的值，因为订阅器
                    # Tick 已禁用，其计算函数不会覆盖这些值。
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), _ext["speed"])
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), _ext["steer"])
                    _sub_np = _meta.get("sub_node_path")
                    if _sub_np:
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:speed"), float(_ext["speed"]))
                        except Exception: pass
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:steeringAngle"), float(_ext["steer"]))
                        except Exception: pass
                else:
                    # ROS2 模式下在 ros2_cmd_timeout_s 内未收到指令：安全停车。
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), 0.0)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), 0.0)
                    _sub_np = _meta.get("sub_node_path")
                    if _sub_np:
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:speed"), 0.0)
                        except Exception: pass
                        try: og.Controller.set(og.Controller.attribute(f"{_sub_np}.outputs:steeringAngle"), 0.0)
                        except Exception: pass
                
                # 无条件将观测发布器节点与当前物理实际值同步，防止未连接的 OmniGraph
                # 节点反复向 ROS 2 话题发送 0.0。
                _pub_p = _meta["pub_node_path"]
                og.Controller.set(og.Controller.attribute(f"{_pub_p}.inputs:speed"), float(_apply_speed))
                og.Controller.set(og.Controller.attribute(f"{_pub_p}.inputs:steeringAngle"), float(_apply_steer))
            except Exception: pass


        # ── 终端状态：原位显示两行（自车 + 对手车）────────────────────────────
        if iteration % max(1, app_freq // 10) == 0:
            _rt_now = time.monotonic()
            _rt_wall_dt = _rt_now - _rt_wall_last
            _rt_pct = (((iteration - _rt_iter_last) * _rt_sim_dt / _rt_wall_dt) * 100.0) if _rt_wall_dt > 0 else 100.0
            _rt_wall_last = _rt_now
            _rt_iter_last = iteration
            _status_lines = []
            for _sv, (_ss, _sstr) in _hud_cmds.items():
                _smode = "KBD" if veh_ctrl_mode.get(_sv, "KEYBOARD_CONTROL") == "KEYBOARD_CONTROL" else "ROS"
                _status_lines.append(
                    f"[{_smode}] {_sv}: Spd={_ss:+.2f} m/s  Str={float(_sstr)*57.2958:+.1f}°"
                )
            if _status_lines:
                _status_lines[-1] += f"  | RT={_rt_pct:.0f}%"
            if _last_n_status_lines > 0 and _status_lines:
                print(f"\033[{_last_n_status_lines}A", end="", flush=False)
            for _sl in _status_lines:
                print(f"\r{_sl:<70}", flush=False)
            if _status_lines:
                sys.stdout.flush()
            _last_n_status_lines = len(_status_lines)

        # 跟随相机（无头模式没有视口，因此跳过）
        if not headless_mode:
         for veh_name, cfg in veh_follow_configs.items():
            data = follow_cam_handles.get(veh_name)
            if not data or not data["cam"].GetPrim().IsValid():
                from pxr import UsdGeom
                base_prim = stage.GetPrimAtPath(cfg["base_path"])
                if not base_prim.IsValid(): continue
                # 相机位于世界层级，而不在底盘 Prim 下。将其设为底盘的子节点会导致
                # 单帧抖动：物理引擎在写入局部变换后才更新底盘世界变换，
                # 因此渲染器在每个 Tick 会短暂看到 new_chassis × old_local。
                fc_path = f"/World/{cfg['cam_name']}"
                cam_p = stage.GetPrimAtPath(fc_path)
                if not cam_p.IsValid():
                    cam_p = UsdGeom.Camera.Define(stage, fc_path).GetPrim()
                    UsdGeom.Camera(cam_p).GetHorizontalApertureAttr().Set(20.955)
                    UsdGeom.Camera(cam_p).GetVerticalApertureAttr().Set(11.787)
                    UsdGeom.Camera(cam_p).GetFocalLengthAttr().Set(18.1)
                data = {
                    "cam": UsdGeom.Camera(cam_p), "base": base_prim,
                    "front": stage.GetPrimAtPath(cfg["front_path"]) if cfg["front_path"] else None,
                    "rear": stage.GetPrimAtPath(cfg["rear_path"]) if cfg["rear_path"] else None,
                    "dist": cfg["dist"], "height": cfg["height"], "focus_height": cfg["focus_height"],
                    "buf_x":  collections.deque(maxlen=40),
                    "buf_y":  collections.deque(maxlen=40),
                    "buf_z":  collections.deque(maxlen=40),
                    "buf_lz": collections.deque(maxlen=40),
                    # 旋转平滑：对约 24 帧的前向向量分量求平均，
                    # 在跟踪转向的同时消除方向抖动
                    "buf_fx": collections.deque(maxlen=24),
                    "buf_fy": collections.deque(maxlen=24),
                    "buf_fz": collections.deque(maxlen=24),
                }
                follow_cam_handles[veh_name] = data

            try:
                m = _get_wtm(data["base"])
                pos = m.ExtractTranslation()
                rot = m.ExtractRotationMatrix()
                # 仅根据底盘旋转矩阵推导前向。使用前后 Prim 的世界位置会在前轮转向时
                # 引发振动：前部 Prim 是转向几何体的子节点，因此即使车身静止，
                # 它的世界位置也会发生偏移。
                raw_fwd = rot.GetRow(0)
                # ExtractRotationMatrix() 并不总是返回单位行向量——行向量长度等于 Prim
                # 的世界缩放因子。进行归一化，使平滑缓冲区无论车辆缩放如何都保存单位向量。
                _rf_len = raw_fwd.GetLength()
                if _rf_len > 1e-6: raw_fwd = raw_fwd / _rf_len
                # 平滑位置
                data["buf_x"].append(pos[0]); data["buf_y"].append(pos[1])
                data["buf_z"].append(pos[2] + data["height"])
                data["buf_lz"].append(pos[2] + data["focus_height"])
                # 平滑前向向量分量以消除旋转抖动
                data["buf_fx"].append(raw_fwd[0])
                data["buf_fy"].append(raw_fwd[1])
                data["buf_fz"].append(raw_fwd[2])
                sx = _avg(data["buf_x"]); sy = _avg(data["buf_y"])
                sz = _avg(data["buf_z"]); slz = _avg(data["buf_lz"])
                sfwd = Gf.Vec3d(_avg(data["buf_fx"]), _avg(data["buf_fy"]), _avg(data["buf_fz"]))
                sfwd_len = sfwd.GetLength()
                if sfwd_len > 0.01: sfwd = sfwd / sfwd_len
                else: sfwd = raw_fwd
                xy_base = Gf.Vec3d(sx, sy, pos[2]) - (sfwd * data["dist"])
                cam_pos_world = Gf.Vec3d(xy_base[0], xy_base[1], sz)
                lookat_pos = Gf.Vec3d(sx, sy, slz)
                lookat_m_world = Gf.Matrix4d().SetLookAt(cam_pos_world, lookat_pos, _FC_UP)
                # 相机位于世界层级，因此其局部变换就是世界变换——无需父节点逆变换，
                # 也不会受到底盘物理抖动的影响。
                local_m = lookat_m_world.GetInverse()
                xformable = UsdGeom.Xformable(data["cam"])
                x_attr = data.get("x_attr") or xformable.GetPrim().GetAttribute("xformOp:transform")
                if not x_attr:
                    xformable.AddTransformOp().Set(local_m)
                    data["x_attr"] = xformable.GetPrim().GetAttribute("xformOp:transform")
                else:
                    x_attr.Set(local_m)
                    data["x_attr"] = x_attr
            except Exception: pass

        # ── 一次性 UI 设置（视口相机、分屏、隐藏面板）─────────────────────────
        # 延迟到第一次跟随相机 Tick 后执行，确保 USD 相机 Prim 已存在。
        if not headless_mode and not _ui_setup_done and follow_cam_handles:
            _ui_setup_done = True
            # 将自车跟随相机设为默认视口相机
            if "Ego_Vehicle" in veh_follow_configs:
                _set_viewport_camera(f"/World/{veh_follow_configs['Ego_Vehicle']['cam_name']}")

            # 分屏：创建视口 2，并将其指向对手车相机
            if _ui_split_screen and len(enabled_vehicles) >= 2 and "Opponent_Vehicle" in veh_follow_configs:
                try:
                    from omni.kit.viewport.utility import create_viewport_window as _cvw
                    import omni.ui as _ui2
                    _vp2_win = _cvw("Viewport 2")
                    if _vp2_win is not None:
                        _dock_vp2_iteration = iteration + 2  # 再过 2 帧执行停靠
                        _opp_cam_p = f"/World/{veh_follow_configs['Opponent_Vehicle']['cam_name']}"
                        if hasattr(_vp2_win, "viewport_api"):
                            _vp2_api = _vp2_win.viewport_api
                            _vp2_api.camera_path = _opp_cam_p
                    print("[UI] Split-screen: Viewport 2 created for Opponent_Vehicle.")
                except Exception as _vp2e:
                    print(f"[UI] Split-screen setup error: {_vp2e}")

            _rviz_reset()

        if _ui_split_screen and _dock_vp2_iteration > 0 and iteration >= _dock_vp2_iteration:
            try:
                import omni.ui as _ui2
                _vp_win = _ui2.Workspace.get_window("Viewport")
                _vp2_win_o = _ui2.Workspace.get_window("Viewport 2")
                if _vp_win and _vp2_win_o:
                    _vp2_win_o.dock_in(_vp_win, _ui2.DockPosition.RIGHT, 0.5)
                    print(f"[UI] Split-screen: Viewport 2 docked successfully at iter {iteration}.")
                    _dock_vp2_iteration = -1  # 设置成功，停止重试
                elif iteration > _dock_vp2_iteration + 60:
                    print("[UI] Split-screen: Viewport dock timeout (windows not found).")
                    _dock_vp2_iteration = -1  # 已超时
            except Exception as e:
                print(f"[UI] Split-screen dock setup error: {e}")
                _dock_vp2_iteration = -1

        # ── GNSS 发布 ─────────────────────────────────────────────────────────
        # 按 _gnss_frame_skip 间隔运行，例如 120 Hz 下每 12 帧执行一次即为 10 Hz。
        # 读取底盘世界变换，以配置的地图原点为基准应用等距圆柱投影，生成 WGS-84
        # 经度、纬度和高度，加入高斯噪声后发布 sensor_msgs/NavSatFix。
        if _gnss_enabled and _gnss_publishers and (iteration % _gnss_frame_skip == 0):
            _gnss_stamp_ns = _gnss_time.time_ns()
            for _gvn, _gpub_info in _gnss_publishers.items():
                try:
                    _gcfg = veh_follow_configs.get(_gvn)
                    if not _gcfg:
                        continue
                    if _gvn not in _gnss_base_prims:
                        _gnss_base_prims[_gvn] = stage.GetPrimAtPath(_gcfg["base_path"])
                    _gbase = _gnss_base_prims[_gvn]
                    if not _gbase.IsValid():
                        continue
                    _gp = _get_wtm(_gbase).ExtractTranslation()
                    _gx, _gy, _gz = float(_gp[0]), float(_gp[1]), float(_gp[2])

                    # 等距圆柱投影：仿真 +X = 东（经度），仿真 +Y = 北（纬度）
                    _lat = _gnss_lat0 + _gnss_math.degrees(_gy / _R_EARTH)
                    _lon = _gnss_lon0 + _gnss_math.degrees(_gx / (_R_EARTH * _gnss_cos_lat0))
                    _alt = _gnss_alt0 + _gz

                    # 在每个轴上加入相互独立的高斯噪声
                    _lat += _gnss_math.degrees(_gnss_random.gauss(0.0, _gnss_h_std) / _R_EARTH)
                    _lon += _gnss_math.degrees(_gnss_random.gauss(0.0, _gnss_h_std) / (_R_EARTH * _gnss_cos_lat0))
                    _alt += _gnss_random.gauss(0.0, _gnss_v_std)

                    _gnss_hud_data[_gvn] = (_lat, _lon)

                    # 通过标准输入管道发送给 gnss_bridge.py 子进程。
                    _gline = (f"{_gpub_info['topic']}\t{_gpub_info['frame_id']}\t"
                              f"{_lat}\t{_lon}\t{_alt}\t"
                              f"{_gnss_h_std}\t{_gnss_v_std}\t{_gnss_stamp_ns}\n")
                    _gnss_bridge.stdin.write(_gline)
                    _gnss_bridge.stdin.flush()
                except Exception:
                    pass

        # 更新 HUD
        if iteration % 4 == 0 and HUD_ENABLED:
            if _split_active and opp_hud_window is not None:
                # VP1 的 HUD 始终位于 VP1，VP2 的 HUD 始终位于 VP2。
                try:
                    import omni.ui as _ui_hud
                    _vp1_ui = _ui_hud.Workspace.get_window("Viewport")
                    _vp2_ui = _ui_hud.Workspace.get_window("Viewport 2")
                    if _vp1_ui and _vp2_ui:
                        hud_window.position_x     = _vp1_ui.position_x + 10
                        opp_hud_window.position_x = _vp2_ui.position_x + 10
                except Exception:
                    pass
                # 每个 HUD 窗口显示当前位于对应视口中的车辆。
                # hud_labels["Ego_Vehicle"] 包含 hud_window（VP1）的控件；
                # hud_labels["Opponent_Vehicle"] 包含 opp_hud_window（VP2）的控件。
                for _hud_veh, v_models in [
                    (_vp1_showing, hud_labels.get("Ego_Vehicle", {})),
                    (_vp2_showing, hud_labels.get("Opponent_Vehicle", {})),
                ]:
                    if not v_models:
                        continue
                    s_meta = vehicle_sensor_nodes.get(_hud_veh, {})
                    if "header" in v_models:
                        v_models["header"].text = _hud_veh.replace("_", " ")
                        v_models["header"].style = {"color": C_WHITE, "font_size": FONT_HEADER, "font_style": "Bold"}
                    _cs = "CONTROL: ROS2" if veh_ctrl_mode.get(_hud_veh, "KEYBOARD_CONTROL") == "ROS2_CONTROL" else "CONTROL: KEYBOARD"
                    if "ctrl_src" in v_models:
                        v_models["ctrl_src"].text = _cs
                        v_models["ctrl_src"].style = {"color": 0xFFFFFFFF, "font_size": FONT_ROW, "font_style": "Bold"}
                    _cmd = _hud_cmds.get(_hud_veh, (0.0, 0.0))
                    v_models["speed"].text = f"{_cmd[0]:+.2f} m/s"
                    v_models["steer"].text = f"{float(_cmd[1]) * 57.2958:+.2f}°"
                    odom_p = s_meta.get("odom_path")
                    if odom_p:
                        try:
                            pos = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:position"))
                            lv  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:linearVelocity"))
                            av  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:angularVelocity"))
                            if pos is not None:
                                v_models["pos_x"].text = f"{pos[0]:+.3f} m"
                                v_models["pos_y"].text = f"{pos[1]:+.3f} m"
                                v_models["pos_z"].text = f"{pos[2]:+.3f} m"
                            if lv is not None:
                                v_models["lin_vel"].text = f"{lv[0]:+.2f} m/s"
                            if av is not None:
                                v_models["ang_vel"].text = f"{av[2]:+.2f} r/s"
                        except Exception: pass
                    imu_p = s_meta.get("imu_path")
                    if imu_p:
                        try:
                            la     = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:linAcc"))
                            av_imu = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:angVel"))
                            if la is not None:
                                v_models["lin_acc"].text = f"{la[0]:+.2f} m/s²"
                            if av_imu is not None:
                                v_models["ang_acc"].text = f"{av_imu[2]:+.2f} r/s"
                        except Exception: pass
                    if "gnss_lat" in v_models:
                        _gd = _gnss_hud_data.get(_hud_veh)
                        if _gd is not None:
                            v_models["gnss_lat"].text = f"{_gd[0]:.6f}°"
                            v_models["gnss_lon"].text = f"{_gd[1]:.6f}°"
            else:
                for v_name, v_models in hud_labels.items():
                    s_meta = vehicle_sensor_nodes.get(v_name, {})
                    _cs = "CONTROL: ROS2" if veh_ctrl_mode.get(v_name, "KEYBOARD_CONTROL") == "ROS2_CONTROL" else "CONTROL: KEYBOARD"
                    if "ctrl_src" in v_models:
                        v_models["ctrl_src"].text = _cs
                        v_models["ctrl_src"].style = {"color": 0xFFFFFFFF, "font_size": FONT_ROW, "font_style": "Bold"}
                    _cmd = _hud_cmds.get(v_name, (0.0, 0.0))
                    v_models["speed"].text = f"{_cmd[0]:+.2f} m/s"
                    v_models["steer"].text = f"{float(_cmd[1]) * 57.2958:+.2f}°"
                    odom_p = s_meta.get("odom_path")
                    if odom_p:
                        try:
                            pos = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:position"))
                            lv  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:linearVelocity"))
                            av  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:angularVelocity"))
                            if pos is not None:
                                v_models["pos_x"].text = f"{pos[0]:+.3f} m"
                                v_models["pos_y"].text = f"{pos[1]:+.3f} m"
                                v_models["pos_z"].text = f"{pos[2]:+.3f} m"
                            if lv is not None:
                                v_models["lin_vel"].text = f"{lv[0]:+.2f} m/s"
                            if av is not None:
                                v_models["ang_vel"].text = f"{av[2]:+.2f} r/s"
                        except Exception: pass
                    imu_p = s_meta.get("imu_path")
                    if imu_p:
                        try:
                            la     = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:linAcc"))
                            av_imu = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:angVel"))
                            if la is not None:
                                v_models["lin_acc"].text = f"{la[0]:+.2f} m/s²"
                            if av_imu is not None:
                                v_models["ang_acc"].text = f"{av_imu[2]:+.2f} r/s"
                        except Exception: pass
                    if "gnss_lat" in v_models:
                        _gd = _gnss_hud_data.get(v_name)
                        if _gd is not None:
                            v_models["gnss_lat"].text = f"{_gd[0]:.6f}°"
                            v_models["gnss_lon"].text = f"{_gd[1]:.6f}°"

        # ── 分割数据采集 ─────────────────────────────────────────────────────
        if is_recording and seg_enabled and seg_rgb_annot and seg_mask_annot:
            if iteration % seg_capture_freq == 0:
                try:
                    # 标注器会直接提取最新缓冲区，无需执行 Orchestrator 步骤。
                    _rgb_data = seg_rgb_annot.get_data()
                    _seg_data = seg_mask_annot.get_data()

                    if _rgb_data is not None and _seg_data is not None:
                        # 确定输出文件编号
                        if seg_overwrite:
                            _file_num = seg_capture_index
                        else:
                            _file_num = seg_capture_index
                            while (
                                os.path.exists(os.path.join(seg_images_dir,   f"{_file_num:06d}.png")) or
                                os.path.exists(os.path.join(seg_gt_masks_dir, f"{_file_num:06d}.png"))
                            ):
                                _file_num += 1

                        _img_path  = os.path.join(seg_images_dir,   f"{_file_num:06d}.png")
                        _mask_path = os.path.join(seg_gt_masks_dir, f"{_file_num:06d}.png")

                        # 保存 RGB 图像（若存在 Alpha 通道则将其丢弃）
                        _rgb_arr = _np.array(_rgb_data, dtype=_np.uint8)
                        if _rgb_arr.ndim == 3 and _rgb_arr.shape[2] == 4:
                            _rgb_arr = _rgb_arr[:, :, :3]
                        _PILImage.fromarray(_rgb_arr).save(_img_path)
                        os.chmod(_img_path, 0o666)

                        # 根据语义分割数据构建彩色掩码。
                        # seg_data["data"]：形状为 (H, W) 的 Replicator 内部 ID uint32 数组
                        # seg_data["info"]["idToLabels"]：{rep_id_str -> {"class": "our_id_str"}}
                        _seg_pixels   = _np.asarray(_seg_data.get("data", _np.zeros((1, 1), dtype=_np.uint32)))
                        _id_to_labels = _seg_data.get("info", {}).get("idToLabels", {})
                        _default_color = seg_color_map.get(seg_default_id, (0, 0, 0))

                        if _seg_pixels.ndim == 3:          # (H, W, 4) 打包 RGBA ID
                            _seg_pixels = _seg_pixels.view(_np.uint32).reshape(_seg_pixels.shape[:2])

                        _h, _w = _seg_pixels.shape[:2]
                        _mask = _np.full((_h, _w, 3), _default_color, dtype=_np.uint8)

                        for _rep_id_str, _label_dict in _id_to_labels.items():
                            try:
                                _our_id = int(_label_dict.get("class", str(seg_default_id)))
                            except (ValueError, TypeError):
                                _our_id = seg_default_id
                            _color = seg_color_map.get(_our_id, _default_color)
                            _mask[_seg_pixels == int(_rep_id_str)] = _color

                        _PILImage.fromarray(_mask).save(_mask_path)
                        os.chmod(_mask_path, 0o666)

                        seg_capture_index += 1
                        print(f"[Record] #{_file_num:06d} | rgb: {_img_path} | mask: {_mask_path}")

                except Exception as _cap_err:
                    import traceback
                    print(f"[Record] Capture error: {_cap_err}")
                    traceback.print_exc()

        if _profile_enabled:
            _profile_tick_end = time.perf_counter()
            _profile_update_ms.append((_profile_after_update - _profile_tick_start) * 1000.0)
            _profile_post_ms.append((_profile_tick_end - _profile_after_update) * 1000.0)
            _profile_elapsed = _profile_tick_end - _profile_window_start
            if _profile_elapsed >= _profile_interval_s:
                _profile_count = len(_profile_update_ms)
                def _profile_summary(samples):
                    _sorted = sorted(samples)
                    _p95_index = min(len(_sorted) - 1, int(0.95 * (len(_sorted) - 1)))
                    return sum(samples) / len(samples), _sorted[_p95_index], max(samples)

                _upd_avg, _upd_p95, _upd_max = _profile_summary(_profile_update_ms)
                _post_avg, _post_p95, _post_max = _profile_summary(_profile_post_ms)
                _total_avg = _upd_avg + _post_avg
                _update_share = (100.0 * _upd_avg / _total_avg) if _total_avg else 0.0
                print(
                    f"[Profile] loop={_profile_count / _profile_elapsed:.1f} Hz "
                    f"({_profile_count} ticks/{_profile_elapsed:.2f}s) | "
                    f"update avg/p95/max={_upd_avg:.2f}/{_upd_p95:.2f}/{_upd_max:.2f} ms | "
                    f"post avg/p95/max={_post_avg:.2f}/{_post_p95:.2f}/{_post_max:.2f} ms | "
                    f"update={_update_share:.0f}%"
                )
                # 终端实时因子显示使用光标上移转义序列重写之前的状态行。
                # 分析器行是持久输出，因此要防止下一次状态更新覆盖它。
                _last_n_status_lines = 0
                _profile_window_start = _profile_tick_end
                _profile_update_ms.clear()
                _profile_post_ms.clear()

    simulation_app.close()

if __name__ == "__main__":
    main()
