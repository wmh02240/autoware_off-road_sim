"""LARIAD 车辆语义 TF 和传感器能力发现。

这里集中处理 USD Prim 到 ROS 语义坐标系的映射。外参仍然从组合 Stage
实时计算，不在 YAML 中复制一份容易失真的 CAD 参数。
"""

from __future__ import annotations


def node_type(prim):
    """读取 OmniGraph Prim 的节点类型。"""
    attr = prim.GetAttribute("node:type")
    return str(attr.Get() or "") if attr else ""


def relative_pose(parent_prim, child_prim):
    """计算 child 相对于 parent 的 ROS/TF 位姿 (x,y,z,qx,qy,qz,qw)。"""
    import omni.usd

    parent_world = omni.usd.get_world_transform_matrix(parent_prim)
    child_world = omni.usd.get_world_transform_matrix(child_prim)
    parent_pos = parent_world.ExtractTranslation()
    child_pos = child_world.ExtractTranslation()
    parent_rot = parent_world.ExtractRotation()
    child_rot = child_world.ExtractRotation()
    local_pos = parent_rot.GetInverse().TransformDir(child_pos - parent_pos)
    local_rot = child_rot * parent_rot.GetInverse()
    local_quat = local_rot.GetQuat()
    local_imag = local_quat.GetImaginary()
    return (
        float(local_pos[0]), float(local_pos[1]), float(local_pos[2]),
        float(local_imag[0]), float(local_imag[1]), float(local_imag[2]),
        float(local_quat.GetReal()),
    )


def rotation_delta(current_pose, initial_pose):
    """返回当前姿态相对于启动姿态的单位四元数增量。"""
    cx, cy, cz, cw = (float(value) for value in current_pose[3:7])
    ix, iy, iz, iw = (float(value) for value in initial_pose[3:7])
    ix, iy, iz = -ix, -iy, -iz
    dx = cw * ix + cx * iw + cy * iz - cz * iy
    dy = cw * iy - cx * iz + cy * iw + cz * ix
    dz = cw * iz + cx * iy - cy * ix + cz * iw
    dw = cw * iw - cx * ix - cy * iy - cz * iz
    norm = max((dx*dx + dy*dy + dz*dz + dw*dw) ** 0.5, 1.0e-12)
    return dx / norm, dy / norm, dz / norm, dw / norm


def wheel_rotation(current_pose, initial_pose):
    """去除 CAD 固定旋转，仅保留 wheel_link 绕车辆 Y 轴的滚动角。"""
    import math

    _qx, qy, _qz, qw = rotation_delta(current_pose, initial_pose)
    angle = 2.0 * math.atan2(qy, qw)
    half = 0.5 * angle
    return 0.0, math.sin(half), 0.0, math.cos(half)


def build_semantic_tf_specs(stage, vehicles, ros_frame, disable_builtin_tf):
    """发现模型能力并构造静态/动态语义 TF 规格。"""
    import math
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    _semantic_tf_specs = {}

    def _pose_override(frame_id, automatic_pose, extrinsics):
        """使用 YAML 中相对语义 base_link 的外参覆盖 USD 自动计算值。"""
        overrides = extrinsics.get("frames", {}) if isinstance(extrinsics, dict) else {}
        leaf = str(frame_id).rsplit("/", 1)[-1]
        entry = overrides.get(frame_id, overrides.get(leaf))
        if not isinstance(entry, dict):
            return automatic_pose
        xyz = entry.get("xyz", automatic_pose[:3])
        if len(xyz) != 3:
            raise ValueError(f"TF extrinsic '{leaf}.xyz' must contain 3 values")
        if "quaternion_xyzw" in entry:
            quat = entry["quaternion_xyzw"]
            if len(quat) != 4:
                raise ValueError(
                    f"TF extrinsic '{leaf}.quaternion_xyzw' must contain 4 values")
        elif "rpy_deg" in entry:
            rpy = entry["rpy_deg"]
            if len(rpy) != 3:
                raise ValueError(f"TF extrinsic '{leaf}.rpy_deg' must contain 3 values")
            roll, pitch, yaw = (math.radians(float(value)) for value in rpy)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            quat = (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )
        else:
            quat = automatic_pose[3:7]
        return tuple(float(value) for value in (*xyz, *quat))

    def _node_type(_prim):
        _attr = _prim.GetAttribute("node:type")
        return str(_attr.Get() or "") if _attr else ""

    _relative_pose = relative_pose

    for _p0_veh in vehicles:
        if not _p0_veh.get("enabled", True):
            continue
        _p0_name = _p0_veh.get("name", "Vehicle")
        _p0_root = f"/World/{_p0_name}"
        _p0_prefix = "/" + _p0_veh.get(
            "topic_prefix", f"/{_p0_name.lower()}"
        ).strip("/")
        _p0_base_frame = ros_frame(_p0_veh, "base_link", "base_link")
        _p0_footprint_frame = ros_frame(
            _p0_veh, "base_footprint", "base_footprint")
        _p0_chassis_frame = ros_frame(_p0_veh, "chassis", "chassis_link")
        _p0_camera_frame = ros_frame(_p0_veh, "camera", "camera_frame")
        _p0_camera_optical = ros_frame(
            _p0_veh, "camera_optical", "camera_optical_frame")
        _p0_imu_frame = ros_frame(_p0_veh, "imu", "imu_link")
        _p0_lidar_frame = ros_frame(_p0_veh, "lidar", "lidar_link")
        _p0_extrinsics = _p0_veh.get("tf_extrinsics", {})
        if not isinstance(_p0_extrinsics, dict):
            raise ValueError(f"{_p0_name}.tf_extrinsics must be a mapping")

        _p0_chassis = None
        _p0_chassis_container = None
        for _candidate in stage.Traverse():
            _candidate_path = _candidate.GetPath().pathString
            if (_candidate_path.startswith(_p0_root)
                    and _candidate_path.endswith("/Rigid_Bodies/Chassis")):
                _p0_chassis_container = _candidate
                break
        if (_p0_chassis_container is None
                or not _p0_chassis_container.IsValid()):
            print(f"[P0 TF] {_p0_name}: missing Chassis prim; semantic TF skipped")
            continue
        # 部分模型把 Chassis 作为组织层级，真正的刚体位于其下的 base_link。
        # 所有运动学参考必须绑定实际带 RigidBodyAPI 的 Prim，不能绑定静态容器。
        _p0_chassis_candidates = [
            prim for prim in stage.Traverse()
            if prim.GetPath().pathString.startswith(
                _p0_chassis_container.GetPath().pathString)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and (prim == _p0_chassis_container or prim.GetName() == "base_link")
        ]
        if _p0_chassis_candidates:
            _p0_odom_chassis = min(
                _p0_chassis_candidates,
                key=lambda prim: (
                    0 if prim.GetName() == "base_link" else 1,
                    prim.GetPath().pathString.count("/"),
                    prim.GetPath().pathString,
                ),
            )
        else:
            print(f"[P0 TF] {_p0_name}: Chassis subtree has no RigidBodyAPI; "
                  "semantic TF skipped")
            continue
        _p0_base_candidate = stage.GetPrimAtPath(
            _p0_chassis_container.GetPath().AppendChild("base_link"))
        _p0_chassis = (_p0_base_candidate if _p0_base_candidate.IsValid()
                       else _p0_odom_chassis)
        _p0_model_root = _p0_chassis_container.GetPath().pathString.removesuffix(
            "/Rigid_Bodies/Chassis")
        print(f"[P0 TF] {_p0_name}: odometry rigid body -> "
              f"{_p0_odom_chassis.GetPath()}; base pose reference -> "
              f"{_p0_chassis.GetPath()}")

        _p0_render_products = []
        _p0_camera_prim = None
        _p0_has_camera_info_helper = False
        _p0_imu_prim = None
        _p0_lidar_prim = None
        _p1_camera_prims = []
        _p1_imu_prims = []
        _p1_lidar_prims = []
        _p2_radar_prims = []
        _p1_wheel_meshes = {}

        for _p0_prim in stage.Traverse():
            _p0_path = _p0_prim.GetPath().pathString
            if not _p0_path.startswith(_p0_root):
                continue
            _p0_ntype = _node_type(_p0_prim)
            if "CameraInfoHelper" in _p0_ntype:
                _p0_has_camera_info_helper = True

            # P1：收集模型中真实存在的全部传感器和轮胎视觉 Mesh。轮胎刚体
            # 原点不能作为轮轴中心，因此必须使用其子 Mesh 的实时世界姿态。
            if _p0_prim.IsA(UsdGeom.Camera):
                _p1_camera_prims.append(_p0_prim)
            if _p0_prim.GetTypeName() == "IsaacImuSensor":
                _p1_imu_prims.append(_p0_prim)
            if "OmniLidar" in _p0_prim.GetTypeName():
                _p1_lidar_prims.append(_p0_prim)
            if "Radar" in _p0_prim.GetTypeName():
                _p2_radar_prims.append(_p0_prim)
            if (_p0_prim.IsA(UsdGeom.Mesh)
                    and _p0_path.startswith(f"{_p0_model_root}/Rigid_Bodies/Wheel_")):
                _p1_wheel_name = _p0_path.split("/Rigid_Bodies/", 1)[1].split("/", 1)[0]
                _p1_wheel_meshes.setdefault(_p1_wheel_name, []).append(_p0_prim)

            # 里程计必须以真实 Chassis 为数据源，而不是固定在车体上的相机刚体。
            if "ComputeOdometry" in _p0_ntype:
                _p0_chassis_rel = _p0_prim.GetRelationship("inputs:chassisPrim")
                if _p0_chassis_rel:
                    _p0_old = list(_p0_chassis_rel.GetTargets())
                    _p0_chassis_rel.SetTargets([_p0_odom_chassis.GetPath()])
                    print(f"[P0 Odom] {_p0_name}: {_p0_path} chassisPrim "
                          f"{_p0_old} -> {_p0_odom_chassis.GetPath()}")

            # 找出所有连接到真实相机 Prim 的有效 Render Product。
            if "CreateRenderProduct" in _p0_ntype:
                _p0_cam_rel = _p0_prim.GetRelationship("inputs:cameraPrim")
                _p0_cams = list(_p0_cam_rel.GetTargets()) if _p0_cam_rel else []
                if len(_p0_cams) == 1 and stage.GetPrimAtPath(_p0_cams[0]).IsValid():
                    _p0_output = _p0_prim.GetAttribute("outputs:renderProductPath")
                    if _p0_output:
                        _p0_render_products.append(_p0_output.GetPath())
                        if _p0_camera_prim is None:
                            _p0_camera_prim = stage.GetPrimAtPath(_p0_cams[0])

            if "ReadIMU" in _p0_ntype:
                _p0_imu_rel = _p0_prim.GetRelationship("inputs:imuPrim")
                _p0_imus = list(_p0_imu_rel.GetTargets()) if _p0_imu_rel else []
                if len(_p0_imus) == 1 and stage.GetPrimAtPath(_p0_imus[0]).IsValid():
                    _p0_imu_prim = stage.GetPrimAtPath(_p0_imus[0])

            if "PublishTransformTree" in _p0_ntype and disable_builtin_tf:
                # 该 Isaac Sim 版本的 TransformTree 会忽略运行时 enabled=false。
                # 在 OmniGraph 编译前将 Prim 停用，才能保证错误的 CAD 刚体 TF
                # 不再发布。覆盖只存在于组合 Stage，不会修改源 USD。
                _p0_exec = _p0_prim.GetAttribute("inputs:execIn")
                if _p0_exec:
                    _p0_exec.SetConnections([])
                _p0_prim.SetActive(False)
                print(f"[P0 TF] {_p0_name}: disabled invalid built-in TransformTree {_p0_path}")

            if _p0_prim.GetTypeName() == "IsaacImuSensor" and _p0_imu_prim is None:
                _p0_imu_prim = _p0_prim
            if "OmniLidar" in _p0_prim.GetTypeName():
                _p0_lidar_prim = _p0_prim

        # P2：将“模型中存在什么”和“用户希望启用什么”分开。相机可从任意
        # Camera Prim 生成 RGB/Depth/CameraInfo；双目要求至少两个相机。
        _p2_detected = {
            "rgb": bool(_p1_camera_prims),
            "depth": bool(_p1_camera_prims),
            "camera_info": bool(_p1_camera_prims),
            "stereo": len(_p1_camera_prims) >= 2,
            "imu": bool(_p1_imu_prims),
            "realsense_imu": len(_p1_imu_prims) >= 2,
            "lidar": bool(_p1_lidar_prims),
            "gnss": True,  # 由启动器根据车辆世界位姿仿真
            "radar": bool(_p2_radar_prims),
        }
        _p2_declared = _p0_veh.get("sensor_capabilities", {})
        _p2_enable_keys = {
            "rgb": "enable_rgb", "depth": "enable_depth",
            "camera_info": "enable_camera_info", "stereo": "enable_stereo",
            "imu": "enable_imu", "realsense_imu": "enable_realsense_imu",
            "lidar": "enable_lidar", "gnss": "enable_gnss",
            "radar": "enable_radar",
        }
        _p2_legacy_defaults = {
            "rgb": bool(_p0_veh.get("enable_camera", True)),
            "depth": bool(_p0_veh.get("enable_camera", True)),
            "camera_info": bool(_p0_veh.get("enable_camera", True)),
            "stereo": False,
            "imu": True,
            "realsense_imu": False,
            "lidar": bool(_p0_veh.get("enable_lidar", True)),
            "gnss": bool(_p0_veh.get("enable_gnss", True)),
            "radar": False,
        }
        _p2_effective = {}
        for _p2_sensor, _p2_enable_key in _p2_enable_keys.items():
            _p2_requested = bool(_p0_veh.get(
                _p2_enable_key, _p2_legacy_defaults[_p2_sensor],
            ))
            _p2_available = bool(_p2_detected[_p2_sensor])
            _p2_effective[_p2_sensor] = _p2_requested and _p2_available
            if _p2_sensor in _p2_declared and bool(_p2_declared[_p2_sensor]) != _p2_available:
                print(f"[P2 Sensors] WARNING {_p0_name}: declared capability "
                      f"'{_p2_sensor}={bool(_p2_declared[_p2_sensor])}' but Stage "
                      f"detection is '{_p2_available}'")
            if _p2_requested and not _p2_available:
                print(f"[P2 Sensors] WARNING {_p0_name}: requested '{_p2_sensor}', "
                      "but the model has no matching sensor Prim; publisher disabled")
        _p2_gnss_mode = str(_p0_veh.get("gnss_mode", "simulated")).lower()
        if _p2_effective["gnss"] and _p2_gnss_mode != "simulated":
            print(f"[P2 Sensors] WARNING {_p0_name}: unsupported gnss_mode="
                  f"'{_p2_gnss_mode}'; expected 'simulated'; GNSS disabled")
            _p2_effective["gnss"] = False
        _p0_veh["_sensor_effective"] = _p2_effective
        _p2_unavailable = [
            key for key in _p2_enable_keys if not _p2_detected[key]
        ]
        _p2_disabled = [
            key for key in _p2_enable_keys
            if _p2_detected[key] and not _p2_effective[key]
        ]
        _p2_summary = ", ".join(
            f"{key}={'on' if _p2_effective[key] else 'off'}"
            for key in _p2_enable_keys)
        print(f"[P2 Sensors] {_p0_name}: {_p2_summary}")
        if _p2_unavailable:
            print(f"[P2 Sensors] {_p0_name}: model does not provide: "
                  f"{', '.join(_p2_unavailable)}; no publisher will be created")
        if _p2_disabled:
            print(f"[P2 Sensors] {_p0_name}: available but disabled by configuration: "
                  f"{', '.join(_p2_disabled)}")

        # 深度 Helper 只允许连接一个有效 Render Product。源 OffRoad USD 中还保留了
        # 指向不存在 isaac_create_render_product_01 的悬空连接，这会导致深度图异常。
        if _p0_render_products:
            _p0_valid_render = _p0_render_products[0]
            for _p0_prim in stage.Traverse():
                _p0_path = _p0_prim.GetPath().pathString
                if not _p0_path.startswith(_p0_root):
                    continue
                _p0_ntype = _node_type(_p0_prim)
                if "Camera" not in _p0_ntype or "Helper" not in _p0_ntype:
                    continue
                _p0_type_attr = _p0_prim.GetAttribute("inputs:type")
                _p0_topic_attr = _p0_prim.GetAttribute("inputs:topicName")
                _p0_kind = str(_p0_type_attr.Get() or "") if _p0_type_attr else ""
                _p0_topic = str(_p0_topic_attr.Get() or "") if _p0_topic_attr else ""
                _p0_render_attr = _p0_prim.GetAttribute("inputs:renderProductPath")
                if _p0_render_attr and (_p0_kind == "depth" or _p0_topic.strip("/") == "depth"):
                    _p0_old_connections = list(_p0_render_attr.GetConnections())
                    _p0_render_attr.SetConnections([_p0_valid_render])
                    print(f"[P0 Depth] {_p0_name}: {_p0_path} render product "
                          f"{_p0_old_connections} -> [{_p0_valid_render}]")
                _p0_frame_attr = _p0_prim.GetAttribute("inputs:frameId")
                if _p0_frame_attr:
                    _p0_frame_attr.Set(_p0_camera_optical)

        # IMU 和 LiDAR 消息使用各自的真实传感器 frame，而不是 base_link。
        for _p0_prim in stage.Traverse():
            _p0_path = _p0_prim.GetPath().pathString
            if not _p0_path.startswith(_p0_root):
                continue
            _p0_ntype = _node_type(_p0_prim)
            _p0_frame_attr = _p0_prim.GetAttribute("inputs:frameId")
            if not _p0_frame_attr:
                continue
            if "PublishImu" in _p0_ntype:
                _p0_frame_attr.Set(_p0_imu_frame)
            elif "LidarHelper" in _p0_ntype:
                _p0_frame_attr.Set(_p0_lidar_frame)

        # 根据轮胎世界包围盒下边界推导 base_footprint；base_link 定义在 Chassis
        # 原点。这样不会把出生高度或 CAD 根节点偏移误认为底盘离地高度。
        _p0_ground_z = None
        try:
            _p0_bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            for _p0_prim in stage.Traverse():
                _p0_path = _p0_prim.GetPath().pathString
                if (not _p0_path.startswith(f"{_p0_model_root}/Rigid_Bodies/Wheel_")
                        or not _p0_prim.IsA(UsdGeom.Mesh)):
                    continue
                _p0_min_z = float(
                    _p0_bbox_cache.ComputeWorldBound(_p0_prim)
                    .ComputeAlignedBox().GetMin()[2])
                _p0_ground_z = _p0_min_z if _p0_ground_z is None else min(
                    _p0_ground_z, _p0_min_z)
        except Exception as _p0_bbox_err:
            print(f"[P0 TF] {_p0_name}: wheel ground detection failed: {_p0_bbox_err}")

        _p0_chassis_z = float(
            omni.usd.get_world_transform_matrix(_p0_chassis).ExtractTranslation()[2])
        _p0_base_height = max(0.0, _p0_chassis_z - _p0_ground_z) \
            if _p0_ground_z is not None else 0.0

        _p0_static = [
            (_p0_footprint_frame, _p0_base_frame,
             (0.0, 0.0, _p0_base_height, 0.0, 0.0, 0.0, 1.0)),
            (_p0_base_frame, _p0_chassis_frame,
             (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
        ]
        _p1_static_children = {_p0_base_frame, _p0_chassis_frame}

        def _p1_add_static(_parent, _child, _pose):
            if not _child or _child in _p1_static_children:
                return
            _p0_static.append((_parent, _child, _pose))
            _p1_static_children.add(_child)

        # 为每个相机建立 mount frame 和 optical frame。当前实际发布图像的相机
        # 沿用 camera/camera_optical 配置；其余相机按名称映射到独立 frame。
        for _p1_camera in _p1_camera_prims:
            _p1_camera_path = _p1_camera.GetPath().pathString
            _p1_camera_name = _p1_camera.GetName().lower()
            if (_p0_camera_prim is not None
                    and _p1_camera_path == _p0_camera_prim.GetPath().pathString):
                _p1_camera_key, _p1_optical_key = "camera", "camera_optical"
                _p1_camera_default, _p1_optical_default = "camera_frame", "camera_optical_frame"
            elif "right" in _p1_camera_name:
                _p1_camera_key, _p1_optical_key = "camera_right", "camera_right_optical"
                _p1_camera_default, _p1_optical_default = "camera_right_frame", "camera_right_optical_frame"
            elif "left" in _p1_camera_name:
                _p1_camera_key, _p1_optical_key = "camera_left", "camera_left_optical"
                _p1_camera_default, _p1_optical_default = "camera_left_frame", "camera_left_optical_frame"
            elif "depth" in _p1_camera_name:
                _p1_camera_key, _p1_optical_key = "camera_depth", "camera_depth_optical"
                _p1_camera_default, _p1_optical_default = "camera_depth_frame", "camera_depth_optical_frame"
            else:
                _p1_camera_key, _p1_optical_key = "camera_color", "camera_color_optical"
                _p1_camera_default, _p1_optical_default = "camera_color_frame", "camera_color_optical_frame"
            _p1_camera_frame = ros_frame(_p0_veh, _p1_camera_key, _p1_camera_default)
            _p1_optical_frame = ros_frame(_p0_veh, _p1_optical_key, _p1_optical_default)
            _p1_add_static(
                _p0_base_frame, _p1_camera_frame,
                _relative_pose(_p0_chassis, _p1_camera))
            # USD Camera：+X 向右、+Y 向上、-Z 向前；ROS optical：+X 向右、
            # +Y 向下、+Z 向前，因此需要绕 X 轴旋转 180°。
            _p1_add_static(
                _p1_camera_frame, _p1_optical_frame,
                (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))

        for _p1_imu in _p1_imu_prims:
            _p1_imu_path = _p1_imu.GetPath().pathString
            _p1_is_active = (_p0_imu_prim is not None
                             and _p1_imu_path == _p0_imu_prim.GetPath().pathString)
            _p1_imu_frame = (_p0_imu_frame if _p1_is_active else
                             ros_frame(_p0_veh, "realsense_imu", "realsense_imu_link"))
            _p1_add_static(
                _p0_base_frame, _p1_imu_frame,
                _relative_pose(_p0_chassis, _p1_imu))

        for _p1_lidar in _p1_lidar_prims:
            _p1_add_static(
                _p0_base_frame, _p0_lidar_frame,
                _relative_pose(_p0_chassis, _p1_lidar))

        _p1_wheel_key_map = {
            "Wheel_Front_Left": "front_left_wheel",
            "Wheel_Front_Right": "front_right_wheel",
            "Wheel_Rear_Left": "rear_left_wheel",
            "Wheel_Rear_Right": "rear_right_wheel",
        }
        _p1_wheels = []
        for _p1_wheel_name, _p1_wheel_key in _p1_wheel_key_map.items():
            _p1_candidates = _p1_wheel_meshes.get(_p1_wheel_name, [])
            if not _p1_candidates:
                print(f"[P1 TF] {_p0_name}: missing visual Mesh for {_p1_wheel_name}")
                continue
            # 选择层级最浅的 Mesh；它是承载轮胎中心变换的视觉根 Mesh。
            _p1_wheel_prim = min(
                _p1_candidates,
                key=lambda _prim: (_prim.GetPath().pathString.count("/"),
                                   _prim.GetPath().pathString),
            )
            _p1_wheel_frame = ros_frame(
                _p0_veh, _p1_wheel_key, f"{_p1_wheel_key}_link")
            _p1_wheel_body = stage.GetPrimAtPath(
                f"{_p0_model_root}/Rigid_Bodies/{_p1_wheel_name}")
            if not _p1_wheel_body.IsValid():
                print(f"[P1 TF] {_p0_name}: missing rigid body for {_p1_wheel_name}")
                continue
            _p1_wheels.append(
                (_p1_wheel_frame, _p1_wheel_prim, _p1_wheel_body))

        # base_footprint 的 x/y 定义为四个轮轴中心的几何中点，z 定义为
        # 轮胎接地点所在的车辆支撑平面。Odometry 的位姿来自 base_link，
        # 因此需要同时保存 base_link -> base_footprint 的局部偏移；随后
        # odom -> base_footprint 使用该偏移，base_footprint -> base_link
        # 发布其反向静态变换，保证两段 TF 合成后仍等于原始里程计位姿。
        _p1_wheel_poses = [
            (_wheel_frame, _relative_pose(_p0_chassis, _wheel_prim))
            for _wheel_frame, _wheel_prim, _wheel_body in _p1_wheels
            if _wheel_prim.IsValid()
        ]
        _p1_wheel_centers = [_pose for _wheel_frame, _pose in _p1_wheel_poses]
        if _p1_wheel_centers:
            _p1_auto_center_x = sum(_pose[0] for _pose in _p1_wheel_centers) \
                / len(_p1_wheel_centers)
            _p1_auto_center_y = sum(_pose[1] for _pose in _p1_wheel_centers) \
                / len(_p1_wheel_centers)
        else:
            _p1_auto_center_x = 0.0
            _p1_auto_center_y = 0.0
        _p1_center_xy = _p0_extrinsics.get(
            "base_link_center_xy", [_p1_auto_center_x, _p1_auto_center_y])
        if len(_p1_center_xy) != 2:
            raise ValueError(
                f"{_p0_name}.tf_extrinsics.base_link_center_xy must contain 2 values")
        _p1_footprint_x, _p1_footprint_y = (
            float(_p1_center_xy[0]), float(_p1_center_xy[1]))
        _p0_base_height = float(
            _p0_extrinsics.get("base_link_height", _p0_base_height))
        _p1_footprint_offset = (
            float(_p1_footprint_x), float(_p1_footprint_y),
            -float(_p0_base_height))

        # 将此前从 CAD Chassis 原点计算出的所有 base_link 子 frame 外参，
        # 统一改写为相对于四轮几何中心处的语义 base_link。base_footprint 与
        # base_link 的 x/y 因而完全重合，只保留固定高度差。
        _p0_static[0] = (
            _p0_footprint_frame, _p0_base_frame,
            (0.0, 0.0, _p0_base_height, 0.0, 0.0, 0.0, 1.0),
        )
        for _p1_index in range(1, len(_p0_static)):
            _parent, _child, _pose = _p0_static[_p1_index]
            if _parent != _p0_base_frame:
                continue
            _rebased_pose = (
                float(_pose[0]) - _p1_footprint_x,
                float(_pose[1]) - _p1_footprint_y,
                float(_pose[2]), *_pose[3:7])
            _p0_static[_p1_index] = (
                _parent, _child,
                _pose_override(_child, _rebased_pose, _p0_extrinsics))
        # 固定位置来自 YAML/USD authored 轮轴中心；旋转来自 Wheel 刚体
        # 相对启动姿态的增量，避免把悬架位移误当作安装外参。
        _p1_dynamic_wheels = []
        _p1_wheel_body_by_frame = {
            _frame: _body for _frame, _prim, _body in _p1_wheels}
        for _wheel_frame, _wheel_pose in _p1_wheel_poses:
            _rebased_wheel_pose = (
                float(_wheel_pose[0]) - _p1_footprint_x,
                float(_wheel_pose[1]) - _p1_footprint_y,
                float(_wheel_pose[2]), 0.0, 0.0, 0.0, 1.0)
            _fixed_pose = _pose_override(
                _wheel_frame, _rebased_wheel_pose, _p0_extrinsics)
            _wheel_body = _p1_wheel_body_by_frame[_wheel_frame]
            _p1_dynamic_wheels.append((
                _wheel_frame, _wheel_body,
                _relative_pose(_p0_chassis, _wheel_body),
                _fixed_pose[:3]))
        print(f"[P0 TF] {_p0_name}: semantic base_link center in CAD frame = "
              f"({_p1_footprint_x:.6f}, {_p1_footprint_y:.6f}); "
              f"base_footprint -> base_link = (0, 0, {_p0_base_height:.6f})")

        _semantic_tf_specs[_p0_name] = {
            "odom_topic": _p0_prefix + "/odom",
            "odom_frame": ros_frame(_p0_veh, "odom", "odom"),
            "footprint_frame": _p0_footprint_frame,
            "base_frame": _p0_base_frame,
            "base_height": _p0_base_height,
            "base_center_offset": (
                _p1_footprint_x, _p1_footprint_y, 0.0),
            "footprint_offset": _p1_footprint_offset,
            "static": _p0_static,
            "chassis_prim": _p0_chassis,
            "model_root": _p0_model_root,
            "wheels": _p1_dynamic_wheels,
            "camera_prims": _p1_camera_prims,
            "active_camera_prim": _p0_camera_prim,
            "active_render_product_attr": (
                _p0_render_products[0] if _p0_render_products else None
            ),
            "has_camera_info_helper": _p0_has_camera_info_helper,
            "imu_prims": _p1_imu_prims,
            "active_imu_prim": _p0_imu_prim,
            "lidar_prims": _p1_lidar_prims,
            "sensor_effective": _p2_effective,
        }
        print(f"[P1 TF] {_p0_name}: semantic frames prepared; "
              f"base_link height={_p0_base_height:.6f} m, "
              f"static TFs={len(_p0_static)}, dynamic wheels={len(_p1_dynamic_wheels)}")
    return _semantic_tf_specs
