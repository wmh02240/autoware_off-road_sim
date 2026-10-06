"""RoboRacer USD/PhysX 兼容性修复。

这些修复只写入当前组合 Stage，不修改源 USD 文件。
"""


def repair_vehicle_physics(stage):
    """修复已知车辆资源中的 IMU、悬架和质量属性问题。"""
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics

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

