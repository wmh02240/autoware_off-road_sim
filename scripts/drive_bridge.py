#!/usr/bin/env python3
"""Isaac Sim 的 ROS 2 驾驶指令桥接程序。

Isaac Sim 使用 Python 3.12，但 ROS Humble 中的 rclpy 是为 Python 3.10 编译的。
本脚本作为 Python 3.10 子进程运行，用于处理 ROS 2 驾驶指令订阅，
并可选发布地图和静态 TF；其实现模式与 gnss_bridge.py 相同。

标准输入协议（每行一条命令）：
  执行 ``start`` 之前：
    sub<TAB>veh_name<TAB>drive_topic<TAB>control_topic
    odom_tf<TAB>odom_topic<TAB>parent_frame<TAB>child_frame
  执行 ``start`` 之后（延迟命令，例如物理预热完成后地图数据才就绪）：
    map<TAB>width<TAB>height<TAB>resolution<TAB>orig_x<TAB>orig_y<TAB>data_b64
    tf<TAB>parent_frame<TAB>child_frame
    tf_pose<TAB>parent_frame<TAB>child_frame<TAB>x<TAB>y<TAB>z<TAB>qx<TAB>qy<TAB>qz<TAB>qw
    ctrl_mode<TAB>veh_name<TAB>value  -- 发布控制模式（0=KEYBOARD_CONTROL，1=ROS2_CONTROL）

标准输出协议：
  ready                                           -- 节点已启动且订阅已注册
  cmd<TAB>veh_name<TAB>speed<TAB>steer<TAB>source -- 每条收到的驾驶指令

标准输入关闭（父进程退出）时正常结束。
"""

import sys
import os
import signal
import struct
import base64
import threading

# 忽略 SIGINT——关闭流程由父进程（Isaac Sim）负责。
signal.signal(signal.SIGINT, signal.SIG_IGN)

import rclpy
from rclpy.node import Node

rclpy.init()
_node = Node("isaacsim_drive_bridge")

# 将 tf2 缓存时间保持为 2 秒（默认值为 10 秒）。
# 该值必须小于重启时 set_current_time 使用的偏移量（launch_sim.py 中为 3.0 秒），
# 这样仿真时钟向前跳变 3 秒时，所有旧 TF 条目（最多早 2 秒）都会立即被清除，
# 从第一个物理帧开始即可为 RViz 提供干净的 TF 数据，避免出现 TF_OLD_DATA 空窗期。
try:
    from tf2_ros import Buffer as _TF2Buffer, TransformListener as _TFListener
    from rclpy.duration import Duration as _Duration
    _tf_buffer   = _TF2Buffer(cache_time=_Duration(seconds=2.0))
    _tf_listener = _TFListener(_tf_buffer, _node)
except Exception as _e:
    sys.stderr.write(f"[drive_bridge] tf2 buffer init skipped: {_e}\n")

_pending_subs = []   # (veh_name, drive_topic, control_topic) 列表
_pending_odom_tfs = []  # (odom_topic, parent_frame, child_frame) 列表
_pending_maps = []   # 原始字段列表 [width, height, res, ox, oy, b64]
_pending_tfs  = []   # (parent_frame, child_frame) 列表
_pending_pose_tfs = []  # (parent, child, x, y, z, qx, qy, qz, qw) 列表
_ctrl_mode_pubs = {}  # 映射：veh_name -> rclpy Publisher<Int32>

# ── 阶段 1：从标准输入读取注册信息，直到收到 "start" ─────────────────────────
for _line in sys.stdin:
    _line = _line.rstrip("\n")
    if _line == "start":
        break
    _parts = _line.split("\t")
    if not _parts:
        continue
    _cmd = _parts[0]
    if _cmd == "sub" and len(_parts) == 4:
        _pending_subs.append((_parts[1], _parts[2], _parts[3]))
    elif _cmd == "odom_tf" and len(_parts) == 4:
        _pending_odom_tfs.append((_parts[1], _parts[2], _parts[3]))
    elif _cmd == "map" and len(_parts) == 7:
        _pending_maps.append(_parts[1:])
    elif _cmd == "tf" and len(_parts) == 3:
        _pending_tfs.append((_parts[1], _parts[2]))
    elif _cmd == "tf_pose" and len(_parts) == 10:
        _pending_pose_tfs.append(_parts[1:])

# ── 阶段 2：注册驾驶指令订阅 ──────────────────────────────────────────────────
def _pub_cmd(veh_name, speed, steer, source):
    sys.stdout.write(f"cmd\t{veh_name}\t{speed}\t{steer}\t{source}\n")
    sys.stdout.flush()

for _vn, _dt, _ct in _pending_subs:
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
        def _make_ack_cb(v):
            def _cb(msg):
                _pub_cmd(v, msg.drive.speed, msg.drive.steering_angle, "ackermann")
            return _cb
        _node.create_subscription(AckermannDriveStamped, _dt, _make_ack_cb(_vn), 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: subscribed '{_dt}' [AckermannDriveStamped]\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: AckermannDriveStamped failed: {_e}\n")
    try:
        from autoware_control_msgs.msg import Control
        from ackermann_msgs.msg import AckermannDriveStamped
        _ctrl_pub = _node.create_publisher(AckermannDriveStamped, _dt, 10)
        def _make_ctrl_cb(v, pub, dt):
            def _cb(msg):
                spd = msg.longitudinal.velocity
                steer = msg.lateral.steering_tire_angle
                _pub_cmd(v, spd, steer, "autoware")
                # 重新发布为 AckermannDriveStamped，使驾驶话题上的内置 OmniGraph 订阅器能够直接接收该指令。
                ack = AckermannDriveStamped()
                ack.header.stamp = _node.get_clock().now().to_msg()
                ack.drive.speed = float(spd)
                ack.drive.steering_angle = float(steer)
                pub.publish(ack)
            return _cb
        _node.create_subscription(Control, _ct, _make_ctrl_cb(_vn, _ctrl_pub, _dt), 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: subscribed '{_ct}' [autoware_control_msgs/Control] → republish on '{_dt}'\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: autoware_control_msgs/Control failed: {_e}\n")
    try:
        from std_msgs.msg import Int32 as _Int32
        _cm_topic = _dt.rsplit("/", 1)[0] + "/control_mode"
        _ctrl_mode_pubs[_vn] = _node.create_publisher(_Int32, _cm_topic, 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: control_mode publisher on '{_cm_topic}'\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: control_mode publisher failed: {_e}\n")

# Isaac 里程计发布器会发布 nav_msgs/Odometry，但源车辆图不会发布对应的
# odom -> base_link TF。将每个里程计位姿同步到 /tf，使使用方看到一棵连通的树。
# 此处复制消息时间戳；若使用墙上时间，该变换将无法与 /clock 配合使用。
_tf_broadcaster = None
if _pending_odom_tfs:
    try:
        from tf2_ros import TransformBroadcaster
        _tf_broadcaster = TransformBroadcaster(_node)
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] dynamic TF broadcaster failed: {_e}\n")

for _odom_topic, _parent_frame, _child_frame in _pending_odom_tfs:
    try:
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped

        def _make_odom_tf_cb(parent_frame, child_frame):
            def _cb(msg):
                if _tf_broadcaster is None:
                    return
                tf_msg = TransformStamped()
                tf_msg.header.stamp = msg.header.stamp
                tf_msg.header.frame_id = parent_frame
                tf_msg.child_frame_id = child_frame
                tf_msg.transform.translation.x = msg.pose.pose.position.x
                tf_msg.transform.translation.y = msg.pose.pose.position.y
                tf_msg.transform.translation.z = msg.pose.pose.position.z
                tf_msg.transform.rotation = msg.pose.pose.orientation
                _tf_broadcaster.sendTransform(tf_msg)
            return _cb

        _node.create_subscription(
            Odometry,
            _odom_topic,
            _make_odom_tf_cb(_parent_frame, _child_frame),
            10,
        )
        sys.stderr.write(
            f"[drive_bridge] TF from '{_odom_topic}': "
            f"{_parent_frame} -> {_child_frame}\n"
        )
    except Exception as _e:
        sys.stderr.write(
            f"[drive_bridge] odom TF subscription '{_odom_topic}' failed: {_e}\n"
        )

# ── 阶段 3：在后台线程中开始 spin ─────────────────────────────────────────────
_spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
_spin_thread.start()

# ── 阶段 4：通知父进程已就绪 ──────────────────────────────────────────────────
sys.stdout.write("ready\n")
sys.stdout.flush()


# ── 辅助函数：发布 /map 和静态 TF ─────────────────────────────────────────────
def _publish_map(parts):
    try:
        import struct as _s, base64 as _b
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        _w, _h, _res, _ox, _oy, _db64 = parts
        _w, _h = int(_w), int(_h)
        _res, _ox, _oy = float(_res), float(_ox), float(_oy)
        _raw  = _b.b64decode(_db64)
        _data = list(_s.unpack(f"{len(_raw)}b", _raw))
        _qos  = QoSProfile(depth=1,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)
        _pub  = _node.create_publisher(OccupancyGrid, "/map", _qos)
        _msg  = OccupancyGrid()
        _msg.header.frame_id            = "map"
        _msg.header.stamp               = _node.get_clock().now().to_msg()
        _msg.info.width                 = _w
        _msg.info.height                = _h
        _msg.info.resolution            = _res
        _msg.info.origin.position.x     = _ox
        _msg.info.origin.position.y     = _oy
        _msg.info.origin.orientation.w  = 1.0
        _msg.data                       = _data
        _pub.publish(_msg)
        sys.stderr.write(f"[drive_bridge] Published /map ({_w}×{_h} @ {_res} m/cell)\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] map publish failed: {_e}\n")


_stf_broadcaster = None  # 收到第一条 tf 命令时再延迟创建

def _publish_tf(parent, child, pose=None):
    global _stf_broadcaster
    try:
        from tf2_ros import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        if _stf_broadcaster is None:
            _stf_broadcaster = StaticTransformBroadcaster(_node)
        _t = TransformStamped()
        _t.header.stamp     = _node.get_clock().now().to_msg()
        _t.header.frame_id  = parent
        _t.child_frame_id   = child
        if pose is None:
            _t.transform.rotation.w = 1.0
        else:
            x, y, z, qx, qy, qz, qw = (float(value) for value in pose)
            _t.transform.translation.x = x
            _t.transform.translation.y = y
            _t.transform.translation.z = z
            _t.transform.rotation.x = qx
            _t.transform.rotation.y = qy
            _t.transform.rotation.z = qz
            _t.transform.rotation.w = qw
        _stf_broadcaster.sendTransform([_t])
        sys.stderr.write(f"[drive_bridge] Published static TF: {parent} → {child}\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] TF broadcast failed: {_e}\n")


# 处理 start 之前到达的所有地图和 TF 数据
for _mp in _pending_maps:
    _publish_map(_mp)
for _parent, _child in _pending_tfs:
    _publish_tf(_parent, _child)
for _pose_tf in _pending_pose_tfs:
    _publish_tf(_pose_tf[0], _pose_tf[1], _pose_tf[2:])


# ── 阶段 5：保持运行并处理延迟到达的标准输入命令 ───────────────────────────────
try:
    while True:
        _line = sys.stdin.readline()
        if not _line:
            break
        _line = _line.rstrip("\n")
        if not _line:
            continue
        _parts = _line.split("\t")
        _cmd = _parts[0] if _parts else ""
        if _cmd == "map" and len(_parts) == 7:
            _publish_map(_parts[1:])
        elif _cmd == "tf" and len(_parts) == 3:
            _publish_tf(_parts[1], _parts[2])
        elif _cmd == "tf_pose" and len(_parts) == 10:
            _publish_tf(_parts[1], _parts[2], _parts[3:])
        elif _cmd == "ctrl_mode" and len(_parts) == 3:
            try:
                from std_msgs.msg import Int32 as _Int32
                _cmvn, _cmval = _parts[1], int(_parts[2])
                if _cmvn in _ctrl_mode_pubs:
                    _cm_msg = _Int32()
                    _cm_msg.data = _cmval
                    _ctrl_mode_pubs[_cmvn].publish(_cm_msg)
            except Exception as _e:
                sys.stderr.write(f"[drive_bridge] ctrl_mode publish failed: {_e}\n")
except Exception:
    pass

try:
    if rclpy.ok():
        rclpy.shutdown()
except Exception:
    pass

try:
    _spin_thread.join(timeout=2.0)
except Exception:
    pass

try:
    _node.destroy_node()
except Exception:
    pass
