#!/usr/bin/env python3
"""Isaac Sim 的 GNSS 发布桥接程序（Python 3.10 子进程）。

Isaac Sim 使用 Python 3.12，但 ROS Humble 中的 rclpy 仅为 Python 3.10 编译。
launch_sim.py 使用 /usr/bin/python3.10 启动本脚本，从而通过 rclpy 发布 GNSS
消息，同时避免 Python 版本冲突。

协议（标准输入，每行一条记录）：
  topic<TAB>frame_id<TAB>lat<TAB>lon<TAB>alt<TAB>h_std<TAB>v_std<TAB>stamp_ns

  - lat、lon、alt  ：浮点数（度/米）
  - h_std、v_std   ：以米为单位的 1-sigma 噪声（此处平方后作为协方差）
  - stamp_ns       ：自纪元起的墙上时钟纳秒数（整数）

rclpy 节点启动后向标准输出写入 "ready\\n"；父进程读取该信号，
以免在桥接程序初始化完成前发送消息。

标准输入关闭（父进程退出）时正常结束。
"""

import sys
import os
import signal

# 忽略 SIGINT——关闭流程由父进程（Isaac Sim）负责。
signal.signal(signal.SIGINT, signal.SIG_IGN)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

rclpy.init()

_node = Node("isaacsim_gnss_bridge")
_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
_pubs = {}  # topic -> Publisher（收到第一条消息时再延迟创建）

# 通知父进程当前已就绪。
sys.stdout.write("ready\n")
sys.stdout.flush()

for _line in sys.stdin:
    _line = _line.rstrip("\n")
    if not _line:
        continue
    try:
        _topic, _fid, _lat, _lon, _alt, _hs, _vs, _ts = _line.split("\t")
        _lat, _lon, _alt = float(_lat), float(_lon), float(_alt)
        _ch = float(_hs) ** 2
        _cv = float(_vs) ** 2
        _ts = int(_ts)

        if _topic not in _pubs:
            _pubs[_topic] = _node.create_publisher(NavSatFix, _topic, _qos)

        _msg = NavSatFix()
        _msg.header.frame_id          = _fid
        _msg.header.stamp.sec         = _ts // 1_000_000_000
        _msg.header.stamp.nanosec     = _ts % 1_000_000_000
        _msg.status.status            = NavSatStatus.STATUS_FIX
        _msg.status.service           = NavSatStatus.SERVICE_GPS
        _msg.latitude                 = _lat
        _msg.longitude                = _lon
        _msg.altitude                 = _alt
        _msg.position_covariance      = [_ch, 0.0, 0.0,
                                         0.0, _ch, 0.0,
                                         0.0, 0.0, _cv]
        _msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        _pubs[_topic].publish(_msg)
    except Exception:
        pass

try:
    rclpy.shutdown()
except Exception:
    pass
