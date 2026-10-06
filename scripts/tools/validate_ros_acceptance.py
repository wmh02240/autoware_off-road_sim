#!/usr/bin/env python3
"""采集第 9 章验收所需的 ROS 2 消息与 TF 证据。"""

import argparse
import json
import math
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def image_summary(message):
    bytes_per_pixel = {
        "rgb8": 3,
        "rgba8": 4,
        "bgr8": 3,
        "bgra8": 4,
        "mono8": 1,
        "mono16": 2,
        "16UC1": 2,
        "32FC1": 4,
    }.get(message.encoding)
    expected_step = message.width * bytes_per_pixel if bytes_per_pixel else None
    summary = {
        "stamp_ns": stamp_ns(message.header),
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "encoding": message.encoding,
        "is_bigendian": message.is_bigendian,
        "step": message.step,
        "data_bytes": len(message.data),
        "expected_step": expected_step,
        "layout_valid": bool(
            message.step == expected_step and len(message.data) == message.step * message.height
            if expected_step is not None else False
        ),
    }
    if message.encoding == "32FC1" and not message.is_bigendian and len(message.data) >= 4:
        all_values = memoryview(message.data).cast("f")
        count = min(len(all_values), 4096)
        stride = max(1, len(all_values) // count)
        values = all_values[::stride][:count]
        finite = [value for value in values if math.isfinite(value)]
        positive = [value for value in finite if value > 0.0]
        summary["sample_count"] = count
        summary["finite_ratio"] = len(finite) / count
        summary["positive_ratio"] = len(positive) / count
        if positive:
            summary["positive_min"] = min(positive)
            summary["positive_max"] = max(positive)
    return summary


def camera_info_summary(message):
    return {
        "stamp_ns": stamp_ns(message.header),
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "distortion_model": message.distortion_model,
        "k": list(message.k),
        "p": list(message.p),
        "intrinsics_valid": bool(
            message.k[0] > 0.0 and message.k[4] > 0.0 and message.k[8] == 1.0
        ),
    }


class AcceptanceCollector(Node):
    def __init__(self, namespace, expect_lidar, expect_realsense_imu):
        super().__init__("acceptance_collector")
        prefix = "/" + namespace.strip("/")
        self.samples = defaultdict(list)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(depth=10)
        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.create_subscription(
            Image, prefix + "/left/image_raw", self._image_cb("left_image"), sensor_qos)
        self.create_subscription(
            Image, prefix + "/right/image_raw", self._image_cb("right_image"), sensor_qos)
        self.create_subscription(Image, prefix + "/depth", self._image_cb("depth"), sensor_qos)
        self.create_subscription(
            CameraInfo, prefix + "/left/camera_info",
            self._camera_info_cb("left_camera_info"), sensor_qos)
        self.create_subscription(
            CameraInfo, prefix + "/right/camera_info",
            self._camera_info_cb("right_camera_info"), sensor_qos)
        self.create_subscription(Imu, prefix + "/imu", self._imu_cb("imu"), sensor_qos)
        if expect_lidar:
            self.create_subscription(
                PointCloud2, prefix + "/point_cloud", self._point_cloud_cb, sensor_qos)
        if expect_realsense_imu:
            self.create_subscription(
                Imu, prefix + "/realsense/imu", self._imu_cb("realsense_imu"), sensor_qos)
        self.create_subscription(TFMessage, "/tf", self._tf_cb("tf"), reliable_qos)
        self.create_subscription(TFMessage, "/tf_static", self._tf_cb("tf_static"), static_qos)

    def _append(self, key, value, limit=100):
        values = self.samples[key]
        values.append(value)
        if len(values) > limit:
            del values[0]

    def _image_cb(self, key):
        return lambda message: self._append(key, image_summary(message))

    def _camera_info_cb(self, key):
        return lambda message: self._append(key, camera_info_summary(message))

    def _imu_cb(self, key):
        def callback(message):
            self._append(key, {
                "stamp_ns": stamp_ns(message.header),
                "frame_id": message.header.frame_id,
                "orientation_covariance": list(message.orientation_covariance),
                "angular_velocity": [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ],
                "linear_acceleration": [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ],
            })
        return callback

    def _point_cloud_cb(self, message):
        self._append("point_cloud", {
            "stamp_ns": stamp_ns(message.header),
            "frame_id": message.header.frame_id,
            "width": message.width,
            "height": message.height,
            "point_step": message.point_step,
            "row_step": message.row_step,
            "data_bytes": len(message.data),
            "layout_valid": bool(
                message.row_step == message.width * message.point_step
                and len(message.data) == message.row_step * message.height
            ),
            "fields": [field.name for field in message.fields],
        })

    def _tf_cb(self, key):
        def callback(message):
            for transform in message.transforms:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._append(key, {
                    "stamp_ns": stamp_ns(transform.header),
                    "parent": transform.header.frame_id,
                    "child": transform.child_frame_id,
                    "translation": [translation.x, translation.y, translation.z],
                    "rotation": [rotation.x, rotation.y, rotation.z, rotation.w],
                }, limit=1000)
        return callback


def quaternion_distance(first, last):
    dot = abs(sum(a * b for a, b in zip(first, last)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def quaternion_delta_axis(first, last):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = last
    # q_delta = q_last * inverse(q_first)，结果轴位于父坐标系中。
    x = -w2 * x1 + x2 * w1 - y2 * z1 + z2 * y1
    y = -w2 * y1 + x2 * z1 + y2 * w1 - z2 * x1
    z = -w2 * z1 - x2 * y1 + y2 * x1 + z2 * w1
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1e-12:
        return [0.0, 0.0, 0.0]
    return [x / length, y / length, z / length]


def compact_report(samples, namespace):
    report = {"namespace": namespace, "topics": {}, "tf_static": {}, "tf_dynamic": {}}
    for key in ("left_image", "right_image", "depth", "left_camera_info",
                "right_camera_info", "imu", "realsense_imu", "point_cloud"):
        values = samples.get(key, [])
        if not values:
            report["topics"][key] = {"received": False}
            continue
        report["topics"][key] = {
            "received": True,
            "sample_count": len(values),
            "first_stamp_ns": values[0]["stamp_ns"],
            "last_stamp_ns": values[-1]["stamp_ns"],
            "latest": values[-1],
        }

    stamp_sets = {
        key: {value["stamp_ns"] for value in samples.get(key, [])}
        for key in ("left_image", "right_image", "depth",
                    "left_camera_info", "right_camera_info")
    }
    report["camera_alignment"] = {
        "stereo_image_common_stamps": len(
            stamp_sets["left_image"] & stamp_sets["right_image"]),
        "left_image_info_common_stamps": len(
            stamp_sets["left_image"] & stamp_sets["left_camera_info"]),
        "right_image_info_common_stamps": len(
            stamp_sets["right_image"] & stamp_sets["right_camera_info"]),
        "stereo_all_common_stamps": len(
            stamp_sets["left_image"] & stamp_sets["right_image"]
            & stamp_sets["left_camera_info"] & stamp_sets["right_camera_info"]
        ),
        "left_depth_common_stamps": len(
            stamp_sets["left_image"] & stamp_sets["depth"]),
    }

    for transform in samples.get("tf_static", []):
        report["tf_static"][transform["child"]] = transform

    dynamic_by_child = defaultdict(list)
    for transform in samples.get("tf", []):
        dynamic_by_child[transform["child"]].append(transform)
    for child, transforms in dynamic_by_child.items():
        first, last = transforms[0], transforms[-1]
        translation_delta = math.sqrt(sum(
            (a - b) ** 2 for a, b in zip(first["translation"], last["translation"])
        ))
        report["tf_dynamic"][child] = {
            "sample_count": len(transforms),
            "parent": last["parent"],
            "first": first,
            "latest": last,
            "translation_delta_m": translation_delta,
            "rotation_delta_rad": quaternion_distance(first["rotation"], last["rotation"]),
            "rotation_axis_in_parent": quaternion_delta_axis(
                first["rotation"], last["rotation"]
            ),
        }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--expect-lidar", action="store_true")
    parser.add_argument("--expect-realsense-imu", action="store_true")
    parser.add_argument("--raw", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = AcceptanceCollector(args.namespace, args.expect_lidar, args.expect_realsense_imu)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = dict(node.samples) if args.raw else compact_report(node.samples, args.namespace)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
