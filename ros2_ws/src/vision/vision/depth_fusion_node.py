#!/usr/bin/env python3
"""ROS2 node that fuses YOLO detections with ZED depth frames."""

from __future__ import annotations

import math
from collections import deque
from threading import Lock
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray
from auv_msgs.msg import VisionObject, VisionObjectArray
import yaml

from vision.depth_fusion import BoundingBox, configure_depth_fusion, estimate_depth_for_detection


class DetectionDepthFusionNode(Node):
    """Depth fusion front-end that wraps the stateless estimator."""

    def __init__(self) -> None:
        super().__init__("depth_fusion")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("depth_topic", "/zed2i/zed_node/depth/image_rect"),
                ("camera_info_topic", "/zed2i/zed_node/depth/camera_info"),
                ("detection_topic", "/vision/front_cam/detections"),
                ("output_topic", "/vision/front_cam/depth_fusion"),
                ("config_file", ""),
                ("debug", False),
                ("sync_slop", 0.15),
            ],
        )

        self.bridge = CvBridge()
        self.latest_camera_info: Optional[CameraInfo] = None
        self.debug_enabled = self.get_parameter("debug").get_parameter_value().bool_value
        self.sync_slop = float(
            self.get_parameter("sync_slop").get_parameter_value().double_value
        )
        self.sync_slop_ns = int(self.sync_slop * 1e9) if self.sync_slop > 0 else 0

        self._load_config()

        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        det_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.get_logger().info(
            f"Depth fusion subscribing to depth={depth_topic}, detections={det_topic}, info={info_topic}"
        )

        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        det_qos = QoSProfile(depth=5)

        self.depth_lock = Lock()
        self.depth_buffer: deque[Image] = deque(maxlen=10)
        self._no_depth_warning_emitted = False

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            depth_qos,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            det_topic,
            self._detection_callback,
            det_qos,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self._camera_info_callback,
            depth_qos,
        )

        self.publisher = self.create_publisher(VisionObjectArray, output_topic, 10)
        self.get_logger().info(f"Publishing fused results on {output_topic}")

    def _load_config(self) -> None:
        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_path:
            configure_depth_fusion({})
            self.get_logger().info("No depth fusion config specified; using defaults")
            return
        path = Path(config_path)
        if not path.is_file():
            self.get_logger().warn(f"Config file {config_path} missing, using defaults")
            configure_depth_fusion({})
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            configure_depth_fusion(data)
            self.get_logger().info(f"Loaded depth fusion config from {path}")
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to load config {path}: {exc}")
            configure_depth_fusion({})

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _depth_callback(self, msg: Image) -> None:
        with self.depth_lock:
            self.depth_buffer.append(msg)
        if self._no_depth_warning_emitted:
            self._no_depth_warning_emitted = False

    def _detection_callback(self, detections_msg: Detection2DArray) -> None:
        depth_msg = self._match_depth_frame(detections_msg)
        if depth_msg is None:
            if not self._no_depth_warning_emitted:
                self.get_logger().warn(
                    "No synchronized depth frame available for detection timestamp"
                )
                self._no_depth_warning_emitted = True
            return
        self._synced_callback(depth_msg, detections_msg)

    def _match_depth_frame(self, detections_msg: Detection2DArray) -> Optional[Image]:
        with self.depth_lock:
            if not self.depth_buffer:
                return None
            target_stamp = detections_msg.header.stamp
            if target_stamp.sec == 0 and target_stamp.nanosec == 0:
                return self.depth_buffer[-1]
            target_time = Time.from_msg(target_stamp)
            best_msg: Optional[Image] = None
            best_delta: Optional[int] = None
            for depth_msg in self.depth_buffer:
                depth_stamp = depth_msg.header.stamp
                if depth_stamp.sec == 0 and depth_stamp.nanosec == 0:
                    continue
                delta = abs((Time.from_msg(depth_stamp) - target_time).nanoseconds)
                if best_delta is None or delta < best_delta:
                    best_delta = delta
                    best_msg = depth_msg
            if best_msg is None:
                return None
            if self.sync_slop_ns and best_delta is not None and best_delta > self.sync_slop_ns:
                return None
            return best_msg

    def _synced_callback(self, depth_msg: Image, detections_msg: Detection2DArray) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed for depth: {exc}")
            return

        if depth.dtype != np.float32 and depth.dtype != np.float64:
            depth = depth.astype(np.float32)
            if depth_msg.encoding.upper() == "16UC1":
                depth *= 0.001

        fused = VisionObjectArray()
        fused.array = []

        for detection in detections_msg.detections:
            if not detection.results:
                continue
            label, score = self._extract_label_and_score(detection)
            bbox = BoundingBox(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.size_x,
                detection.bbox.size_y,
            )
            depth_est, confidence, debug_info, is_valid = estimate_depth_for_detection(
                bbox, label, depth
            )

            if not is_valid:
                if self.debug_enabled:
                    reason = debug_info.get("failure_reason", "unknown")
                    self.get_logger().debug(
                        f"Rejected {label} bbox ({reason}); baseline={debug_info.get('baseline_depth')}"
                    )
                continue

            point = self._pixel_to_point(bbox.center_x, bbox.center_y, depth_est)
            obj = VisionObject()
            obj.label = str(label)
            obj.x = point[0]
            obj.y = point[1]
            obj.z = depth_est
            obj.theta_z = 0.0
            obj.extra_field = float(debug_info.get("baseline_depth", math.nan))
            obj.confidence = min(max(confidence * score, 0.0), 1.0)
            fused.array.append(obj)

            if self.debug_enabled:
                self.get_logger().info(
                    f"{label}: depth={depth_est:.2f}m conf={obj.confidence:.2f} "
                    f"samples={debug_info.get('valid_samples')} cluster={debug_info.get('cluster_size')}"
                )

        self.publisher.publish(fused)

    def _pixel_to_point(self, u: float, v: float, depth: float) -> Tuple[float, float, float]:
        info = self.latest_camera_info
        if info is None or depth != depth or depth <= 0.0:
            return (float("nan"), float("nan"), depth)
        fx = info.k[0]
        fy = info.k[4]
        cx = info.k[2]
        cy = info.k[5]
        if fx == 0 or fy == 0:
            return (float("nan"), float("nan"), depth)
        x = (u - cx) / fx * depth
        y = (v - cy) / fy * depth
        return (float(x), float(y), float(depth))

    def _extract_label_and_score(self, detection: Detection2D) -> Tuple[str, float]:
        best = detection.results[0]
        best_score = best.hypothesis.score
        best_label = best.hypothesis.class_id or str(best.id)
        for result in detection.results[1:]:
            if result.hypothesis.score > best_score:
                best = result
                best_score = result.hypothesis.score
                best_label = result.hypothesis.class_id or str(result.id)
        return str(best_label), float(best_score)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DetectionDepthFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
