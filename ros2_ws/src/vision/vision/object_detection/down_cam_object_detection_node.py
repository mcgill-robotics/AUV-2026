#!/usr/bin/env python3
from datetime import datetime
from functools import partial
import os
import threading
import time

from auv_msgs.srv import AutomaticCapture
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray

from vision.object_detection.utils import (
    build_detection2d_msg,
    get_detections,
    load_model,
    publish_annotated_image_util,
    toggle_collection_callback_util,
)


class DownCamObjectDetectorNode():
    """Down-camera object detection with direct USB capture (no usb_cam_node).

    Opens the camera via cv2.VideoCapture and runs inference in a tight
    threaded grab loop, mirroring the front-cam ZED pattern.
    """

    def __init__(self, node: Node):
        self.node = node

        # ── Model / detection parameters ─────────────────────────
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter('publish_annotated_every_n_frames', 1)
        self.node.declare_parameter("model_detection_threshold", 0.40)

        # ── Camera hardware parameters ───────────────────────────
        self.node.declare_parameter('video_device', '/dev/video0')
        self.node.declare_parameter('image_width', 640)
        self.node.declare_parameter('image_height', 480)
        self.node.declare_parameter('camera_fps', 30)
        self.node.declare_parameter('camera_frame_id', 'sensors/down_cam')

        # ── V4L2 image controls (HD USB Camera) ──────────────────
        self.node.declare_parameter('brightness', 8)
        self.node.declare_parameter('contrast', 8)
        self.node.declare_parameter('saturation', 7)
        self.node.declare_parameter('hue', 0)
        self.node.declare_parameter('sharpness', 6)
        self.node.declare_parameter('gamma', 7)
        self.node.declare_parameter('backlight_compensation', 0)
        self.node.declare_parameter('power_line_frequency', 2)
        self.node.declare_parameter('auto_white_balance', True)
        self.node.declare_parameter('white_balance_temperature', 2800)
        self.node.declare_parameter('auto_exposure', 3)
        self.node.declare_parameter('exposure_time_absolute', 625)
        self.node.declare_parameter('auto_focus', True)
        self.node.declare_parameter('focus_absolute', 16)

        # ── Collection parameters ────────────────────────────────
        self.node.declare_parameter('collection_dir', '/tmp/down_cam_collection')
        self.node.declare_parameter('collection_interval_seconds', 2.0)

        # ── Read parameters ──────────────────────────────────────
        self.class_names = list(
            self.node.get_parameter('class_names').get_parameter_value().string_array_value
        )
        self.node.get_logger().info(f"Class names: {self.class_names}")

        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = (
            self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        )
        self.publish_annotated_every_n_frames = (
            self.node.get_parameter('publish_annotated_every_n_frames').get_parameter_value().integer_value
        )
        self.conf_threshold = (
            self.node.get_parameter('model_detection_threshold').get_parameter_value().double_value
        )

        # Camera hardware
        self.video_device = self.node.get_parameter('video_device').get_parameter_value().string_value
        self.image_width = self.node.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.node.get_parameter('image_height').get_parameter_value().integer_value
        self.camera_fps = self.node.get_parameter('camera_fps').get_parameter_value().integer_value
        self.camera_frame_id = self.node.get_parameter('camera_frame_id').get_parameter_value().string_value

        # Collection
        self._collecting = False
        self._last_collection_time = 0.0
        self._frame_counter = 0
        self.collection_dir = self.node.get_parameter('collection_dir').get_parameter_value().string_value
        self.collection_interval = self.node.get_parameter('collection_interval_seconds').get_parameter_value().double_value

        self.node.create_service(
            AutomaticCapture,
            '~/toggle_collection',
            partial(toggle_collection_callback_util, self),
        )

        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        input_format = CompressedImage if self.compressed else Image

        self.bridge = CvBridge()

        # ── Load model ───────────────────────────────────────────
        if not os.path.exists(model_path):
            self.node.get_logger().fatal(f"Model path does not exist: {model_path}")
            raise FileNotFoundError(f"Model path does not exist: {model_path}")

        self.model = load_model(model_path, self.node.get_logger())

        # ── Publishers ───────────────────────────────────────────
        self.pub_detections = self.node.create_publisher(
            Detection2DArray,
            detection_topic,
            queue_size,
        )
        self.node.get_logger().info(f"Publishing detections to: {detection_topic}")

        if self.publish_annotated_image:
            publish_topic = detection_topic + "/annotated"
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size,
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")

        # ── Open camera & apply controls ──────────────────────────
        self.cap = self._open_camera()
        self._apply_camera_controls()

        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

        # ── Start grab loop ──────────────────────────────────────
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

    # ──────────────────────────────────────────────────────────────
    # Camera helpers
    # ──────────────────────────────────────────────────────────────

    # V4L2 control name → OpenCV CAP_PROP mapping
    # Controls with a direct OpenCV constant are applied via cap.set();
    # the rest are applied via v4l2-ctl subprocess.
    V4L2_OPENCV_CONTROLS = (
        ("brightness",               cv2.CAP_PROP_BRIGHTNESS),
        ("contrast",                 cv2.CAP_PROP_CONTRAST),
        ("saturation",               cv2.CAP_PROP_SATURATION),
        ("hue",                      cv2.CAP_PROP_HUE),
        ("sharpness",                cv2.CAP_PROP_SHARPNESS),
        ("gamma",                    cv2.CAP_PROP_GAMMA),
        ("auto_white_balance",       cv2.CAP_PROP_AUTO_WB),
        ("white_balance_temperature", cv2.CAP_PROP_WB_TEMPERATURE),
        ("auto_focus",               cv2.CAP_PROP_AUTOFOCUS),
        ("focus_absolute",           cv2.CAP_PROP_FOCUS),
        ("auto_exposure",            cv2.CAP_PROP_AUTO_EXPOSURE),
        ("exposure_time_absolute",   cv2.CAP_PROP_EXPOSURE),
    )

    def _open_camera(self) -> cv2.VideoCapture:
        """Open the USB camera with configured resolution/fps."""
        self.node.get_logger().info(
            f"Opening camera: {self.video_device} "
            f"({self.image_width}x{self.image_height} @ {self.camera_fps} fps)"
        )

        cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera: {self.video_device}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.node.get_logger().info(
            f"Camera opened: {actual_w}x{actual_h} @ {actual_fps:.1f} fps"
        )
        return cap

    def _apply_camera_controls(self):
        """Apply V4L2 image controls from ROS parameters to the open camera."""
        import subprocess

        applied = []

        for param_name, cv_prop in self.V4L2_OPENCV_CONTROLS:
            param = self.node.get_parameter(param_name)
            if param.type_ == Parameter.Type.BOOL:
                value = int(param.get_parameter_value().bool_value)
            else:
                value = param.get_parameter_value().integer_value

            # Skip manual white balance temperature when auto WB is on
            if param_name == "white_balance_temperature":
                if self.node.get_parameter('auto_white_balance').get_parameter_value().bool_value:
                    continue

            # Skip manual exposure when auto exposure is on (3 = Aperture Priority)
            if param_name == "exposure_time_absolute":
                if self.node.get_parameter('auto_exposure').get_parameter_value().integer_value == 3:
                    continue

            # Skip manual focus when autofocus is on
            if param_name == "focus_absolute":
                if self.node.get_parameter('auto_focus').get_parameter_value().bool_value:
                    continue

            ok = self.cap.set(cv_prop, value)
            if not ok:
                self.node.get_logger().warn(f"Failed to set {param_name}={value} via OpenCV")
            else:
                applied.append(f"{param_name}={value}")

        # Controls without OpenCV constants — apply via v4l2-ctl
        v4l2_only_controls = [
            ("power_line_frequency", self.node.get_parameter('power_line_frequency').get_parameter_value().integer_value),
            ("backlight_compensation", self.node.get_parameter('backlight_compensation').get_parameter_value().integer_value),
        ]
        for ctrl_name, ctrl_value in v4l2_only_controls:
            try:
                subprocess.run(
                    ["v4l2-ctl", "-d", self.video_device,
                     "--set-ctrl", f"{ctrl_name}={ctrl_value}"],
                    check=True, capture_output=True, timeout=5,
                )
                applied.append(f"{ctrl_name}={ctrl_value}")
            except Exception as e:
                self.node.get_logger().warn(f"v4l2-ctl {ctrl_name} failed: {e}")

        self.node.get_logger().info(f"Applied camera controls: {', '.join(applied)}")

    # ──────────────────────────────────────────────────────────────
    # Main grab loop (runs on dedicated thread)
    # ──────────────────────────────────────────────────────────────

    def _grab_loop(self):
        """Tight grab loop — runs on its own thread. read() blocks at camera fps."""
        while rclpy.ok():
            ret, img = self.cap.read()
            if not ret:
                self.node.get_logger().warn(
                    "Failed to read frame from camera", throttle_duration_sec=5.0
                )
                continue

            self._frame_counter += 1
            frame_stamp = self.node.get_clock().now()

            # ── Dataset collection ────────────────────────────
            if self._collecting:
                now = time.time()
                if now - self._last_collection_time >= self.collection_interval:
                    self._last_collection_time = now
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    filepath = os.path.join(self.collection_dir, f'down_{timestamp}.jpg')
                    cv2.imwrite(filepath, img)
                    self.node.get_logger().debug(f"Saved {filepath}")

            # ── Inference ─────────────────────────────────────
            tracked_detections = get_detections(self, img)
            
            det_msg = build_detection2d_msg(self, tracked_detections)
            det_msg.header.stamp = frame_stamp.to_msg()
            det_msg.header.frame_id = self.camera_frame_id
            self.pub_detections.publish(det_msg)

            if (
                self.publish_annotated_image
                and self._frame_counter % self.publish_annotated_every_n_frames == 0
            ):
                publish_annotated_image_util(
                    self,
                    img,
                    tracked_detections,
                    frame_stamp.to_msg(),
                    self.camera_frame_id,
                )

    def destroy(self):
        """Release camera resources."""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
