#!/usr/bin/env python3
from datetime import datetime
from functools import partial
import os
import subprocess
import threading
import time

from auv_msgs.srv import AutomaticCapture
from auv_msgs.msg import VisionDetection, VisionDetectionFrame
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
import copy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from auv_msgs.srv import SetDownCamProjectionHeights
import math
from tf_transformations import quaternion_matrix, quaternion_from_euler, quaternion_multiply
from ament_index_python.packages import get_package_share_directory

from vision.object_detection.utils import (
    get_detections,
    load_model,
    publish_annotated_image_util,
    toggle_collection_callback_util,
    get_vector3_parameter,
    build_transform_matrix_from_xyz_rpy,
    apply_snells_law_lateral,
)


class DownCamObjectDetectorNode():
    """Down-camera object detection with direct USB capture (no usb_cam_node).

    Opens the camera via cv2.VideoCapture and runs inference in a tight
    threaded grab loop, mirroring the front-cam ZED pattern.
    """

    # V4L2 control name --> OpenCV CAP_PROP mapping
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
    V4L2_ONLY_CONTROLS = (
        "power_line_frequency",
        "backlight_compensation",
    )

    def __init__(self, node: Node):
        self.node = node

        # ── Model / detection parameters ─────────────────────────
        self.node.declare_parameter('model_class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_relative_path_override', Parameter.Type.STRING)
        self.node.declare_parameter('model_relative_path.sim', Parameter.Type.STRING)
        self.node.declare_parameter('model_relative_path.real', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated', Parameter.Type.BOOL)
        self.node.declare_parameter('publish_annotated_every_n_frames', Parameter.Type.INTEGER)
        self.node.declare_parameter("model_detection_threshold", Parameter.Type.DOUBLE)
        self.node.declare_parameter('compressed', Parameter.Type.BOOL)
        self.node.declare_parameter("enable_object_detection", Parameter.Type.BOOL)
        self.node.declare_parameter('sim', Parameter.Type.BOOL)
        self.node.declare_parameter('image_topic', Parameter.Type.STRING)

        # ── Camera hardware parameters ───────────────────────────
        self.node.declare_parameter('video_device', Parameter.Type.STRING)
        self.node.declare_parameter('width', Parameter.Type.INTEGER)
        self.node.declare_parameter('height', Parameter.Type.INTEGER)
        self.node.declare_parameter('fps', Parameter.Type.INTEGER)
        self.node.declare_parameter('camera_frame_id', Parameter.Type.STRING)

        # ── Camera intrinsics parameters ─────────────────────────
        self.node.declare_parameter('base_frame_id', Parameter.Type.STRING)
        self.node.declare_parameter('fx', Parameter.Type.DOUBLE)
        self.node.declare_parameter('fy', Parameter.Type.DOUBLE)
        self.node.declare_parameter('cx', Parameter.Type.DOUBLE)
        self.node.declare_parameter('cy', Parameter.Type.DOUBLE)
        self.node.declare_parameter('camera_info_topic', Parameter.Type.STRING)

        # ── Extrinsics & 3D Projection parameters ────────────────
        self.node.declare_parameter('auv_to_down_cam.xyz', Parameter.Type.DOUBLE_ARRAY)
        self.node.declare_parameter('auv_to_down_cam.rpy', Parameter.Type.DOUBLE_ARRAY)
        self.node.declare_parameter('down_cam_projection_labels', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('down_cam_projection_heights', Parameter.Type.DOUBLE_ARRAY)
        self.node.declare_parameter('pool_floor_z', Parameter.Type.DOUBLE)
        self.node.declare_parameter('depth_scale.sim', Parameter.Type.DOUBLE)
        self.node.declare_parameter('depth_scale.real', Parameter.Type.DOUBLE)
        self.node.declare_parameter('lateral_scale.sim', Parameter.Type.DOUBLE)
        self.node.declare_parameter('lateral_scale.real', Parameter.Type.DOUBLE)
        self.node.declare_parameter('pose_source', Parameter.Type.STRING)
        self.node.declare_parameter('auv_pose_topic', Parameter.Type.STRING)
        self.node.declare_parameter('border_margin', Parameter.Type.INTEGER)

        # ── V4L2 image controls (HD USB Camera) ──────────────────
        self.node.declare_parameter('brightness', Parameter.Type.INTEGER)
        self.node.declare_parameter('contrast', Parameter.Type.INTEGER)
        self.node.declare_parameter('saturation', Parameter.Type.INTEGER)
        self.node.declare_parameter('hue', Parameter.Type.INTEGER)
        self.node.declare_parameter('sharpness', Parameter.Type.INTEGER)
        self.node.declare_parameter('gamma', Parameter.Type.INTEGER)
        self.node.declare_parameter('backlight_compensation', Parameter.Type.INTEGER)
        self.node.declare_parameter('power_line_frequency', Parameter.Type.INTEGER)
        self.node.declare_parameter('auto_white_balance', Parameter.Type.BOOL)
        self.node.declare_parameter('white_balance_temperature', Parameter.Type.INTEGER)
        self.node.declare_parameter('auto_exposure', Parameter.Type.INTEGER)
        self.node.declare_parameter('exposure_time_absolute', Parameter.Type.INTEGER)
        self.node.declare_parameter('auto_focus', Parameter.Type.BOOL)
        self.node.declare_parameter('focus_absolute', Parameter.Type.INTEGER)

        # ── Collection parameters ────────────────────────────────
        self.node.declare_parameter('collection_dir', Parameter.Type.STRING)
        self.node.declare_parameter('collection_interval_seconds', Parameter.Type.DOUBLE)
        self.node.declare_parameter('collection_file_extension', Parameter.Type.STRING)

        # ── Read parameters ──────────────────────────────────────
        self.class_names = list(
            self.node.get_parameter('model_class_names').get_parameter_value().string_array_value
        )
        self.node.get_logger().info(f"Class names: {self.class_names}")

        self.sim = self.node.get_parameter('sim').get_parameter_value().bool_value
        rel_path = self.node.get_parameter('model_relative_path_override').get_parameter_value().string_value
        if not rel_path:
            rel_path = self.node.get_parameter('model_relative_path.sim' if self.sim else 'model_relative_path.real').get_parameter_value().string_value
        model_path = os.path.join(get_package_share_directory('vision'), rel_path)
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = (
            self.node.get_parameter('publish_annotated').get_parameter_value().bool_value
        )
        self.annotated_image_enabled = self.publish_annotated_image
        self.publish_annotated_every_n_frames = (
            self.node.get_parameter('publish_annotated_every_n_frames').get_parameter_value().integer_value
        )
        self.conf_threshold = (
            self.node.get_parameter('model_detection_threshold').get_parameter_value().double_value
        )
        self.enable_object_detection = (
            self.node.get_parameter('enable_object_detection').get_parameter_value().bool_value
        )
        self.image_topic = self.node.get_parameter('image_topic').get_parameter_value().string_value
        
        depth_scale_param = 'depth_scale.sim' if self.sim else 'depth_scale.real'
        self.depth_scale = self.node.get_parameter(depth_scale_param).get_parameter_value().double_value
        lateral_scale_param = 'lateral_scale.sim' if self.sim else 'lateral_scale.real'
        self.lateral_scale = self.node.get_parameter(lateral_scale_param).get_parameter_value().double_value
        
        self.border_margin = self.node.get_parameter('border_margin').get_parameter_value().integer_value

        # Camera hardware
        self.video_device = self.node.get_parameter('video_device').get_parameter_value().string_value
        self.image_width = self.node.get_parameter('width').get_parameter_value().integer_value
        self.image_height = self.node.get_parameter('height').get_parameter_value().integer_value
        self.camera_fps = self.node.get_parameter('fps').get_parameter_value().integer_value
        self.camera_frame_id = self.node.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.node.get_parameter('base_frame_id').get_parameter_value().string_value

        # Intrinsics
        self.fx = self.node.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.node.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.node.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.node.get_parameter('cy').get_parameter_value().double_value
        self.camera_info_topic = self.node.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Extrinsics & 3D Projection
        self.auv_to_down_cam_xyz = get_vector3_parameter(self.node, 'auv_to_down_cam.xyz')
        self.auv_to_down_cam_rpy = get_vector3_parameter(self.node, 'auv_to_down_cam.rpy')
        self.T_auv_down_cam = build_transform_matrix_from_xyz_rpy(self.auv_to_down_cam_xyz, self.auv_to_down_cam_rpy)
        
        self.down_cam_projection_labels = list(self.node.get_parameter('down_cam_projection_labels').get_parameter_value().string_array_value)
        self.down_cam_projection_heights = list(self.node.get_parameter('down_cam_projection_heights').get_parameter_value().double_array_value)
        self.down_cam_heights_map = dict(zip(self.down_cam_projection_labels, self.down_cam_projection_heights))
        self.pool_floor_z = self.node.get_parameter('pool_floor_z').get_parameter_value().double_value
        self.pose_source = self.node.get_parameter('pose_source').get_parameter_value().string_value
        self.auv_pose_topic = self.node.get_parameter('auv_pose_topic').get_parameter_value().string_value

        # Pose Subscriber
        self._auv_pose_lock = threading.Lock()
        self._auv_pose_msg = None
        
        if self.pose_source == "auv_pose":
            self.auv_pose_sub = self.node.create_subscription(
                PoseStamped,
                self.auv_pose_topic,
                self._auv_pose_callback,
                qos_profile_sensor_data
            )
            self.node.get_logger().info(f"Subscribed to AUV pose on: {self.auv_pose_topic} with sensor data QoS")

        # Collection
        self._collecting = False
        self._last_collection_time = 0.0
        self._frame_counter = 0
        self.collection_dir = self.node.get_parameter('collection_dir').get_parameter_value().string_value
        self.collection_interval = self.node.get_parameter('collection_interval_seconds').get_parameter_value().double_value
        self.collection_extension = self.node.get_parameter('collection_file_extension').get_parameter_value().string_value

        self.node.create_service(
            AutomaticCapture,
            '/vision/down_cam/toggle_collection',
            partial(toggle_collection_callback_util, self),
        )
        self.node.create_service(
            Trigger,
            '/vision/down_cam/toggle_annotated_image',
            self._toggle_annotated_image_callback,
        )
        self.node.create_service(
            Trigger,
            '/vision/down_cam/get_camera_settings',
            self._get_camera_settings_callback,
        )
        self.node.create_service(
            SetDownCamProjectionHeights,
            '/vision/down_cam/set_projection_heights',
            self._set_projection_heights_callback
        )

        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        input_format = CompressedImage if self.compressed else Image

        self.bridge = CvBridge()

        # ── Load model ───────────────────────────────────────────
        if self.enable_object_detection and not os.path.exists(model_path):
            self.node.get_logger().fatal(f"Model path does not exist: {model_path}")
            raise FileNotFoundError(f"Model path does not exist: {model_path}")

        if self.enable_object_detection:
            self.model = load_model(model_path, self.node.get_logger())
        else:
            self.model = None
            self.node.get_logger().info("Object detection is disabled. Publishing raw feed only.")

        # ── Publishers ───────────────────────────────────────────
        self.pub_detections = self.node.create_publisher(
            VisionDetectionFrame,
            detection_topic,
            queue_size,
        )
        self.node.get_logger().info(f"Publishing detections to: {detection_topic}")

        self.pub_camera_info = self.node.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            queue_size,
        )
        self.node.get_logger().info(f"Publishing camera info to: {self.camera_info_topic}")
        self.camera_info_template = self._build_camera_info_template()

        if self.publish_annotated_image:
            publish_topic = detection_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size,
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")

        # ── Open camera & apply controls ──────────────────────────
        if not self.sim:
            self.cap = self._open_camera()
            self._apply_camera_controls()

            # ── Start grab loop ──────────────────────────────────────
            self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
            self._grab_thread.start()
        else:
            self.node.get_logger().info(f"Simulation mode: Subscribing to {self.image_topic}")
            self.image_sub = self.node.create_subscription(
                input_format,
                self.image_topic,
                self._image_callback,
                queue_size
            )

        self.node.get_logger().info(f"{self.node.get_name()} initialized.")
        self.node.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, params):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return SetParametersResult(successful=True)
            
        success = True
        
        cv_param_map = {name: prop for name, prop in self.V4L2_OPENCV_CONTROLS}
        
        for param in params:
            if param.name in cv_param_map:
                cv_prop = cv_param_map[param.name]
                if param.type_ == Parameter.Type.BOOL:
                    val = int(param.value)
                else:
                    val = param.value
                    
                ok = self.cap.set(cv_prop, val)
                if not ok:
                    success = False
                    self.node.get_logger().warn(f"Failed to dynamically set {param.name}={val}")
                else:
                    self.node.get_logger().info(f"Dynamically updated {param.name}={val}")
            
            elif param.name in self.V4L2_ONLY_CONTROLS:
                try:
                    subprocess.run(
                        ["v4l2-ctl", "-d", self.video_device, "--set-ctrl", f"{param.name}={param.value}"],
                        check=True, capture_output=True, timeout=5,
                    )
                    self.node.get_logger().info(f"Dynamically updated v4l2 {param.name}={param.value}")
                except Exception as e:
                    self.node.get_logger().warn(f"v4l2-ctl {param.name} failed: {e}")
                    success = False
                    
        return SetParametersResult(successful=success)

    # ──────────────────────────────────────────────────────────────
    # Camera helpers
    # ──────────────────────────────────────────────────────────────

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
        for ctrl_name in self.V4L2_ONLY_CONTROLS:
            ctrl_value = self.node.get_parameter(ctrl_name).get_parameter_value().integer_value
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

            frame_stamp = self.node.get_clock().now()
            self._process_frame(img, frame_stamp)

    def _image_callback(self, msg):
        try:
            if self.compressed:
                img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge error: {e}", throttle_duration_sec=5.0)
            return
            
        frame_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self._process_frame(img, frame_stamp)

    def _toggle_annotated_image_callback(self, request, response):
        del request
        self.annotated_image_enabled = not self.annotated_image_enabled
        response.success = True
        response.message = (
            f"Annotated image {'enabled' if self.annotated_image_enabled else 'disabled'}"
        )
        self.node.get_logger().info(response.message)
        return response

    def _get_camera_settings_callback(self, request, response):
        del request
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            response.success = False
            response.message = "Camera not opened."
            return response
            
        current_settings = {}
        for param_name, cv_prop in self.V4L2_OPENCV_CONTROLS:
            val = self.cap.get(cv_prop)
            current_settings[param_name] = val
            
        for ctrl_name in self.V4L2_ONLY_CONTROLS:
            try:
                res = subprocess.run(
                    ["v4l2-ctl", "-d", self.video_device, "--get-ctrl", ctrl_name],
                    check=True, capture_output=True, text=True, timeout=5,
                )
                output = res.stdout.strip()
                if ":" in output:
                    current_settings[ctrl_name] = output.split(":")[-1].strip()
                else:
                    current_settings[ctrl_name] = output
            except Exception as e:
                current_settings[ctrl_name] = f"ERROR {e}"
                
        response.success = True
        response.message = " | ".join(f"{k}: {v}" for k, v in current_settings.items())
        return response

    def _set_projection_heights_callback(self, request, response):
        projection_height = request.projection_height
        response.success = True

        if request.all_labels:
            for label in self.down_cam_heights_map.keys():
                self.down_cam_heights_map[label] = projection_height
        else:
            if request.specific_label in self.down_cam_heights_map:
                self.down_cam_heights_map[request.specific_label] = projection_height
            else:
                response.success = False
                response.message = f"Label '{request.specific_label}' not found in down_cam_projection_labels."
        
        del request
        return response

    def _auv_pose_callback(self, msg: PoseStamped):
        with self._auv_pose_lock:
            self._auv_pose_msg = msg

    def _compute_3d_position(self, label: str, u: float, v: float, auv_pose_msg: PoseStamped):
        # Determine target Z
        if label not in self.down_cam_heights_map:
            self.node.get_logger().warn(f"Label '{label}' not in down_cam_projection_labels, skipping 3D projection.", throttle_duration_sec=2.0)
            return None
        target_z = self.pool_floor_z + self.down_cam_heights_map[label]
        
        # Precompute T_W_C
        pose = auv_pose_msg.pose
        T_W_auv = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T_W_auv[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        T_W_C = np.dot(T_W_auv, self.T_auv_down_cam)
        
        # Ray generation in Standard ROS Frame (X forward, Y left, Z up).
        # For the down camera, X (forward in camera frame) points DOWN in the world.
        # Apply Snell's Law to get true ray vector components at depth X=1
        x_ray, y_ray = apply_snells_law_lateral(1.0, u - self.cx, v - self.cy, self.fx, self.fy, n_w=self.lateral_scale)
        
        v_c_ros = np.array([1.0, -x_ray, -y_ray, 0.0])
        v_w_ros = np.dot(T_W_C, v_c_ros)
        
        P0_w = np.dot(T_W_C, np.array([0.0, 0.0, 0.0, 1.0]))
        
        if abs(v_w_ros[2]) < 1e-6:
            self.node.get_logger().warn(f"Ray is horizontal for {label}, cannot intersect floor!", throttle_duration_sec=2.0)
            return None
            
        t = (target_z - P0_w[2]) / v_w_ros[2]
        if t <= 0: # Intersection must be in front of the camera
            self.node.get_logger().warn(f"Intersection behind camera for {label}! t={t}", throttle_duration_sec=2.0)
            return None
            
        P_c = t * v_c_ros[:3]
        
        return P_c[0], P_c[1], P_c[2]

    def _process_frame(self, img, frame_stamp):
        self._frame_counter += 1

        # ── Dataset collection ────────────────────────────
        if self._collecting:
            now = time.time()
            if now - self._last_collection_time >= self.collection_interval:
                self._last_collection_time = now
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filepath = os.path.join(self.collection_dir, f'down_{timestamp}{self.collection_extension}')
                cv2.imwrite(filepath, img)
                self.node.get_logger().debug(f"Saved {filepath}")

        # ── Inference ─────────────────────────────────────
        if self.enable_object_detection:
            tracked_detections = get_detections(self, img)
        else:
            tracked_detections = None
        
        with self._auv_pose_lock:
            auv_pose_msg = self._auv_pose_msg

        if auv_pose_msg is None:
            self.node.get_logger().warn("auv_pose_msg is None! Cannot compute 3D positions.", throttle_duration_sec=2.0)
            
        det_frame = VisionDetectionFrame()
        det_frame.header.stamp = frame_stamp.to_msg()
        det_frame.header.frame_id = self.base_frame_id
        if auv_pose_msg:
            det_frame.auv_pose = auv_pose_msg
        
        if tracked_detections is not None:
            obb_list = []
            for i in range(len(tracked_detections)):
                x1, y1, x2, y2 = tracked_detections.xyxy[i]
                w = float(x2 - x1)
                h = float(y2 - y1)
                
                # Default to bounding box center and no orientation
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)
                yaw = None
                
                # Default unrotated box
                box = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
                
                # If a segmentation mask is available, use its center of mass for precision
                _yaw_reason = "no mask"
                if tracked_detections.mask is not None:
                    mask = tracked_detections.mask[i].astype(np.uint8)
                    M = cv2.moments(mask)
                    if M["m00"] != 0:
                        cx = float(M["m10"] / M["m00"])
                        cy = float(M["m01"] / M["m00"])
                        
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours) > 0:
                        c = max(contours, key=cv2.contourArea)
                        _yaw_reason = f"contour only {len(c)} pts (<5)"
                        if len(c) >= 5:
                            rect = cv2.minAreaRect(c)
                            box = cv2.boxPoints(rect)
                            
                            # Check if bounding box touches the image border
                            height, width = img.shape[:2]
                            margin = self.border_margin
                            touches_border = (x1 <= margin or y1 <= margin or x2 >= width - margin or y2 >= height - margin)
                            _yaw_reason = f"touches_border (margin={margin}, x1={x1:.0f},y1={y1:.0f},x2={x2:.0f},y2={y2:.0f} vs {width}x{height})"
                            
                            if not touches_border:
                                p0, p1, p2 = box[0], box[1], box[2]
                                dist1 = np.linalg.norm(p1 - p0)
                                dist2 = np.linalg.norm(p2 - p1)
                                
                                if dist1 > dist2:
                                    dx = p1[0] - p0[0]
                                    dy = p1[1] - p0[1]
                                else:
                                    dx = p2[0] - p1[0]
                                    dy = p2[1] - p1[1]
                                    
                                yaw = math.atan2(dy, dx)
                                _yaw_reason = "ok"
                    else:
                        _yaw_reason = "no contours found"
                
                obb_list.append(box)

                conf = float(tracked_detections.confidence[i])
                cls_id = int(tracked_detections.class_id[i])

                if cls_id >= len(self.class_names):
                    continue
                label = self.class_names[cls_id]
                
                det = VisionDetection()
                det.label = label
                det.confidence = conf
                
                # Bounding box
                det.bbox_center_x = cx
                det.bbox_center_y = cy
                det.bbox_size_x = w
                det.bbox_size_y = h
                
                if auv_pose_msg is not None:
                    pos_3d = self._compute_3d_position(label, cx, cy, auv_pose_msg)
                    if pos_3d is not None:
                        det.pose_camera.pose.position.x = float(pos_3d[0])
                        det.pose_camera.pose.position.y = float(pos_3d[1])
                        det.pose_camera.pose.position.z = float(pos_3d[2])
                    else:
                        det.pose_camera.pose.position.x = float('nan')
                        det.pose_camera.pose.position.y = float('nan')
                        det.pose_camera.pose.position.z = float('nan')
                else:
                    det.pose_camera.pose.position.x = float('nan')
                    det.pose_camera.pose.position.y = float('nan')
                    det.pose_camera.pose.position.z = float('nan')
                        

                if yaw is not None:
                    # The object lies flat on the pool floor.
                    # A flat object facing forward in the world has an orientation of Ry(-90) in the down_camera frame.
                    # A rotation in the image plane corresponds to a rotation around the camera's X-axis.
                    alpha = math.pi/2 + yaw
                    q_rot = quaternion_from_euler(alpha, 0, 0)
                    q_base = quaternion_from_euler(0, -math.pi/2, 0)
                    q = quaternion_multiply(q_rot, q_base)
                    det.pose_camera.pose.orientation.x = float(q[0])
                    det.pose_camera.pose.orientation.y = float(q[1])
                    det.pose_camera.pose.orientation.z = float(q[2])
                    det.pose_camera.pose.orientation.w = float(q[3])
                else:
                    # Invalid quaternion signals the tracker to ignore this orientation update
                    det.pose_camera.pose.orientation.x = 0.0
                    det.pose_camera.pose.orientation.y = 0.0
                    det.pose_camera.pose.orientation.z = 0.0
                    det.pose_camera.pose.orientation.w = 0.0

                det_frame.detections.append(det)

            tracked_detections.data["xyxyxyxy"] = np.array(obb_list)

        self.pub_detections.publish(det_frame)

        # ── Publish CameraInfo ───────────────────────────
        if self.pub_camera_info is not None:
            camera_info_msg = copy.deepcopy(self.camera_info_template)
            camera_info_msg.header.stamp = frame_stamp.to_msg()
            self.pub_camera_info.publish(camera_info_msg)

        if (
            self.annotated_image_enabled
            and self._frame_counter % self.publish_annotated_every_n_frames == 0
        ):
            publish_annotated_image_util(
                self,
                img,
                tracked_detections,
                frame_stamp.to_msg(),
                self.camera_frame_id,
            )

    def _build_camera_info_template(self) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id

        camera_info.width = self.image_width
        camera_info.height = self.image_height
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return camera_info

    def destroy(self):
        """Release camera resources."""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
