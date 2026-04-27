#!/usr/bin/env python3
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

import os
import cv2
import time
import copy
import threading
from time import sleep
from datetime import datetime
from pathlib import Path
import numpy as np

from tf_transformations import concatenate_matrices, euler_matrix, inverse_matrix, quaternion_from_matrix
from vision.object_detection.utils import load_model, get_detections, publish_annotated_image_util

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, Quaternion
from auv_msgs.msg import VisionDetection, VisionDetectionFrame

class FrontCamObjectDetectorNode():
    POSE_SOURCE_ZED_VIO = "zed_vio"
    POSE_SOURCE_AUV_POSE = "auv_pose"

    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter("detection_frame_topic", Parameter.Type.STRING)

        self.node.declare_parameter("model_detection_threshold", 0.40)
        self.node.declare_parameter("depth_confidence_threshold", 95)

        # we consume stream_ip and sim properties for zed sdk configuration
        self.node.declare_parameter('sim', Parameter.Type.BOOL)
        self.node.declare_parameter('stream_ip', Parameter.Type.STRING)
        self.node.declare_parameter('stream_port', Parameter.Type.INTEGER)
        self.node.declare_parameter('vio_pose_topic', '/vision/vio_pose')
        self.node.declare_parameter('pose_source', self.POSE_SOURCE_ZED_VIO)
        self.node.declare_parameter('auv_pose_topic', 'state/pose')
        self.node.declare_parameter('enable_gate_top_crop', True)
        self.node.declare_parameter('gate_top_crop_ratio', 0.50)
        self.node.declare_parameter('enable_border_exclusion', True)
        self.node.declare_parameter('border_exclusion_margin_px', 20)
        self.node.declare_parameter('border_exclusion_labels', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('global_frame_id', 'pool_link')
        self.node.declare_parameter('auv_frame_id', 'auv_link')
        self.node.declare_parameter('detection_frame_id', 'zed_left_camera_frame')
        self.node.declare_parameter('auv_to_camera_center_xyz', [0.0, 0.0, 0.0])
        self.node.declare_parameter('auv_to_camera_center_rpy', [0.0, 0.0, 0.0])
        self.node.declare_parameter('camera_center_to_detection_xyz', [0.0, 0.06, 0.0])
        self.node.declare_parameter('camera_center_to_detection_rpy', [0.0, 0.0, 0.0])

        # Image collection parameters
        self.node.declare_parameter('collection_dir', '/tmp/front_cam_collection')
        self.node.declare_parameter('collection_interval_seconds', 2.0)

        # Depth sensor state (pressure sensor overrides VIO Z)
        self._depth_lock = threading.Lock()
        self._sensor_depth = 0.0
        self._auv_pose_lock = threading.Lock()
        self._auv_pose_msg = None
        self._has_auv_pose = False
        self.depth_sub = self.node.create_subscription(
            Float64,
            '/sensors/depth/z',
            self._depth_callback,
            qos_profile_sensor_data
        )

        # Image collection state
        self._collecting = False
        self._last_collection_time = 0.0
        self.collection_dir = self.node.get_parameter('collection_dir').get_parameter_value().string_value
        self.collection_interval = self.node.get_parameter('collection_interval_seconds').get_parameter_value().double_value

        # Service to toggle image collection on/off
        self.node.create_service(SetBool, '/vision/front_cam/toggle_collection', self._toggle_collection_callback)
 
        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        self.detection_frame_topic = (
            self.node.get_parameter('detection_frame_topic').get_parameter_value().string_value
        )
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value

        self.conf_threshold = self.node.get_parameter('model_detection_threshold').get_parameter_value().double_value
        self.depth_confidence_threshold = (
            self.node.get_parameter('depth_confidence_threshold').get_parameter_value().integer_value
        )
        sim = self.node.get_parameter('sim').get_parameter_value().bool_value
        stream_port = self.node.get_parameter('stream_port').get_parameter_value().integer_value
        self.pose_source = self.node.get_parameter('pose_source').get_parameter_value().string_value
        self.auv_pose_topic = self.node.get_parameter('auv_pose_topic').get_parameter_value().string_value
        self.enable_gate_top_crop = self.node.get_parameter('enable_gate_top_crop').get_parameter_value().bool_value
        self.gate_top_crop_ratio = self.node.get_parameter('gate_top_crop_ratio').get_parameter_value().double_value
        self.enable_border_exclusion = self.node.get_parameter('enable_border_exclusion').get_parameter_value().bool_value
        self.border_exclusion_margin_px = self.node.get_parameter('border_exclusion_margin_px').get_parameter_value().integer_value
        self.border_exclusion_labels = list(self.node.get_parameter('border_exclusion_labels').get_parameter_value().string_array_value)
        self.global_frame_id = self.node.get_parameter('global_frame_id').get_parameter_value().string_value
        self.auv_frame_id = self.node.get_parameter('auv_frame_id').get_parameter_value().string_value
        self.detection_frame_id = (
            self.node.get_parameter('detection_frame_id').get_parameter_value().string_value
        )

        auv_to_camera_center_xyz = self._get_vector3_parameter('auv_to_camera_center_xyz')
        auv_to_camera_center_rpy = self._get_vector3_parameter('auv_to_camera_center_rpy')
        camera_center_to_detection_xyz = self._get_vector3_parameter('camera_center_to_detection_xyz')
        camera_center_to_detection_rpy = self._get_vector3_parameter('camera_center_to_detection_rpy')

        self.T_auv_camera_center = self._build_transform_matrix_from_xyz_rpy(
            auv_to_camera_center_xyz,
            auv_to_camera_center_rpy,
        )
        self.T_camera_center_detection = self._build_transform_matrix_from_xyz_rpy(
            camera_center_to_detection_xyz,
            camera_center_to_detection_rpy,
        )
        self.T_auv_detection = concatenate_matrices(
            self.T_auv_camera_center,
            self.T_camera_center_detection,
        )
        self.T_detection_auv = inverse_matrix(self.T_auv_detection)

        if self.pose_source not in {self.POSE_SOURCE_ZED_VIO, self.POSE_SOURCE_AUV_POSE}:
            raise ValueError(
                f"Unsupported pose_source '{self.pose_source}'. "
                f"Expected '{self.POSE_SOURCE_ZED_VIO}' or '{self.POSE_SOURCE_AUV_POSE}'."
            )

        input_format = CompressedImage if self.compressed else Image
        if self.compressed:
            self.node.get_logger().warn(
                (f"WARNING: {self.node.get_name()} running in compressed mode."
                 " Input image assumed to be in compressed format.")
            )
            
        self.bridge = CvBridge()
        if not os.path.exists(model_path):
            self.node.get_logger().error(f"Model path does not exist: {model_path}")
            self.node.get_logger().fatal("Exiting due to missing model.")
            raise FileNotFoundError(f"Model path does not exist: {model_path}")
        
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.sdk_verbose = 1
        init_params.camera_resolution = sl.RESOLUTION.SVGA if sim else sl.RESOLUTION.VGA
        init_params.camera_fps = 30
        init_params.grab_compute_capping_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.depth_maximum_distance = 20.0
        init_params.depth_minimum_distance = 0.01
        init_params.enable_image_validity_check = True
        init_params.input = sl.InputType()
        # init_params.optional_opencv_calibration_file = "/home/douglas/AUV-2026/calibration.yaml"

        if sim:
            stream_ip = self.node.get_parameter('stream_ip').get_parameter_value().string_value
            init_params.set_from_stream(stream_ip, stream_port)

        self.image_buffer = sl.Mat()

        for i in range(5):
            result = self.zed.open(init_params)
            if result == sl.ERROR_CODE.SUCCESS: break
            else:
                self.node.get_logger().error(f"Failed to open zed camera, retrying {5-i} more times")
                sleep(2)
        
        if result != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().fatal("Failed to open zed camera after 5 attempts, exiting")
            exit(1)

        # Enable Positional Tracking
        pos_param = sl.PositionalTrackingParameters()
        pos_param.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        pos_param.enable_imu_fusion = True
        pos_param.set_floor_as_origin = False
        pos_param.enable_area_memory = False
        pos_param.depth_min_range = 0.3
        
        err = self.zed.enable_positional_tracking(pos_param)
        if err != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().error(f"Failed to enable positional tracking: {err}")
            self.zed.close()
            exit(1)

        # Configure ZED Object Detection
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = False
        obj_param.enable_segmentation = False
        obj_param.filtering_mode = sl.OBJECT_FILTERING_MODE.NONE

        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().error(f"Failed to enable object detection: {err}")
            self.zed.close()
            exit(1)
        
        self.obj_runtime_param = sl.CustomObjectDetectionRuntimeParameters()
        self.obj_runtime_param.object_detection_properties.detection_confidence_threshold = 0
        self.obj_runtime_param.object_detection_properties.is_grounded = False
        self.obj_runtime_param.object_detection_properties.is_static = True

        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
        self.runtime_params.confidence_threshold = self.depth_confidence_threshold
        self.runtime_params.texture_confidence_threshold  = 100

        if not sim:
            # Configure streaming
            stream_params = sl.StreamingParameters()
            stream_params.codec = sl.STREAMING_CODEC.H264
            stream_params.port = stream_port
            stream_params.bitrate = 8000 # kbps

            # Enable streaming
            err = self.zed.enable_streaming(stream_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.node.get_logger().error(f"Streaming not enabled: {err}")
                self.zed.close()
                exit(1)

        # Load Model using inference-models with TensorRT acceleration
        self.model = load_model(model_path, self.node.get_logger())
        
        self.node.get_logger().info(f"Setting QOL queue size to: {queue_size}")
        
        self.pub_detection_frame = self.node.create_publisher(
            VisionDetectionFrame,
            self.detection_frame_topic,
            queue_size
        )

        vio_pose_topic = self.node.get_parameter('vio_pose_topic').get_parameter_value().string_value
        self.pub_vio_pose = self.node.create_publisher(
            PoseStamped,
            vio_pose_topic,
            10
        )
        self.node.get_logger().info(f"Publishing world pose to: {vio_pose_topic}")
        self._configure_pose_source()

        self.node.get_logger().info(
            f"Publishing synchronized detection frames to: {self.detection_frame_topic}"
        )

        # Publisher for annotated debug image
        if self.publish_annotated_image:
            publish_topic = self.detection_frame_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")
        
        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

        # Start the grab loop on a dedicated daemon thread
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

    def _depth_callback(self, msg):
        with self._depth_lock:
            self._sensor_depth = msg.data

    def _auv_pose_callback(self, msg: PoseStamped):
        with self._auv_pose_lock:
            self._auv_pose_msg = copy.deepcopy(msg)
            self._has_auv_pose = True

    def _configure_pose_source(self):
        if self.pose_source == self.POSE_SOURCE_AUV_POSE:
            self.auv_pose_sub = self.node.create_subscription(
                PoseStamped,
                self.auv_pose_topic,
                self._auv_pose_callback,
                qos_profile_sensor_data
            )
            self.node.get_logger().info(
                f"Using AUV pose from: {self.auv_pose_topic}"
            )
        else:
            self.auv_pose_sub = None
            self.node.get_logger().info("Using ZED VIO pose to derive synchronized AUV poses")

    def _get_vector3_parameter(self, name: str) -> np.ndarray:
        values = self.node.get_parameter(name).get_parameter_value().double_array_value
        if len(values) != 3:
            raise ValueError(f"Parameter '{name}' must contain exactly 3 values.")
        return np.array([float(values[0]), float(values[1]), float(values[2])], dtype=float)

    def _build_transform_matrix_from_xyz_rpy(
        self,
        translation_xyz: np.ndarray,
        rotation_rpy: np.ndarray,
    ) -> np.ndarray:
        transform = euler_matrix(
            float(rotation_rpy[0]),
            float(rotation_rpy[1]),
            float(rotation_rpy[2]),
        )
        transform[0:3, 3] = translation_xyz
        return transform

    def _build_quaternion_msg(self, orientation_xyzw: np.ndarray) -> Quaternion:
        orientation = Quaternion()
        orientation.x = float(orientation_xyzw[0])
        orientation.y = float(orientation_xyzw[1])
        orientation.z = float(orientation_xyzw[2])
        orientation.w = float(orientation_xyzw[3])
        return orientation

    def _build_pose_message(
        self,
        translation: np.ndarray,
        orientation_xyzw: np.ndarray,
        frame_id: str,
        stamp,
    ) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation = self._build_quaternion_msg(orientation_xyzw)
        return pose_msg

    def _build_pose_message_from_transform(
        self,
        transform: np.ndarray,
        frame_id: str,
        stamp,
    ) -> PoseStamped:
        translation = transform[0:3, 3]
        orientation_xyzw = quaternion_from_matrix(transform)
        return self._build_pose_message(translation, orientation_xyzw, frame_id, stamp)

    def _bbox_from_xyxy(self, x1: float, y1: float, x2: float, y2: float):
        size_x = max(0.0, float(x2) - float(x1))
        size_y = max(0.0, float(y2) - float(y1))
        center_x = float(x1) + size_x / 2.0
        center_y = float(y1) + size_y / 2.0
        return center_x, center_y, size_x, size_y

    def _bbox_from_zed_corners(self, corners) -> tuple[float, float, float, float]:
        if corners is None or len(corners) == 0:
            return 0.0, 0.0, 0.0, 0.0

        xs = [float(point[0]) for point in corners]
        ys = [float(point[1]) for point in corners]
        return self._bbox_from_xyxy(min(xs), min(ys), max(xs), max(ys))

    def _get_auv_world_pose_snapshot(self, stamp):
        if self.pose_source != self.POSE_SOURCE_AUV_POSE:
            return None

        with self._auv_pose_lock:
            if not self._has_auv_pose:
                return None

            pose_msg = copy.deepcopy(self._auv_pose_msg)

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = pose_msg.header.frame_id or self.global_frame_id
        return pose_msg

    def _get_zed_world_pose_snapshot(self, stamp):
        cam_pose = sl.Pose()
        tracking_state = self.zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.node.get_logger().warn(f"VIO tracking not OK: {tracking_state}", throttle_duration_sec=1.0)

        with self._depth_lock:
            sensor_z = self._sensor_depth

        translation = cam_pose.get_translation().get()
        translation = np.array([float(translation[0]), float(translation[1]), -sensor_z])
        rotation = np.array(cam_pose.get_rotation_matrix().r, dtype=float).reshape(3, 3)

        T_world_detection = np.eye(4)
        T_world_detection[0:3, 0:3] = rotation
        T_world_detection[0:3, 3] = translation
        T_world_auv = concatenate_matrices(T_world_detection, self.T_detection_auv)
        return self._build_pose_message_from_transform(T_world_auv, self.global_frame_id, stamp)

    def _capture_auv_pose_snapshot(self, stamp):
        if self.pose_source == self.POSE_SOURCE_AUV_POSE:
            return self._get_auv_world_pose_snapshot(stamp)

        return self._get_zed_world_pose_snapshot(stamp)

    def _has_required_pose_source(self):
        if self.pose_source == self.POSE_SOURCE_AUV_POSE:
            with self._auv_pose_lock:
                return self._has_auv_pose
        return True

    def _toggle_collection_callback(self, request, response):
        self._collecting = request.data
        if self._collecting:
            Path(self.collection_dir).mkdir(parents=True, exist_ok=True)
            self.node.get_logger().info(f"Image collection ENABLED → {self.collection_dir}")
        else:
            self.node.get_logger().info("Image collection DISABLED")
        response.success = True
        response.message = f"Collection {'enabled' if self._collecting else 'disabled'}"
        return response

    def _grab_loop(self):
        """Tight grab loop — runs on its own thread. grab() blocks at camera fps."""
        while rclpy.ok():
            t_start = time.perf_counter()

            result = self.zed.grab(self.runtime_params)
            if result == sl.ERROR_CODE.CORRUPTED_FRAME:
                self.node.get_logger().warn("Corrupted frame", throttle_duration_sec=5.0)
                continue
            if result != sl.ERROR_CODE.SUCCESS:
                self.node.get_logger().warn(f"Zed.grab() failed: {result}", throttle_duration_sec=5.0)
                continue

            if not self._has_required_pose_source():
                self.node.get_logger().warn(
                    f"Waiting for AUV pose on {self.auv_pose_topic}",
                    throttle_duration_sec=1.0
                )
                continue

            frame_stamp = self.node.get_clock().now().to_msg()
            auv_pose_msg = self._capture_auv_pose_snapshot(frame_stamp)
            if auv_pose_msg is None:
                self.node.get_logger().warn(
                    f"Waiting for AUV pose on {self.auv_pose_topic}",
                    throttle_duration_sec=1.0
                )
                continue

            # --- ZED Health Checks ---
            health = self.zed.get_health_status()
            if health.low_image_quality:
                self.node.get_logger().warn("Low image quality", throttle_duration_sec=5.0)
                continue
            if health.low_lighting:
                self.node.get_logger().warn("Low lighting conditions", throttle_duration_sec=5.0)
                continue

            self.zed.retrieve_image(self.image_buffer, sl.VIEW.LEFT)
            img = cv2.cvtColor(self.image_buffer.get_data(), cv2.COLOR_RGBA2RGB)

            # --- Image Collection: save clean image BEFORE annotation ---
            if self._collecting:
                now = time.time()
                if now - self._last_collection_time >= self.collection_interval:
                    self._last_collection_time = now
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    filepath = os.path.join(self.collection_dir, f'front_{timestamp}.jpg')
                    cv2.imwrite(filepath, img)
                    self.node.get_logger().debug(f"Saved {filepath}")

            # Publish VIO pose continuously, even without active detections
            self.pub_vio_pose.publish(auv_pose_msg)

            tracked_detections = get_detections(self, img)

            # --- Border Exclusion Filter ---
            if self.enable_border_exclusion and tracked_detections is not None:
                img_h, img_w = img.shape[:2]
                margin = self.border_exclusion_margin_px
                mask = np.ones(len(tracked_detections), dtype=bool)
                for i in range(len(tracked_detections)):
                    cls_id = int(tracked_detections.class_id[i])
                    if cls_id >= len(self.class_names): continue
                    label = self.class_names[cls_id]
                    
                    if label in self.border_exclusion_labels:
                        x1, y1, x2, y2 = tracked_detections.xyxy[i]
                        if (x1 <= margin or x2 >= img_w - margin or
                            y1 <= margin or y2 >= img_h - margin):
                            mask[i] = False
                tracked_detections = tracked_detections[mask]

            det_msg = VisionDetectionFrame()
            det_msg.header.stamp = frame_stamp
            det_msg.header.frame_id = self.detection_frame_id
            det_msg.auv_pose = auv_pose_msg
            det_objects = []

            if tracked_detections is not None:
                # 1. Ingest into ZED SDK
                custom_boxes = []
                model_bboxes_by_unique_id = {}
                for i in range(len(tracked_detections)):
                    x1, y1, x2, y2 = tracked_detections.xyxy[i]
                    cls_id = int(tracked_detections.class_id[i])
                    if cls_id >= len(self.class_names): continue

                    box = sl.CustomBoxObjectData()
                    box.probability = float(tracked_detections.confidence[i])
                    box.label = cls_id
                    box.is_grounded = False
                    unique_object_id = f"front_cam_det_{i}"
                    box.unique_object_id = unique_object_id
                    model_bboxes_by_unique_id[unique_object_id] = (
                        float(x1),
                        float(y1),
                        float(x2),
                        float(y2),
                    )

                    # --- Gate top crop: only feed the top portion of gate bounding boxes ---
                    # The gate's legs extend into noisy stereo territory causing bad depth
                    label = self.class_names[cls_id]
                    if self.enable_gate_top_crop and label == "gate":
                        h = y2 - y1
                        y2 = y1 + h * self.gate_top_crop_ratio

                    # Clamp to image bounds (PyZED requires unsigned int arrays)
                    img_h, img_w = img.shape[:2]
                    x1_c = min(img_w - 1, max(0, int(x1)))
                    y1_c = min(img_h - 1, max(0, int(y1)))
                    x2_c = min(img_w - 1, max(0, int(x2)))
                    y2_c = min(img_h - 1, max(0, int(y2)))

                    # Format ZED requires: top-left, top-right, bottom-right, bottom-left
                    box.bounding_box_2d = np.array([
                        [x1_c, y1_c], [x2_c, y1_c], [x2_c, y2_c], [x1_c, y2_c]
                    ], dtype=np.uint32)
                    custom_boxes.append(box)

                self.zed.ingest_custom_box_objects(custom_boxes)

                # 2. Retrieve 3D objects
                objects = sl.Objects()
                self.zed.retrieve_custom_objects(objects, self.obj_runtime_param)

                # 3. Build synchronized camera-frame detections
                for obj in objects.object_list:
                    pos_cam = obj.position
                    if np.isnan(pos_cam).any() or np.isinf(pos_cam).any():
                        continue
                    if pos_cam[0] < 0:
                        continue

                    cam_pos_vec = np.array([float(pos_cam[0]), float(pos_cam[1]), float(pos_cam[2])])

                    detection = VisionDetection()
                    detection.label = self.class_names[obj.raw_label]
                    detection.confidence = float(obj.confidence) / 100.0
                    detection.pose_camera.pose.position.x = float(cam_pos_vec[0])
                    detection.pose_camera.pose.position.y = float(cam_pos_vec[1])
                    detection.pose_camera.pose.position.z = float(cam_pos_vec[2])
                    detection.pose_camera.pose.orientation.w = 1.0

                    cov = obj.position_covariance
                    cov_cam = np.array([
                        [float(cov[0]), float(cov[1]), float(cov[2])],
                        [float(cov[1]), float(cov[3]), float(cov[4])],
                        [float(cov[2]), float(cov[4]), float(cov[5])]
                    ])

                    ros_cov = [0.0] * 36
                    for r in range(3):
                        for c in range(3):
                            ros_cov[r * 6 + c] = float(cov_cam[r, c])
                    detection.pose_camera.covariance = ros_cov

                    bbox_xyxy = model_bboxes_by_unique_id.get(obj.unique_object_id)
                    if bbox_xyxy is not None:
                        (
                            detection.bbox_center_x,
                            detection.bbox_center_y,
                            detection.bbox_size_x,
                            detection.bbox_size_y,
                        ) = self._bbox_from_xyxy(*bbox_xyxy)
                    else:
                        (
                            detection.bbox_center_x,
                            detection.bbox_center_y,
                            detection.bbox_size_x,
                            detection.bbox_size_y,
                        ) = self._bbox_from_zed_corners(obj.bounding_box_2d)
                    det_objects.append(detection)

            det_msg.detections = det_objects
            self.pub_detection_frame.publish(det_msg)


            # Always publish the (possibly annotated) image, even with no detections
            publish_annotated_image_util(self, img, tracked_detections)
            
            t_end = time.perf_counter()
            self.node.get_logger().debug(f"Detection latency: {(t_end - t_start)*1000:.1f} ms | Active 3D detections: {len(det_objects)}")
