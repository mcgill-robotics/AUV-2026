#!/usr/bin/env python3
import copy
from datetime import datetime
from functools import partial
import os
from time import sleep
import threading
import time

from auv_msgs.msg import VisionDetection, VisionDetectionFrame
from auv_msgs.srv import AutomaticCapture
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    sl = None
    ZED_SDK_AVAILABLE = False
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf_transformations import concatenate_matrices, inverse_matrix

from vision.object_detection.utils import (
    bbox_from_xyxy,
    bbox_from_zed_corners,
    build_pose_message_from_transform,
    build_transform_matrix_from_xyz_rpy,
    get_detections,
    get_vector3_parameter,
    load_model,
    publish_annotated_image_util,
    toggle_collection_callback_util,
)

class FrontCamObjectDetectorNode():
    POSE_SOURCE_ZED_VIO = "zed_vio"
    POSE_SOURCE_AUV_POSE = "auv_pose"
    
    if ZED_SDK_AVAILABLE:
        ZED_RUNTIME_CAMERA_SETTINGS = (
            ("zed_brightness", sl.VIDEO_SETTINGS.BRIGHTNESS),
            ("zed_contrast", sl.VIDEO_SETTINGS.CONTRAST),
            ("zed_hue", sl.VIDEO_SETTINGS.HUE),
            ("zed_saturation", sl.VIDEO_SETTINGS.SATURATION),
            ("zed_sharpness", sl.VIDEO_SETTINGS.SHARPNESS),
            ("zed_gamma", sl.VIDEO_SETTINGS.GAMMA),
            ("zed_auto_exposure_gain", sl.VIDEO_SETTINGS.AEC_AGC),
            ("zed_auto_whitebalance", sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO),
            ("zed_led_status", sl.VIDEO_SETTINGS.LED_STATUS),
        )
    else:
        ZED_RUNTIME_CAMERA_SETTINGS = ()

    @staticmethod
    def _enum_from_name(enum_cls, raw_name, param_name: str):
        if isinstance(raw_name, bool):
            raw_name = "ON" if raw_name else "OFF"
        normalized_name = raw_name.strip().upper()
        if not hasattr(enum_cls, normalized_name):
            valid_names = sorted(name for name in dir(enum_cls) if not name.startswith("_"))
            raise ValueError(
                f"Unsupported {param_name} '{raw_name}'. Expected one of: {', '.join(valid_names)}"
            )
        return getattr(enum_cls, normalized_name)

    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter('publish_annotated_every_n_frames', 1)
        self.node.declare_parameter('publish_camera_info', True)
        self.node.declare_parameter('publish_depth_image', False)
        self.node.declare_parameter('publish_depth_compressed', False)
        self.node.declare_parameter('publish_depth_every_n_frames', 5)
        self.node.declare_parameter('depth_collection_dir', '/tmp/depth_collection')
        self.node.declare_parameter('collect_depth_image', True)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter("detection_frame_topic", Parameter.Type.STRING)
        self.node.declare_parameter("enable_object_detection", True)
        self.node.declare_parameter("has_zed_sdk", True)
        self.node.declare_parameter("image_topic", "")

        self.node.declare_parameter("model_detection_threshold", 0.40)
        self.node.declare_parameter("depth_confidence_threshold", 95)
        self.node.declare_parameter("zed_depth_maximum_distance", 20.0)
        self.node.declare_parameter("zed_depth_minimum_distance", 0.01)
        self.node.declare_parameter("zed_positional_tracking_depth_min_range", 0.3)
        self.node.declare_parameter("zed_depth_mode", "NEURAL")
        self.node.declare_parameter("zed_depth_stabilization", 30)
        self.node.declare_parameter("zed_camera_resolution_sim", "SVGA")
        self.node.declare_parameter("zed_camera_resolution_real", "HD1080")
        self.node.declare_parameter("zed_camera_fps_sim", 30)
        self.node.declare_parameter("zed_camera_fps_real", 15)
        self.node.declare_parameter(
            "zed_camera_flip_mode",
            "OFF",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.node.declare_parameter("zed_self_calib", True)
        self.node.declare_parameter("zed_enable_right_side_measure", False)
        self.node.declare_parameter("zed_sdk_verbose", 1)
        self.node.declare_parameter("zed_sdk_gpu_id", -1)
        self.node.declare_parameter("zed_enable_image_enhancement", True)
        self.node.declare_parameter("zed_open_timeout_sec", 5.0)
        self.node.declare_parameter("zed_async_grab_camera_recovery", False)
        self.node.declare_parameter("zed_grab_compute_capping_fps", 0.0)
        self.node.declare_parameter("zed_enable_image_validity_check", True)
        self.node.declare_parameter("zed_optional_opencv_calibration_file", "")
        self.node.declare_parameter("zed_brightness", 4)
        self.node.declare_parameter("zed_contrast", 4)
        self.node.declare_parameter("zed_hue", 0)
        self.node.declare_parameter("zed_saturation", 4)
        self.node.declare_parameter("zed_sharpness", 4)
        self.node.declare_parameter("zed_gamma", 8)
        self.node.declare_parameter("zed_gain", 80)
        self.node.declare_parameter("zed_exposure", 80)
        self.node.declare_parameter("zed_auto_exposure_gain", True)
        self.node.declare_parameter("zed_auto_whitebalance", True)
        self.node.declare_parameter("zed_whitebalance_temperature", 4200)
        self.node.declare_parameter("zed_led_status", -1)

        # we consume stream_ip and sim properties for zed sdk configuration
        self.node.declare_parameter('sim', Parameter.Type.BOOL)
        self.node.declare_parameter('stream_ip', Parameter.Type.STRING)
        self.node.declare_parameter('stream_port', Parameter.Type.INTEGER)
        self.node.declare_parameter('enable_vio', True)
        self.node.declare_parameter('vio_pose_topic', '/vision/vio_pose')
        self.node.declare_parameter('pose_source', self.POSE_SOURCE_ZED_VIO)
        self.node.declare_parameter('broadcast_vio_tf', False)
        self.node.declare_parameter('auv_pose_topic', 'state/pose')
        self.node.declare_parameter('enable_gate_top_crop', True)
        self.node.declare_parameter('gate_top_crop_ratio', 0.50)
        self.node.declare_parameter('enable_border_exclusion', True)
        self.node.declare_parameter('border_exclusion_margin_px', 20)
        self.node.declare_parameter('border_exclusion_labels', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('global_frame_id', 'pool_link')
        self.node.declare_parameter('auv_frame_id', 'auv_link')
        self.node.declare_parameter('vio_frame_id', 'auv_vio_link')
        self.node.declare_parameter('detection_frame_id', 'zed_left_camera_frame')
        self.node.declare_parameter('image_frame_id', 'zed_left_camera_frame_optical')
        self.node.declare_parameter('auv_to_camera_center_xyz', [0.0, 0.0, 0.0])
        self.node.declare_parameter('auv_to_camera_center_rpy', [0.0, 0.0, 0.0])
        self.node.declare_parameter('camera_center_to_detection_xyz', [0.0, 0.06, 0.0])
        self.node.declare_parameter('camera_center_to_detection_rpy', [0.0, 0.0, 0.0])

        # Image collection parameters
        self.node.declare_parameter('collection_dir', '/tmp/front_cam_collection')
        self.node.declare_parameter('collection_interval_seconds', 2.0)
        self.node.declare_parameter('collection_file_extension', '.png')

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
        self.depth_collection_dir = self.node.get_parameter('depth_collection_dir').get_parameter_value().string_value
        self.collection_interval = self.node.get_parameter('collection_interval_seconds').get_parameter_value().double_value
        self.collection_extension = self.node.get_parameter('collection_file_extension').get_parameter_value().string_value
        self._frame_counter = 0

        # Service to toggle image collection on/off

        self.node.create_service(AutomaticCapture, '/vision/front_cam/toggle_collection', partial(toggle_collection_callback_util, self))
        self.node.create_service(Trigger, '/vision/front_cam/toggle_annotated_image', self._toggle_annotated_image_callback)
 
        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        self.detection_frame_topic = (
            self.node.get_parameter('detection_frame_topic').get_parameter_value().string_value
        )
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.annotated_image_enabled = self.publish_annotated_image
        self.publish_annotated_every_n_frames = self.node.get_parameter('publish_annotated_every_n_frames').get_parameter_value().integer_value
        self.publish_camera_info = self.node.get_parameter('publish_camera_info').get_parameter_value().bool_value
        self.publish_depth_image = self.node.get_parameter('publish_depth_image').get_parameter_value().bool_value
        self.publish_depth_compressed = self.node.get_parameter('publish_depth_compressed').get_parameter_value().bool_value
        self.broadcast_vio_tf = self.node.get_parameter('broadcast_vio_tf').get_parameter_value().bool_value
        self.publish_depth_every_n_frames = self.node.get_parameter('publish_depth_every_n_frames').get_parameter_value().integer_value
        self.collect_depth_image = self.node.get_parameter('collect_depth_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.enable_object_detection = self.node.get_parameter('enable_object_detection').get_parameter_value().bool_value
        self.has_zed_sdk = self.node.get_parameter('has_zed_sdk').get_parameter_value().bool_value
        self.image_topic = self.node.get_parameter('image_topic').get_parameter_value().string_value

        self.conf_threshold = self.node.get_parameter('model_detection_threshold').get_parameter_value().double_value
        self.depth_confidence_threshold = (
            self.node.get_parameter('depth_confidence_threshold').get_parameter_value().integer_value
        )
        self.zed_depth_maximum_distance = (
            self.node.get_parameter('zed_depth_maximum_distance').get_parameter_value().double_value
        )
        if self.zed_depth_maximum_distance > 40.0:
            self.node.get_logger().warn(f"zed_depth_maximum_distance capped from {self.zed_depth_maximum_distance} to 40.0 (ZED maximum)")
            self.zed_depth_maximum_distance = 40.0

        self.zed_depth_minimum_distance = (
            self.node.get_parameter('zed_depth_minimum_distance').get_parameter_value().double_value
        )
        if self.zed_depth_minimum_distance < 0.2:
            self.node.get_logger().warn(f"zed_depth_minimum_distance capped from {self.zed_depth_minimum_distance} to 0.2 (ZED minimum)")
            self.zed_depth_minimum_distance = 0.2

        self.zed_positional_tracking_depth_min_range = (
            self.node.get_parameter('zed_positional_tracking_depth_min_range').get_parameter_value().double_value
        )
        if self.zed_positional_tracking_depth_min_range < 0.2:
            self.node.get_logger().warn(f"zed_positional_tracking_depth_min_range capped from {self.zed_positional_tracking_depth_min_range} to 0.2 (ZED minimum)")
            self.zed_positional_tracking_depth_min_range = 0.2

        self.zed_depth_mode_name = (
            self.node.get_parameter('zed_depth_mode').get_parameter_value().string_value
        )
        self.zed_depth_stabilization = (
            self.node.get_parameter('zed_depth_stabilization').get_parameter_value().integer_value
        )
        sim = self.node.get_parameter('sim').get_parameter_value().bool_value
        self.zed_camera_resolution_name = self.node.get_parameter(
            'zed_camera_resolution_sim' if sim else 'zed_camera_resolution_real'
        ).get_parameter_value().string_value
        self.zed_camera_fps = self.node.get_parameter(
            'zed_camera_fps_sim' if sim else 'zed_camera_fps_real'
        ).get_parameter_value().integer_value
        self.zed_camera_flip_mode_name = self.node.get_parameter(
            'zed_camera_flip_mode'
        ).get_parameter_value().string_value
        self.zed_self_calib = self.node.get_parameter(
            'zed_self_calib'
        ).get_parameter_value().bool_value
        self.zed_enable_right_side_measure = self.node.get_parameter(
            'zed_enable_right_side_measure'
        ).get_parameter_value().bool_value
        self.zed_sdk_verbose = self.node.get_parameter(
            'zed_sdk_verbose'
        ).get_parameter_value().integer_value
        self.zed_sdk_gpu_id = self.node.get_parameter(
            'zed_sdk_gpu_id'
        ).get_parameter_value().integer_value
        self.zed_enable_image_enhancement = self.node.get_parameter(
            'zed_enable_image_enhancement'
        ).get_parameter_value().bool_value
        self.zed_open_timeout_sec = self.node.get_parameter(
            'zed_open_timeout_sec'
        ).get_parameter_value().double_value
        self.zed_async_grab_camera_recovery = self.node.get_parameter(
            'zed_async_grab_camera_recovery'
        ).get_parameter_value().bool_value
        self.zed_grab_compute_capping_fps = self.node.get_parameter(
            'zed_grab_compute_capping_fps'
        ).get_parameter_value().double_value
        self.zed_enable_image_validity_check = self.node.get_parameter(
            'zed_enable_image_validity_check'
        ).get_parameter_value().bool_value
        self.zed_optional_opencv_calibration_file = self.node.get_parameter(
            'zed_optional_opencv_calibration_file'
        ).get_parameter_value().string_value
        stream_port = self.node.get_parameter('stream_port').get_parameter_value().integer_value
        self.enable_vio = self.node.get_parameter('enable_vio').get_parameter_value().bool_value
        if not self.has_zed_sdk and self.enable_vio:
            self.enable_vio = False
            self.node.get_logger().warn(
                (f"WARNING: {self.node.get_name()} disabling vio since has_zed_sdk is False.")
            )
        self.pose_source = self.node.get_parameter('pose_source').get_parameter_value().string_value
        self.auv_pose_topic = self.node.get_parameter('auv_pose_topic').get_parameter_value().string_value
        self.enable_gate_top_crop = self.node.get_parameter('enable_gate_top_crop').get_parameter_value().bool_value
        self.gate_top_crop_ratio = self.node.get_parameter('gate_top_crop_ratio').get_parameter_value().double_value
        self.enable_border_exclusion = self.node.get_parameter('enable_border_exclusion').get_parameter_value().bool_value
        self.border_exclusion_margin_px = self.node.get_parameter('border_exclusion_margin_px').get_parameter_value().integer_value
        self.border_exclusion_labels = list(self.node.get_parameter('border_exclusion_labels').get_parameter_value().string_array_value)
        self.global_frame_id = self.node.get_parameter('global_frame_id').get_parameter_value().string_value
        self.auv_frame_id = self.node.get_parameter('auv_frame_id').get_parameter_value().string_value
        self.vio_frame_id = self.node.get_parameter('vio_frame_id').get_parameter_value().string_value
        self.detection_frame_id = (
            self.node.get_parameter('detection_frame_id').get_parameter_value().string_value
        )
        self.image_frame_id = (
            self.node.get_parameter('image_frame_id').get_parameter_value().string_value
        )

        auv_to_camera_center_xyz = get_vector3_parameter(self.node, 'auv_to_camera_center_xyz')
        auv_to_camera_center_rpy = get_vector3_parameter(self.node, 'auv_to_camera_center_rpy')
        camera_center_to_detection_xyz = get_vector3_parameter(self.node, 'camera_center_to_detection_xyz')
        camera_center_to_detection_rpy = get_vector3_parameter(self.node, 'camera_center_to_detection_rpy')

        self.T_auv_camera_center = build_transform_matrix_from_xyz_rpy(
            auv_to_camera_center_xyz,
            auv_to_camera_center_rpy,
        )
        self.T_camera_center_detection = build_transform_matrix_from_xyz_rpy(
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
        if not self.enable_vio and self.pose_source == self.POSE_SOURCE_ZED_VIO:
            raise ValueError(
                "pose_source='zed_vio' requires enable_vio=true. "
                "Either enable VIO or switch pose_source to 'auv_pose'."
            )
        if not self.enable_vio and self.broadcast_vio_tf:
            self.node.get_logger().warn(
                "broadcast_vio_tf is enabled but enable_vio is false; VIO TF publication will be skipped."
            )
        if not self.enable_vio and self.zed_depth_stabilization != 0:
            self.node.get_logger().warn(
                "enable_vio is false, so zed_depth_stabilization will be forced to 0 "
                "to avoid implicit positional tracking enablement."
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
        
        if self.has_zed_sdk:
            if not ZED_SDK_AVAILABLE:
                self.node.get_logger().fatal("has_zed_sdk is true but pyzed.sl is not installed!")
                raise ImportError("pyzed.sl is not installed.")
                
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.sdk_verbose = self.zed_sdk_verbose
            init_params.sdk_gpu_id = self.zed_sdk_gpu_id
            init_params.camera_resolution = self._enum_from_name(
                sl.RESOLUTION,
                self.zed_camera_resolution_name,
                'zed_camera_resolution',
            )
            init_params.camera_fps = self.zed_camera_fps
            init_params.camera_disable_self_calib = not self.zed_self_calib
            init_params.camera_image_flip = self._enum_from_name(
                sl.FLIP_MODE,
                self.zed_camera_flip_mode_name,
                'zed_camera_flip_mode',
            )
            init_params.enable_right_side_measure = self.zed_enable_right_side_measure
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
            init_params.depth_mode = self._enum_from_name(
                sl.DEPTH_MODE,
                self.zed_depth_mode_name,
                'zed_depth_mode',
            )
            init_params.depth_stabilization = self.zed_depth_stabilization if self.enable_vio else 0
            init_params.depth_maximum_distance = self.zed_depth_maximum_distance
            init_params.depth_minimum_distance = self.zed_depth_minimum_distance
            init_params.enable_image_enhancement = self.zed_enable_image_enhancement
            init_params.open_timeout_sec = self.zed_open_timeout_sec
            init_params.async_grab_camera_recovery = self.zed_async_grab_camera_recovery
            init_params.grab_compute_capping_fps = self.zed_grab_compute_capping_fps
            init_params.enable_image_validity_check = self.zed_enable_image_validity_check
            if self.zed_optional_opencv_calibration_file:
                init_params.optional_opencv_calibration_file = self.zed_optional_opencv_calibration_file
            init_params.input = sl.InputType()

            if sim:
                stream_ip = self.node.get_parameter('stream_ip').get_parameter_value().string_value
                init_params.set_from_stream(stream_ip, stream_port)

            self.image_buffer = sl.Mat()

            for i in range(5):
                result = self.zed.open(init_params)
                if result == sl.ERROR_CODE.SUCCESS: break
                else:
                    self.node.get_logger().error(f"Failed to open zed camera:, retrying {5-i} more times: {result}")
                    self.zed.close()
                    sleep(2)
            
            if result != sl.ERROR_CODE.SUCCESS:
                self.node.get_logger().fatal("Failed to open zed camera after 5 attempts")
                raise RuntimeError("Failed to open ZED camera after 5 attempts.")

            self._apply_zed_camera_controls()

            if self.enable_vio:
                # Enable Positional Tracking
                pos_param = sl.PositionalTrackingParameters()
                pos_param.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
                pos_param.enable_imu_fusion = True
                pos_param.set_floor_as_origin = False
                pos_param.enable_area_memory = False
                pos_param.depth_min_range = self.zed_positional_tracking_depth_min_range
                
                err = self.zed.enable_positional_tracking(pos_param)
                if err != sl.ERROR_CODE.SUCCESS:
                    self.node.get_logger().error(f"Failed to enable positional tracking: {err}")
                    self.zed.close()
                    raise RuntimeError(f"Failed to enable ZED positional tracking: {err}")
            else:
                self.node.get_logger().info("VIO disabled: skipping ZED positional tracking and VIO pose publication.")

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
                raise RuntimeError(f"Failed to enable ZED object detection: {err}")
            
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
                    raise RuntimeError(f"Failed to enable ZED streaming: {err}")

        # Load Model using inference-models with TensorRT acceleration
        if self.enable_object_detection:
            self.model = load_model(model_path, self.node.get_logger())
        else:
            self.model = None
            self.node.get_logger().info("Object detection is disabled. Publishing raw feed only.")
        
        self.node.get_logger().info(f"Setting QOL queue size to: {queue_size}")
        if self.has_zed_sdk:
            self.camera_info_template = self._build_camera_info_template()
        
        self.pub_detection_frame = self.node.create_publisher(
            VisionDetectionFrame,
            self.detection_frame_topic,
            queue_size
        )

        if self.publish_camera_info:
            self.camera_info_topic = self.detection_frame_topic + "/camera_info"
            self.pub_camera_info = self.node.create_publisher(
                CameraInfo,
                self.camera_info_topic,
                queue_size,
            )
            self.node.get_logger().info(f"Publishing camera info to: {self.camera_info_topic}")
        else:
            self.camera_info_topic = None
            self.pub_camera_info = None

        vio_pose_topic = self.node.get_parameter('vio_pose_topic').get_parameter_value().string_value
        if self.enable_vio:
            self.pub_vio_pose = self.node.create_publisher(
                PoseStamped,
                vio_pose_topic,
                10
            )
            self.node.get_logger().info(f"Publishing VIO pose measurement to: {vio_pose_topic}")
        else:
            self.pub_vio_pose = None
        self._configure_pose_source()

        self.node.get_logger().info(
            f"Publishing synchronized detection frames to: {self.detection_frame_topic}"
        )
        
        if self.publish_annotated_image:
            publish_topic = self.detection_frame_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size
            )
            self.node.get_logger().info(
                f"Annotated debug image topic ready at: {publish_topic} "
                f"(enabled={self.annotated_image_enabled})"
            )
            
        self.depth_image_msg_type = None
        if self.publish_depth_image:
            depth_topic = self.detection_frame_topic + "/depth"
            if self.publish_depth_compressed:
                self.depth_image_msg_type = CompressedImage
                depth_topic += "/compressed"
            else:
                self.depth_image_msg_type = Image

            self.pub_depth_image = self.node.create_publisher(
                self.depth_image_msg_type,
                depth_topic,
                queue_size
            )
            self.node.get_logger().info(f"Publishing depth image to: {depth_topic}")
        
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

        if self.has_zed_sdk:
            # Start the grab loop on a dedicated daemon thread
            self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
            self._grab_thread.start()
        else:
            self.node.get_logger().info(f"ZED SDK disabled. Subscribing to {self.image_topic}")
            self.image_sub = self.node.create_subscription(
                input_format,
                self.image_topic,
                self._image_callback,
                queue_size
            )

    def _apply_zed_camera_controls(self):
        applied_settings = []
        for parameter_name, setting in self.ZED_RUNTIME_CAMERA_SETTINGS:
            value = None
            if parameter_name in {"zed_auto_exposure_gain", "zed_auto_whitebalance"}:
                value = int(self.node.get_parameter(parameter_name).get_parameter_value().bool_value)
            else:
                value = self.node.get_parameter(parameter_name).get_parameter_value().integer_value

            err = self.zed.set_camera_settings(setting, value)
            if err != sl.ERROR_CODE.SUCCESS:
                self.zed.close()
                raise RuntimeError(
                    f"Failed to set {parameter_name}={value} on the ZED camera: {err}"
                )
            applied_settings.append(f"{parameter_name}={value}")

        if not self.node.get_parameter('zed_auto_exposure_gain').get_parameter_value().bool_value:
            for parameter_name, setting in (
                ("zed_gain", sl.VIDEO_SETTINGS.GAIN),
                ("zed_exposure", sl.VIDEO_SETTINGS.EXPOSURE),
            ):
                value = self.node.get_parameter(parameter_name).get_parameter_value().integer_value
                err = self.zed.set_camera_settings(setting, value)
                if err != sl.ERROR_CODE.SUCCESS:
                    self.zed.close()
                    raise RuntimeError(
                        f"Failed to set {parameter_name}={value} on the ZED camera: {err}"
                    )
                applied_settings.append(f"{parameter_name}={value}")

        if not self.node.get_parameter('zed_auto_whitebalance').get_parameter_value().bool_value:
            value = self.node.get_parameter(
                'zed_whitebalance_temperature'
            ).get_parameter_value().integer_value
            err = self.zed.set_camera_settings(
                sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE,
                value,
            )
            if err != sl.ERROR_CODE.SUCCESS:
                self.zed.close()
                raise RuntimeError(
                    f"Failed to set zed_whitebalance_temperature={value} on the ZED camera: {err}"
                )
            applied_settings.append(f"zed_whitebalance_temperature={value}")

        controls_suffix = f" | controls: {', '.join(applied_settings)}" if applied_settings else ""
        self.node.get_logger().info(
            f"Configured ZED camera: resolution={self.zed_camera_resolution_name} "
            f"fps={self.zed_camera_fps} flip={self.zed_camera_flip_mode_name}{controls_suffix}"
        )

    def _depth_callback(self, msg):
        with self._depth_lock:
            self._sensor_depth = msg.data

    def _auv_pose_callback(self, msg: PoseStamped):
        with self._auv_pose_lock:
            self._auv_pose_msg = copy.deepcopy(msg)
            self._has_auv_pose = True

    def _toggle_annotated_image_callback(self, request, response):
        del request
        self.annotated_image_enabled = not self.annotated_image_enabled
        response.success = True
        response.message = (
            f"Annotated image {'enabled' if self.annotated_image_enabled else 'disabled'}"
        )
        self.node.get_logger().info(response.message)
        return response

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

    def _get_vio_world_pose_snapshot(self, stamp):
        if not self.enable_vio:
            return None

        cam_pose = sl.Pose()
        tracking_state = self.zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.node.get_logger().warn(f"VIO tracking not OK: {tracking_state}", throttle_duration_sec=1.0)
            return None

        with self._depth_lock:
            sensor_z = self._sensor_depth

        translation = cam_pose.get_translation().get()
        translation = np.array([float(translation[0]), float(translation[1]), -sensor_z])
        rotation = np.array(cam_pose.get_rotation_matrix().r, dtype=float).reshape(3, 3)

        T_world_detection = np.eye(4)
        T_world_detection[0:3, 0:3] = rotation
        T_world_detection[0:3, 3] = translation
        T_world_auv = concatenate_matrices(T_world_detection, self.T_detection_auv)
        return build_pose_message_from_transform(T_world_auv, self.global_frame_id, stamp)

    def _capture_detection_frame_auv_pose_snapshot(self, stamp, vio_pose_msg=None):
        if self.pose_source == self.POSE_SOURCE_AUV_POSE:
            return self._get_auv_world_pose_snapshot(stamp)

        return vio_pose_msg if vio_pose_msg is not None else self._get_vio_world_pose_snapshot(stamp)

    def _build_camera_info_template(self) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.image_frame_id

        zed_info = self.zed.get_camera_information()
        resolution = zed_info.camera_configuration.resolution
        calibration = zed_info.camera_configuration.calibration_parameters.left_cam

        fx = float(calibration.fx)
        fy = float(calibration.fy)
        cx = float(calibration.cx)
        cy = float(calibration.cy)
        distortion = list(calibration.disto)

        camera_info.width = int(resolution.width)
        camera_info.height = int(resolution.height)
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [float(value) for value in distortion]
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return camera_info

    def _publish_camera_info(self, stamp):
        if self.pub_camera_info is None:
            return

        camera_info_msg = copy.deepcopy(self.camera_info_template)
        camera_info_msg.header.stamp = stamp
        self.pub_camera_info.publish(camera_info_msg)

    def _has_required_pose_source(self):
        if self.pose_source == self.POSE_SOURCE_AUV_POSE:
            with self._auv_pose_lock:
                return self._has_auv_pose
        return True


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

            frame_stamp = self.node.get_clock().now().to_msg()
            vio_pose_msg = self._get_vio_world_pose_snapshot(frame_stamp) if self.enable_vio else None
            
            # --- ZED Health Checks ---
            health = self.zed.get_health_status()
            if health.low_image_quality:
                self.node.get_logger().warn("Low image quality", throttle_duration_sec=5.0)
            if health.low_lighting:
                self.node.get_logger().warn("Low lighting conditions", throttle_duration_sec=5.0)

            self.zed.retrieve_image(self.image_buffer, sl.VIEW.LEFT)
            img = cv2.cvtColor(self.image_buffer.get_data(), cv2.COLOR_RGBA2RGB)
            self._frame_counter += 1

            # --- Depth & Collection ---
            if not hasattr(self, "depth_buffer"):
                self.depth_buffer = sl.Mat()
            
            is_collection_interval = (time.time() - self._last_collection_time) >= self.collection_interval
            is_collection_frame = self._collecting and is_collection_interval
            
            should_get_depth = (self.publish_depth_image and self._frame_counter % self.publish_depth_every_n_frames == 0) or \
                               (self.collect_depth_image and is_collection_frame)
            
            if should_get_depth:
                self.zed.retrieve_measure(self.depth_buffer, sl.MEASURE.DEPTH)
                depth = self.depth_buffer.get_data() # sl.UNIT.METER, shape (H,W) with NaNs
            else:
                depth = None

            self._process_frame(
                img, 
                frame_stamp, 
                depth=depth, 
                vio_pose_msg=vio_pose_msg, 
                is_collection_frame=is_collection_frame, 
                t_start=t_start
            )

    def _image_callback(self, msg):
        try:
            if self.compressed:
                img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge error: {e}", throttle_duration_sec=5.0)
            return
            
        frame_stamp = msg.header.stamp
        t_start = time.perf_counter()
        self._frame_counter += 1
        
        is_collection_frame = self._collecting and (time.time() - self._last_collection_time) >= self.collection_interval
        
        self._process_frame(
            img, 
            frame_stamp, 
            depth=None, 
            vio_pose_msg=None, 
            is_collection_frame=is_collection_frame, 
            t_start=t_start
        )

    def _process_frame(self, img, frame_stamp, depth=None, vio_pose_msg=None, is_collection_frame=False, t_start=None):
        if vio_pose_msg is not None:
            # Publish VIO pose continuously, even without active detections
            self.pub_vio_pose.publish(vio_pose_msg)
        
        # Publish VIO TF if configured
        if vio_pose_msg is not None and self.broadcast_vio_tf:
            t = TransformStamped()
            t.header.stamp = frame_stamp
            t.header.frame_id = self.global_frame_id
            t.child_frame_id = self.vio_frame_id
            t.transform.translation.x = vio_pose_msg.pose.position.x
            t.transform.translation.y = vio_pose_msg.pose.position.y
            t.transform.translation.z = vio_pose_msg.pose.position.z
            t.transform.rotation = vio_pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        if not self._has_required_pose_source():
            self.node.get_logger().warn(
                f"Waiting for AUV pose on {self.auv_pose_topic}",
                throttle_duration_sec=1.0
            )
            return

        detection_frame_auv_pose_msg = self._capture_detection_frame_auv_pose_snapshot(
            frame_stamp,
            vio_pose_msg=vio_pose_msg,
        )
        if detection_frame_auv_pose_msg is None:
            self.node.get_logger().warn(
                f"Waiting for AUV pose on {self.auv_pose_topic}",
                throttle_duration_sec=1.0
            )
            return

        if is_collection_frame:
            self._last_collection_time = time.time()
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            
            filepath = os.path.join(self.collection_dir, f'front_{timestamp}{self.collection_extension}')
            cv2.imwrite(filepath, img)
            
            if self.collect_depth_image and depth is not None:
                depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                depth_mm = (depth_mm * 1000.0).clip(0, 65535).astype(np.uint16)
                depth_path = os.path.join(self.depth_collection_dir, f'depth_{timestamp}.png')
                cv2.imwrite(depth_path, depth_mm)
            
            self.node.get_logger().debug(f"Collected dataset step: {timestamp}")
        
        if self.publish_depth_image and depth is not None and self._frame_counter % self.publish_depth_every_n_frames == 0:
            if self.publish_depth_compressed:
                depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                depth_mm = (depth_mm * 1000.0).clip(0, 65535).astype(np.uint16)
                depth_msg = self.bridge.cv2_to_compressed_imgmsg(depth_mm, dst_format='png')
            else:
                depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            
            depth_msg.header.stamp = frame_stamp
            depth_msg.header.frame_id = self.image_frame_id
            self.pub_depth_image.publish(depth_msg)

        if self.has_zed_sdk:
            self._publish_camera_info(frame_stamp)

        if self.enable_object_detection:
            tracked_detections = get_detections(self, img)
        else:
            tracked_detections = None

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
        det_msg.auv_pose = detection_frame_auv_pose_msg
        det_objects = []

        if tracked_detections is not None:
            if self.has_zed_sdk:
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

                    ros_cov = np.zeros((6, 6), dtype=float)
                    ros_cov[:3, :3] = cov_cam
                    detection.pose_camera.covariance = ros_cov.flatten().tolist()

                    bbox_xyxy = model_bboxes_by_unique_id.get(obj.unique_object_id)
                    if bbox_xyxy is not None:
                        (
                            detection.bbox_center_x,
                            detection.bbox_center_y,
                            detection.bbox_size_x,
                            detection.bbox_size_y,
                        ) = bbox_from_xyxy(*bbox_xyxy)
                    else:
                        (
                            detection.bbox_center_x,
                            detection.bbox_center_y,
                            detection.bbox_size_x,
                            detection.bbox_size_y,
                        ) = bbox_from_zed_corners(obj.bounding_box_2d)
                    det_objects.append(detection)
            else:
                # ZED SDK Disabled: publish 2D detections without depth
                for i in range(len(tracked_detections)):
                    x1, y1, x2, y2 = tracked_detections.xyxy[i]
                    cls_id = int(tracked_detections.class_id[i])
                    if cls_id >= len(self.class_names): continue

                    detection = VisionDetection()
                    detection.label = self.class_names[cls_id]
                    detection.confidence = float(tracked_detections.confidence[i])
                    (
                        detection.bbox_center_x,
                        detection.bbox_center_y,
                        detection.bbox_size_x,
                        detection.bbox_size_y,
                    ) = bbox_from_xyxy(float(x1), float(y1), float(x2), float(y2))
                    
                    det_objects.append(detection)

        det_msg.detections = det_objects
        self.pub_detection_frame.publish(det_msg)


        # Always publish the (possibly annotated) image, even with no detections
        if self.publish_annotated_image and (self._frame_counter % self.publish_annotated_every_n_frames == 0):
            publish_annotated_image_util(self, img, tracked_detections, frame_stamp, self.image_frame_id)
        
        t_end = time.perf_counter()
        self.node.get_logger().debug(f"Detection latency: {(t_end - t_start)*1000:.1f} ms | Active 3D detections: {len(det_objects)}")
