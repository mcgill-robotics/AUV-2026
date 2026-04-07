#!/usr/bin/env python3
import pyzed.sl as sl
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

import os
import cv2
import time
from time import sleep
import supervision as sv
from rfdetr import RFDETRSmall
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision.object_detection.utils import get_detections, build_detection2d_msg, publish_annotated_image_util
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

class FrontCamObjectDetectorNode():
    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('depth_map_topic', Parameter.Type.STRING)        
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter("model_type", "yolo")
        self.node.declare_parameter("confidence_threshold", 0.40)

        # we consume stream_ip and sim properties for zed sdk configuration
        self.node.declare_parameter('sim', Parameter.Type.BOOL)
        self.node.declare_parameter('stream_ip', Parameter.Type.STRING)
        self.node.declare_parameter('stream_port', Parameter.Type.INTEGER)
        self.node.declare_parameter('vio_pose_topic', '/vision/vio_pose')
        self.node.declare_parameter('enable_gate_top_crop', True)
        self.node.declare_parameter('gate_top_crop_ratio', 0.50)

        # Depth sensor state (pressure sensor overrides VIO Z)
        self.sensor_depth = 0.0
        self.depth_sub = self.node.create_subscription(
            Float64,
            '/sensors/depth/z',
            self._depth_callback,
            qos_profile_sensor_data
        )

        self.timer = self.node.create_timer(1.0/30.0, self.run_object_detection)
 
        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        depth_map_topic = self.node.get_parameter('depth_map_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.model_type = self.node.get_parameter('model_type').get_parameter_value().string_value
        self.conf_threshold = self.node.get_parameter('confidence_threshold').get_parameter_value().double_value
        sim = self.node.get_parameter('sim').get_parameter_value().bool_value
        stream_port = self.node.get_parameter('stream_port').get_parameter_value().integer_value
        self.enable_gate_top_crop = self.node.get_parameter('enable_gate_top_crop').get_parameter_value().bool_value
        self.gate_top_crop_ratio = self.node.get_parameter('gate_top_crop_ratio').get_parameter_value().double_value

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
        init_params.camera_resolution = sl.RESOLUTION.SVGA if sim else sl.RESOLUTION.VGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.enable_image_validity_check = True
        init_params.input = sl.InputType()

        if sim:
            stream_ip = self.node.get_parameter('stream_ip').get_parameter_value().string_value
            init_params.set_from_stream(stream_ip, stream_port)

        self.image_buffer = sl.Mat()
        self.depth_buffer = sl.Mat()

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
        pos_param.enable_imu_fusion = True
        pos_param.set_floor_as_origin = False
        err = self.zed.enable_positional_tracking(pos_param)
        if err != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().error(f"Failed to enable positional tracking: {err}")
            self.zed.close()
            exit(1)

        # Configure ZED Object Detection
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = False
        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().error(f"Failed to enable object detection: {err}")
            self.zed.close()
            exit(1)
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
        
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

        # Avoiding CPU Oversubscription
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        
        # Load Model based on model_type
        if self.model_type == 'rfdetr':
            self.node.get_logger().info(f"Loading fine-tuned RF-DETRSmall from: {model_path} on GPU...")
            
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model = RFDETRSmall(pretrain_weights=model_path, device=device)
                
            try:
                self.model.optimize_for_inference()
                self.node.get_logger().info("RF-DETRSmall optimized for inference (TensorRT/Torch Compile)")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to optimize inference: {e}")
        else:
            self.model = YOLO(model_path)
            self.node.get_logger().info(f"Loaded YOLO model from: {model_path}")
            if torch.cuda.is_available():
                self.device = 0
                self.node.get_logger().info("Using CUDA")
            else:
                self.device = "cpu"
                self.node.get_logger().warn("Using CPU")
        
        self.node.get_logger().info(f"Setting QOL queue size to: {queue_size}")
        
        self.pub_detections = self.node.create_publisher(
            Detection3DArray,
            detection_topic,
            queue_size
        )

        vio_pose_topic = self.node.get_parameter('vio_pose_topic').get_parameter_value().string_value
        self.pub_vio_pose = self.node.create_publisher(
            PoseStamped,
            vio_pose_topic,
            10
        )
        self.node.get_logger().info(f"Publishing VIO pose to: {vio_pose_topic}")

        self.pub_depth_map = self.node.create_publisher(
            CompressedImage if self.compressed else Image,
            depth_map_topic,
            queue_size
        )

        self.node.get_logger().info(f"Publishing to output topic: {detection_topic}")

        # Publisher for annotated debug image
        if self.publish_annotated_image:
            publish_topic = detection_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")
        
        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

    def _depth_callback(self, msg):
        self.sensor_depth = msg.data

    def run_object_detection(self):
        result = self.zed.grab(self.runtime_params)
        if result == sl.ERROR_CODE.CORRUPTED_FRAME:
            self.node.get_logger().warn("Corrupted frame detected - skipping")
            return
        if result != sl.ERROR_CODE.SUCCESS:
            self.node.get_logger().warn(f"Zed.grab() failed: {result}, continuing")
            return
        
        # --- ZED Health Checks (ported from zed_detection.cpp) ---
        health = self.zed.get_health_status()
        if health.low_image_quality:
            self.node.get_logger().warn("Low image quality", throttle_duration_sec=5.0)
            return
        if health.low_lighting:
            self.node.get_logger().warn("Low lighting conditions", throttle_duration_sec=5.0)
            return

        t_start = time.perf_counter()
        
        self.zed.retrieve_image(self.image_buffer, sl.VIEW.LEFT)

        # fully handle depth map first (better caching)
        if False:
            self.zed.retrieve_measure(self.depth_buffer, sl.MEASURE.DEPTH)
            depth = cv2.cvtColor(self.depth_buffer.get_data(), cv2.COLOR_RGBA2RGB)
            depth_map = self.bridge.cv2_to_compressed_imgmsg(depth)
            self.pub_depth_map.publish(depth_map)

        img = cv2.cvtColor(self.image_buffer.get_data(), cv2.COLOR_RGBA2RGB)

        tracked_detections = get_detections(self, img)
        if tracked_detections is not None:
            # 1. Ingest into ZED SDK
            custom_boxes = []
            for i in range(len(tracked_detections)):
                x1, y1, x2, y2 = tracked_detections.xyxy[i]
                cls_id = int(tracked_detections.class_id[i])
                if cls_id >= len(self.class_names): continue

                box = sl.CustomBoxObjectData()
                box.probability = float(tracked_detections.confidence[i])
                box.label = cls_id
                box.is_grounded = False

                # --- Gate top crop: only feed the top portion of gate bounding boxes ---
                # The gate's legs extend into noisy stereo territory causing bad depth
                label = self.class_names[cls_id]
                if self.enable_gate_top_crop and label == "gate":
                    h = y2 - y1
                    y2 = y1 + h * self.gate_top_crop_ratio

                # PyZED requires strictly unsigned int arrays, so we must clamp YOLO outputs
                # which occasionally overshoot into negative or beyond-frame pixel space boundaries
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
            self.zed.retrieve_objects(objects, self.obj_runtime_param)

            # Get camera pose rotation matrix for covariance rotation
            cam_pose = sl.Pose()
            tracking_state = self.zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
            if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
                self.node.get_logger().warn(f"VIO tracking not OK: {tracking_state}", throttle_duration_sec=1.0)

            # Extract 3x3 rotation matrix from camera pose
            rotation = cam_pose.get_rotation_matrix()
            R = np.array(rotation.r).reshape(3, 3)
            cam_translation = cam_pose.get_translation().get()
            # Override VIO Z with pressure sensor depth (more accurate than VIO vertical drift)
            cam_translation = np.array([float(cam_translation[0]), float(cam_translation[1]), -self.sensor_depth])
            cam_orientation = cam_pose.get_orientation().get()

            # Publish VIO pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.node.get_clock().now().to_msg()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = float(cam_translation[0])
            pose_msg.pose.position.y = float(cam_translation[1])
            pose_msg.pose.position.z = float(cam_translation[2])
            pose_msg.pose.orientation.w = float(cam_orientation[0])
            pose_msg.pose.orientation.x = float(cam_orientation[1])
            pose_msg.pose.orientation.y = float(cam_orientation[2])
            pose_msg.pose.orientation.z = float(cam_orientation[3])
            self.pub_vio_pose.publish(pose_msg)

            # 3. Build Detection3DArray
            det_msg = Detection3DArray()
            det_msg.header.stamp = self.node.get_clock().now().to_msg()
            det_msg.header.frame_id = "zed_camera_center"
            
            det_objects = []
            for obj in objects.object_list:
                pos_cam = obj.position  # position in CAMERA frame
                if np.isnan(pos_cam).any() or np.isinf(pos_cam).any():
                    continue
                
                # Skip objects behind camera (negative X in camera-local frame)
                if pos_cam[0] < 0:
                    continue

                # Transform to world frame: world_pos = R * cam_pos + t
                # (ported from zed_detection.cpp::transform_to_world)
                cam_pos_vec = np.array([float(pos_cam[0]), float(pos_cam[1]), float(pos_cam[2])])
                world_pos = R @ cam_pos_vec + cam_translation

                detection = Detection3D()
                detection.bbox.center.position.x = float(world_pos[0])
                detection.bbox.center.position.y = float(world_pos[1])
                detection.bbox.center.position.z = float(world_pos[2])
                
                # Covariance: ZED returns 6-element upper-triangular in CAMERA frame
                # We must rotate it to WORLD frame using R * cov_cam * R^T
                # (ported from zed_detection.cpp::get_world_covariance)
                cov = obj.position_covariance
                cov_cam = np.array([
                    [float(cov[0]), float(cov[1]), float(cov[2])],
                    [float(cov[1]), float(cov[3]), float(cov[4])],
                    [float(cov[2]), float(cov[4]), float(cov[5])]
                ])
                cov_world = R @ cov_cam @ R.T

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.class_names[obj.raw_label]
                hypothesis.hypothesis.score = float(obj.confidence) / 100.0  # ZED returns 0-100
                
                # ROS geometry_msgs/PoseWithCovariance expects a 36-element float64 array (6x6)
                # We map the 3x3 world-frame spatial covariance into the top-left of the 6x6 matrix
                ros_cov = [0.0] * 36
                for r in range(3):
                    for c in range(3):
                        ros_cov[r * 6 + c] = float(cov_world[r, c])
                
                hypothesis.pose.pose.position.x = float(world_pos[0])
                hypothesis.pose.pose.position.y = float(world_pos[1])
                hypothesis.pose.pose.position.z = float(world_pos[2])
                hypothesis.pose.covariance = ros_cov

                # Pack camera translation into the Detection3D bbox size fields
                # so object_map can compute distance-from-AUV for pipe filtering
                detection.bbox.size.x = float(cam_translation[0])
                detection.bbox.size.y = float(cam_translation[1])
                detection.bbox.size.z = float(cam_translation[2])

                detection.results = [hypothesis]
                det_objects.append(detection)
                
            det_msg.detections = det_objects
            self.pub_detections.publish(det_msg)
            publish_annotated_image_util(self, img, tracked_detections)
            
            t_end = time.perf_counter()
            self.node.get_logger().debug(f"Detection latency: {(t_end - t_start)*1000:.1f} ms | Active 3D detections: {len(det_objects)}")

