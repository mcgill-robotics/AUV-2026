#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import os
import cv2
import supervision as sv
from rfdetr import RFDETRSmall
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision.object_detection.utils import get_detections, build_detection2d_msg, publish_annotated_image_util

from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class DownCamObjectDetectorNode():
    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('input_topic', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter("model_type", "yolo")
        self.node.declare_parameter("confidence_threshold", 0.40)

        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        input_topic = self.node.get_parameter('input_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.model_type = self.node.get_parameter('model_type').get_parameter_value().string_value
        self.conf_threshold = self.node.get_parameter('confidence_threshold').get_parameter_value().double_value

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
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Often better for high-bandwidth images
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.node.create_subscription(
            input_format,
            input_topic,
            self.run_object_detection,
            qos_profile
        )
        
        self.node.get_logger().info(f"Subscribed to input topic: {input_topic}")

        self.pub_detections = self.node.create_publisher(
            Detection2DArray,
            detection_topic,
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

    def run_object_detection(self, msg):
        try:
            if self.compressed:
                img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        except Exception as e:
            self.node.get_logger().error(f"cv_bridge failed: {e}")
            return
        
        stamp_time = self.node.get_clock().now()
        
        tracked_detections = get_detections(self, img)
        if tracked_detections is not None:
            det_msg = build_detection2d_msg(self, tracked_detections)
            self.pub_detections.publish(det_msg)
            publish_annotated_image_util(self, img, tracked_detections)
            
            current_time = self.node.get_clock().now()
            time_diff = (current_time - stamp_time).nanoseconds / 1e9
            self.node.get_logger().debug(f"Detection latency: {time_diff:.9f} s | Active detections: {len(tracked_detections)}")
