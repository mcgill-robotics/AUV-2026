#!/usr/bin/env python3
from datetime import datetime
from functools import partial
import os
import time

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray

from vision.object_detection.utils import (
    build_detection2d_msg,
    get_detections,
    load_model,
    publish_annotated_image_util,
    toggle_collection_callback_util,
)

class DownCamObjectDetectorNode():
    def __init__(self, node: Node):
        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('detection_topic', Parameter.Type.STRING)
        self.node.declare_parameter('input_topic', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter('publish_annotated_every_n_frames', 1)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
        self.node.declare_parameter('collection_dir', '/tmp/down_cam_collection')
        self.node.declare_parameter('collection_interval_seconds', 2.0)

        self.node.declare_parameter("model_detection_threshold", 0.40)

        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        detection_topic = self.node.get_parameter('detection_topic').get_parameter_value().string_value
        input_topic = self.node.get_parameter('input_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.publish_annotated_every_n_frames = self.node.get_parameter('publish_annotated_every_n_frames').get_parameter_value().integer_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.collection_dir = self.node.get_parameter('collection_dir').get_parameter_value().string_value
        self.collection_interval = self.node.get_parameter('collection_interval_seconds').get_parameter_value().double_value
        
        self._collecting = False
        self._last_collection_time = 0.0
        self._frame_counter = 0
        self.node.create_service(SetBool, '~/toggle_collection', partial(toggle_collection_callback_util, self))

        self.conf_threshold = self.node.get_parameter('model_detection_threshold').get_parameter_value().double_value

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
        
        # Load Model using inference-models with TensorRT acceleration
        self.model = load_model(model_path, self.node.get_logger())
        
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
        
        self._frame_counter += 1
        stamp_time = self.node.get_clock().now()
        
        if self._collecting:
            now = time.time()
            if now - self._last_collection_time >= self.collection_interval:
                self._last_collection_time = now
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filepath = os.path.join(self.collection_dir, f'down_{timestamp}.jpg')
                cv2.imwrite(filepath, img)
                self.node.get_logger().debug(f"Saved {filepath}")

        tracked_detections = get_detections(self, img)
        if tracked_detections is not None:
            det_msg = build_detection2d_msg(self, tracked_detections)
            self.pub_detections.publish(det_msg)
            
            if self.publish_annotated_image and (self._frame_counter % self.publish_annotated_every_n_frames == 0):
                publish_annotated_image_util(self, img, tracked_detections, msg.header.stamp, msg.header.frame_id)
            
            current_time = self.node.get_clock().now()
            time_diff = (current_time - stamp_time).nanoseconds / 1e9
            self.node.get_logger().debug(f"Detection latency: {time_diff:.9f} s | Active detections: {len(tracked_detections)}")
