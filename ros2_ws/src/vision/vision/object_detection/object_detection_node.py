#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import os

import torch
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class ObjectDetectorNode():
    def __init__(self, node:Node):

        self.node = node
        self.node.declare_parameter('class_names', Parameter.Type.STRING_ARRAY)
        self.node.declare_parameter('model_path', Parameter.Type.STRING)
        self.node.declare_parameter('input_topic', Parameter.Type.STRING)
        self.node.declare_parameter('output_topic', Parameter.Type.STRING)
        self.node.declare_parameter('queue_size', Parameter.Type.INTEGER)
        self.node.declare_parameter('publish_annotated_image', False)
        self.node.declare_parameter("compressed", Parameter.Type.BOOL)
 

        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.node.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.node.get_parameter('output_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value

        # Load YOLO
        self.bridge = CvBridge()
        if not os.path.exists(model_path):
            self.node.get_logger().error(f"Model path does not exist: {model_path}")
            self.node.get_logger().fatal("Exiting due to missing model.")
            raise FileNotFoundError(f"Model path does not exist: {model_path}")
        self.model = YOLO(model_path)
        self.node.get_logger().info(f"Loaded model from: {model_path}")
        
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
            CompressedImage,
            input_topic,
            self.image_callback,
            qos_profile
        )
        
        self.node.get_logger().info(f"Subscribed to input topic: {input_topic}")

        self.pub_detections = self.node.create_publisher(
            Detection2DArray,
            output_topic,
            queue_size
        )

        self.node.get_logger().info(f"Publishing to output topic: {output_topic}")

        # Publisher for annotated debug image
        if self.publish_annotated_image:
            self.pub_annotated_image = self.node.create_publisher(
                CompressedImage,
                output_topic + "_annotated/compressed", 
                queue_size
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {output_topic}/debug_image")
        
        self.node.get_logger().info(f"{self.node.get_name()} initialized.")

    def image_callback(self, msg: Image | CompressedImage):
        try:
            if self.compressed:
                img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        except Exception as e:
            self.node.get_logger().error(f"cv_bridge failed: {e}")
            return

        try:
            results_list = self.model.predict(img, iou=0.1, agnostic_nms=False, device=self.device, verbose=False)  
        except Exception as e:
            self.node.get_logger().error(f"YOLO failed: {e}")
            return
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        
        det_objects = []

        for results in results_list:
            boxes = results.boxes
            if boxes is None: 
                continue
            try:
                boxes = boxes.cpu().numpy() if torch.cuda.is_available() else boxes.numpy()
            except:
                continue

            for box in boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                if cls_id >= len(self.class_names):
                    continue

                label = self.class_names[cls_id]
                cx, cy, w, h = list(box.xywh[0])

                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding Box
                detection.bbox.center.position.x = float(cx)
                detection.bbox.center.position.y = float(cy)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                # Hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = conf
                
                detection.results.append(hypothesis)
                det_objects.append(detection)

        det_msg.detections = det_objects
        self.pub_detections.publish(det_msg)
        
        # Publish the annotated image
        if self.publish_annotated_image:
            annotated_frame = results_list[0].plot() 
            try:
                annotated_img_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_frame)
                annotated_img_msg.header = msg.header
                self.pub_annotated_image.publish(annotated_img_msg)
            except Exception as e:
                self.node.get_logger().error(f"Failed to publish annotated image: {e}")

        # if len(det_objects) > 0:
        # self.node.get_logger().debug(f"Published {len(det_objects)} detections")
        stamp_time = Time.from_msg(msg.header.stamp)
        current_time = self.node.get_clock().now()
        time_diff = (current_time - stamp_time).nanoseconds / 1e9
        self.node.get_logger().info(f"Detection latency: {time_diff:.9f} seconds")