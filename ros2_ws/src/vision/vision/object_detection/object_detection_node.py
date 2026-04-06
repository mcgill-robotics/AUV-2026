#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import os
import supervision as sv
from rfdetr import RFDETRSmall

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
        self.node.declare_parameter("model_type", "yolo")
        self.node.declare_parameter("confidence_threshold", 0.40)
 
        self.class_names = list(self.node.get_parameter('class_names').get_parameter_value().string_array_value)
        self.node.get_logger().info(f"Class names: {self.class_names}")
        model_path = self.node.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.node.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.node.get_parameter('output_topic').get_parameter_value().string_value
        queue_size = self.node.get_parameter('queue_size').get_parameter_value().integer_value
        self.publish_annotated_image = self.node.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.compressed = self.node.get_parameter('compressed').get_parameter_value().bool_value
        self.model_type = self.node.get_parameter('model_type').get_parameter_value().string_value
        self.conf_threshold = self.node.get_parameter('confidence_threshold').get_parameter_value().double_value

        input_format = Image
        if self.compressed:
            input_format = CompressedImage
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
            
            # self.tracker = sv.ByteTrack()
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
            publish_topic = output_topic + "/annotated" + ("/compressed" if self.compressed else "")
            self.pub_annotated_image = self.node.create_publisher(
                input_format,
                publish_topic,
                queue_size
            )
            self.node.get_logger().info(f"Publishing annotated debug image to: {publish_topic}")
        
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

        det_msg = Detection2DArray()
        det_msg.header = msg.header
        det_objects = []

        if self.model_type == 'rfdetr':
            try:
                tracked_detections = self.model.predict(img, threshold=self.conf_threshold)
            except Exception as e:
                self.node.get_logger().error(f"RF-DETR predict failed: {e}")
                return
        else:
            try:
                results_list = self.model.predict(img, iou=0.1, agnostic_nms=False, device=self.device, verbose=False)  
                if not results_list:
                    return
                tracked_detections = sv.Detections.from_ultralytics(results_list[0])
            except Exception as e:
                self.node.get_logger().error(f"YOLO failed: {e}")
                return

        for i in range(len(tracked_detections)):
            x1, y1, x2, y2 = tracked_detections.xyxy[i]
            cx = float((x1 + x2) / 2)
            cy = float((y1 + y2) / 2)
            w = float(x2 - x1)
            h = float(y2 - y1)
            conf = float(tracked_detections.confidence[i])
            cls_id = int(tracked_detections.class_id[i])

            if cls_id >= len(self.class_names):
                continue

            label = self.class_names[cls_id]

            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = conf
            
            detection.results = [hypothesis]
            det_objects.append(detection)

        det_msg.detections = det_objects
        self.pub_detections.publish(det_msg)

        if self.publish_annotated_image:
            annotated = img.copy()

            labels = [f"{self.class_names[int(tracked_detections.class_id[i])]} {tracked_detections.confidence[i]:.2f}"
                      for i in range(len(tracked_detections)) if int(tracked_detections.class_id[i]) < len(self.class_names)]

            box_annotator = sv.BoxAnnotator(thickness=2)
            label_annotator = sv.LabelAnnotator(text_thickness=2, text_scale=0.8)

            annotated = box_annotator.annotate(scene=annotated, detections=tracked_detections)
            annotated = label_annotator.annotate(scene=annotated, detections=tracked_detections, labels=labels)

            try:
                if self.compressed:
                    ann_msg = self.bridge.cv2_to_compressed_imgmsg(annotated)
                else:
                    ann_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                ann_msg.header = msg.header
                self.pub_annotated_image.publish(ann_msg)
            except Exception as e:
                self.node.get_logger().error(f"Failed to publish annotated image: {e}")

        stamp_time = Time.from_msg(msg.header.stamp)
        current_time = self.node.get_clock().now()
        time_diff = (current_time - stamp_time).nanoseconds / 1e9
        
        self.node.get_logger().debug(f"Detection latency: {time_diff:.9f} s | Active detections: {len(tracked_detections)}")