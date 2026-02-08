#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import torch
import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D


class FrontCamDetectorNode(Node):
    def __init__(self):
        super().__init__('front_cam_object_detection')

        self.declare_parameter('class_names')
        self.declare_parameter('model_path')
        self.declare_parameter('input_topic')
        self.declare_parameter('output_topic')
        self.declare_parameter('queue_size')
 

        self.class_names = self.get_parameter('class_names').get_parameter_value().string_array_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

   
        # Load YOLO
        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        if torch.cuda.is_available():
            self.device = "cuda"
            self.model.to("cuda")
            self.get_logger().info("Using CUDA")
        else:
            self.device = "cpu"
            self.get_logger().warn("Using CPU")

        self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            queue_size
        )

        self.pub_detections = self.create_publisher(
            Detection2DArray,
            output_topic,
            queue_size
        )

        self.get_logger().info("FrontCam detector initialized.")

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        try:
            results_list = self.model(img, device=self.device, verbose=False)  
        except Exception as e:
            self.get_logger().error(f"YOLO failed: {e}")
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

        if det_objects:
            self.get_logger().info(f"Published {len(det_objects)} detections")


def main(args=None):
    rclpy.init(args=args)
    node = FrontCamDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()