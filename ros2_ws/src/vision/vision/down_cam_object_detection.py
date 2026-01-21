import rclpy
from rclpy.node import Node

import torch
import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

from sensor_msgs.msg import Image
from auv_msgs.msg import VisionObject, VisionObjectArray


class DownCamDetectorNode(Node):
    def __init__(self):
        super().__init__('down_cam_object_detection')

        self.declare_parameter('class_names')
        self.declare_parameter('downcam_model_path')
        self.declare_parameter('downcam_input_topic')
        self.declare_parameter('downcam_output_topic')
        self.declare_parameter('queue_size')

        self.class_names = self.get_parameter('class_names').get_parameter_value().string_array_value
        model_path = self.get_parameter('downcam_model_path').get_parameter_value().string_value
        input_topic = self.get_parameter('downcam_input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('downcam_output_topic').get_parameter_value().string_value
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
            VisionObjectArray,
            output_topic,
            queue_size
        )

        self.get_logger().info("DownCam detector initialized.")

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

        det_msg = VisionObjectArray()
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

                obj = VisionObject()
                obj.label = label
                obj.x = float(cx)
                obj.y = float(cy)
                obj.z = 0.0
                obj.theta_z = 0.0
                obj.extra_field = 0.0
                obj.confidence = conf

                det_objects.append(obj)

        det_msg.array = det_objects
        self.pub_detections.publish(det_msg)

        if det_objects:
            self.get_logger().info(f"Published {len(det_objects)} detections")


def main(args=None):
    rclpy.init(args=args)
    node = DownCamDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
