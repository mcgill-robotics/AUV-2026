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
        super().__init__('down_cam_detector')

        self.class_names = ['gate', 'octagon_table', 'octagon_top', 'path_marker', 'sawfish', 'shark']

        # Load YOLO
        self.bridge = CvBridge()
        self.model = YOLO("/best_AUV_sim_down_camera_model.pt")

        if torch.cuda.is_available():
            self.device = "cuda"
            self.model.to("cuda")
            self.get_logger().info("Using CUDA")
        else:
            self.device = "cpu"
            self.get_logger().warn("Using CPU")

        self.create_subscription(
            Image,
            "/vision/down_cam/image_enhanced",
            self.image_callback,
            10
        )

        self.pub_detections = self.create_publisher(
            VisionObjectArray,
            "/vision/down_cam/object_detection",
            10
        )

        self.get_logger().info("DownCam detector initialized.")

    def image_callback(self, msg: Image):
        try:
            # img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            try:
                boxes = boxes.cpu().numpy() if torch.cuda.is_available() else boxes.numpy()
            except:
                continue

            for box in boxes:
                conf = float(box.conf[0]) [1,]

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
                obj.extra_field = ""
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
