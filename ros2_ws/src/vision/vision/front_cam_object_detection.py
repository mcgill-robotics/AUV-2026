#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from vision.object_detection import object_detection_node

def main(args=None):
    rclpy.init(args=args)
    frontcam_detector_node = Node('front_cam_object_detection')
    frontcam_detector = object_detection_node.ObjectDetectorNode(frontcam_detector_node)
    rclpy.spin(frontcam_detector_node)
    frontcam_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()