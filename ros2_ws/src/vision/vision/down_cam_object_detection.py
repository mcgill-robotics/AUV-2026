#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from vision.object_detection import object_detection_node

def main(args=None):
    rclpy.init(args=args)
    downcam_detector_node = Node('down_cam_object_detection')
    downcam_detector = object_detection_node.ObjectDetectorNode(downcam_detector_node)
    rclpy.spin(downcam_detector_node)
    downcam_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()