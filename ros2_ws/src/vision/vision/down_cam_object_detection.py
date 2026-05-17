#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from vision.object_detection import down_cam_object_detection_node

def main(args=None):
    rclpy.init(args=args)
    downcam_detector_node = Node('down_cam_object_detection')
    try:
        downcam_detector = down_cam_object_detection_node.DownCamObjectDetectorNode(downcam_detector_node)
        rclpy.spin(downcam_detector_node)
    except Exception as exc:
        downcam_detector_node.get_logger().fatal(
            f"down_cam_object_detection failed during startup: {exc}"
        )
        raise
    finally:
        if 'downcam_detector' in dir():
            downcam_detector.destroy()
        downcam_detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()