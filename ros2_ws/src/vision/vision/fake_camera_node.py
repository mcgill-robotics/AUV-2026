#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-or-later

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        
        rate = self.get_parameter('publish_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        
        # Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.front_pub = self.create_publisher(
            Image, '/zed2i/zed_node/stereo/image_rect_color', 10)
        self.down_pub = self.create_publisher(
            Image, '/vision/down_cam/image_raw', 10)
        
        # Timer for publishing
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        
        self.get_logger().info(f'Fake Camera Node started')
        self.get_logger().info(f'Publishing at {rate} Hz, {self.width}x{self.height}')
    
    def generate_image(self, camera_name):
        """Generate a synthetic image with timestamp and frame info"""
        # Create a colorful gradient background
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Different colors for different cameras
        if camera_name == 'front':
            # Blue gradient for front camera
            for i in range(self.height):
                color = int(255 * i / self.height)
                image[i, :] = [color, 100, 50]
        else:
            # Green gradient for down camera
            for i in range(self.height):
                color = int(255 * i / self.height)
                image[i, :] = [50, color, 100]
        
        # Add some geometric shapes to make it interesting
        center = (self.width // 2, self.height // 2)
        radius = min(self.width, self.height) // 4
        color = (255, 255, 255)
        cv2.circle(image, center, radius, color, 2)
        
        # Add text overlay
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Camera name
        cv2.putText(image, f'{camera_name.upper()} CAMERA', (20, 50),
                   font, 1.5, (255, 255, 255), 3)
        
        # Timestamp
        cv2.putText(image, timestamp, (20, 100),
                   font, 0.8, (255, 255, 255), 2)
        
        # Frame count
        cv2.putText(image, f'Frame: {self.frame_count}', (20, 150),
                   font, 0.8, (255, 255, 255), 2)
        
        # Add some random noise to simulate real camera
        noise = np.random.randint(0, 20, (self.height, self.width, 3), dtype=np.uint8)
        image = cv2.add(image, noise)
        
        return image
    
    def timer_callback(self):
        """Publish images from both cameras"""
        try:
            # Generate and publish front camera image
            front_image = self.generate_image('front')
            front_msg = self.bridge.cv2_to_imgmsg(front_image, 'bgr8')
            front_msg.header.stamp = self.get_clock().now().to_msg()
            front_msg.header.frame_id = 'zed2i_camera_frame'
            self.front_pub.publish(front_msg)
            
            # Generate and publish down camera image
            down_image = self.generate_image('down')
            down_msg = self.bridge.cv2_to_imgmsg(down_image, 'bgr8')
            down_msg.header.stamp = self.get_clock().now().to_msg()
            down_msg.header.frame_id = 'down_camera_frame'
            self.down_pub.publish(down_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error publishing images: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
