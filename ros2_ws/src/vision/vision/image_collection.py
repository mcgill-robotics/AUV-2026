#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-or-later

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import SetBool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import subprocess
from datetime import datetime
import threading
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def load_topics_config():
    """Load centralized topic config from telemetry/config/topics.yaml."""
    try:
        config_path = os.path.join(
            get_package_share_directory('telemetry'), 'config', 'topics.yaml'
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


class ImageCollectionNode(Node):
    def __init__(self):
        super().__init__('image_collection')
        
        # Check if display is available
        self.display_available = self._check_display()
        
        # Load centralized topic config
        topics = load_topics_config()
        default_front = topics['cameras']['front'] if topics else '/zed/zed_node/rgb/color/rect/image/compressed'
        default_down = topics['cameras']['down'] if topics else '/down_cam/image_raw'
        default_depth = topics['cameras']['depth_map'] if topics else '/zed/zed_node/depth/depth_registered'

        # Parameters (defaults from topics.yaml)
        self.declare_parameter('front_cam_data_dir', 'data_front_cam')
        self.declare_parameter('down_cam_data_dir', 'data_down_cam')
        self.declare_parameter('depth_data_dir', 'data_depth')
        self.declare_parameter('front_cam_topic', default_front)
        self.declare_parameter('down_cam_topic', default_down)
        self.declare_parameter('depth_topic', default_depth)
        self.declare_parameter('collection_interval', 2.0)
        
        # Get parameters
        self.front_cam_dir = self.get_parameter('front_cam_data_dir').value
        self.down_cam_dir = self.get_parameter('down_cam_data_dir').value
        self.depth_dir = self.get_parameter('depth_data_dir').value
        front_topic = self.get_parameter('front_cam_topic').value
        down_topic = self.get_parameter('down_cam_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.collection_interval = self.get_parameter('collection_interval').value
        
        # Create directories
        Path(self.front_cam_dir).mkdir(parents=True, exist_ok=True)
        Path(self.down_cam_dir).mkdir(parents=True, exist_ok=True)
        Path(self.depth_dir).mkdir(parents=True, exist_ok=True)
        
        # Bridge for image conversion
        self.bridge = CvBridge()
        
        # Thread-safe image storage with locks
        self.front_image = None
        self.down_image = None
        self.depth_image = None
        self.front_lock = threading.Lock()
        self.down_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        
        # Statistics
        self.front_count = 0
        self.down_count = 0
        self.depth_count = 0
        self.stats_lock = threading.Lock()
        
        # Subscribers
        if 'compressed' in front_topic:
            self.front_sub = self.create_subscription(
                CompressedImage, front_topic, self.front_callback, 10)
        else:
            self.front_sub = self.create_subscription(
                Image, front_topic, self.front_callback, 10)
                
        self.down_sub = self.create_subscription(
            Image, down_topic, self.down_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        
        # Automatic capture state
        self.auto_capture_active = False
        self.auto_timer = None

        # Service controlled capture state
        self.service_capture_timer = None
        self.service_capture_timer_down = None
        self.service_capture_timer_depth = None
        self.create_service(SetBool, '~/toggle_front_collection', self.toggle_front_collection_callback)
        self.create_service(SetBool, '~/toggle_down_collection', self.toggle_down_collection_callback)
        self.create_service(SetBool, '~/toggle_depth_collection', self.toggle_depth_collection_callback)
        
        self.get_logger().info('Image Collection Node initialized')
        self.get_logger().info(f'Front cam dir: {self.front_cam_dir}')
        self.get_logger().info(f'Down cam dir: {self.down_cam_dir}')
        self.get_logger().info(f'Depth dir: {self.depth_dir}')
        if not self.display_available:
            self.get_logger().warn('Display not available - view feature disabled')
        self.get_logger().info('Waiting for camera data...')
    
    def _check_display(self):
        """Check if display is actually available and working"""
        
        # Check DISPLAY variable
        if 'DISPLAY' not in os.environ:
            return False
        
        # Try to actually test X connection
        try:
            result = subprocess.run(
                ['xdpyinfo'], 
                capture_output=True, 
                timeout=1
            )
            return result.returncode == 0
        except:
            return False
    
    def front_callback(self, msg):
        """Callback for front camera images"""
        try:
            if isinstance(msg, CompressedImage):
                # Manual decoding to avoid cv_bridge issues with numpy 2.x
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
            with self.front_lock:
                self.front_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Front camera conversion error: {e}')
    
    def down_callback(self, msg):
        """Callback for down camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.down_lock:
                self.down_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Down camera conversion error: {e}')
    
    def depth_callback(self, msg):
        """Callback for depth map images (32FC1)"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self.depth_lock:
                self.depth_image = depth_image
        except CvBridgeError as e:
            self.get_logger().error(f'Depth map conversion error: {e}')
    
    def save_depth(self, timestamp=None):
        """Save depth map as 16-bit PNG (millimeters)"""
        with self.depth_lock:
            if self.depth_image is None:
                self.get_logger().warn('No depth image available')
                return False
            depth = self.depth_image.copy()
        
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        
        filename = os.path.join(self.depth_dir, f'depth_{timestamp}.png')
        
        try:
            # Convert 32FC1 (meters) to 16-bit unsigned (millimeters)
            # NaN/inf values become 0 (invalid depth)
            depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm = (depth_mm * 1000.0).clip(0, 65535).astype(np.uint16)
            cv2.imwrite(filename, depth_mm)
            with self.stats_lock:
                self.depth_count += 1
                count = self.depth_count
            self.get_logger().info(f'Depth: Saved {filename} (total: {count})')
            return True
        except Exception as e:
            self.get_logger().error(f'Error saving depth: {e}')
            return False

    def save_image(self, camera_type):
        """Save image from specified camera"""
        # Generate shared timestamp for synchronized saves
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        if camera_type == 'front':
            with self.front_lock:
                if self.front_image is None:
                    self.get_logger().warn('No front camera image available')
                    return False
                image = self.front_image.copy()
            output_dir = self.front_cam_dir
        else:  # down
            with self.down_lock:
                if self.down_image is None:
                    self.get_logger().warn('No down camera image available')
                    return False
                image = self.down_image.copy()
            output_dir = self.down_cam_dir
        
        filename = os.path.join(output_dir, f'image_{timestamp}.jpg')
        
        # Save image
        try:
            cv2.imwrite(filename, image)
            with self.stats_lock:
                if camera_type == 'front':
                    self.front_count += 1
                    count = self.front_count
                else:
                    self.down_count += 1
                    count = self.down_count
            self.get_logger().info(f'{camera_type.capitalize()} camera: Saved {filename} (total: {count})')
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')
            return False
        
        # Auto-save depth alongside front camera captures
        if camera_type == 'front':
            self.save_depth(timestamp=timestamp)
        
        return True

    def toggle_front_collection_callback(self, request, response):
        """Service callback to toggle front camera collection"""
        if request.data:
            # Start collection
            if self.service_capture_timer is not None:
                response.success = True
                response.message = 'Front collection already running'
            else:
                self.service_capture_timer = self.create_timer(
                    self.collection_interval, 
                    lambda: self.save_image('front')
                )
                response.success = True
                response.message = f'Started front collection (every {self.collection_interval}s)'
                self.get_logger().info(response.message)
        else:
            # Stop collection
            if self.service_capture_timer is not None:
                self.service_capture_timer.cancel()
                self.service_capture_timer.destroy()
                self.service_capture_timer = None
                response.success = True
                response.message = 'Stopped front collection'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Front collection was not running'
        
        return response
    
    def toggle_down_collection_callback(self, request, response):
        """Service callback to toggle down camera collection"""
        if request.data:
            # Start collection
            if self.service_capture_timer_down is not None:
                response.success = True
                response.message = 'Down collection already running'
            else:
                self.service_capture_timer_down = self.create_timer(
                    self.collection_interval, 
                    lambda: self.save_image('down')
                )
                response.success = True
                response.message = f'Started down collection (every {self.collection_interval}s)'
                self.get_logger().info(response.message)
        else:
            # Stop collection
            if self.service_capture_timer_down is not None:
                self.service_capture_timer_down.cancel()
                self.service_capture_timer_down.destroy()
                self.service_capture_timer_down = None
                response.success = True
                response.message = 'Stopped down collection'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Down collection was not running'
        
        return response
    
    def toggle_depth_collection_callback(self, request, response):
        """Service callback to toggle depth-only collection"""
        if request.data:
            if self.service_capture_timer_depth is not None:
                response.success = True
                response.message = 'Depth collection already running'
            else:
                self.service_capture_timer_depth = self.create_timer(
                    self.collection_interval,
                    lambda: self.save_depth()
                )
                response.success = True
                response.message = f'Started depth collection (every {self.collection_interval}s)'
                self.get_logger().info(response.message)
        else:
            if self.service_capture_timer_depth is not None:
                self.service_capture_timer_depth.cancel()
                self.service_capture_timer_depth.destroy()
                self.service_capture_timer_depth = None
                response.success = True
                response.message = 'Stopped depth collection'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Depth collection was not running'
        
        return response
    
    def show_latest_image(self, camera_type):
        """Display the latest image from specified camera"""
        # Check if display is available
        if not self.display_available:
            self.get_logger().warn('Display not available in this environment')
            self.get_logger().info('View feature will work on Jetson Nano')
            self.get_logger().info(f'Images are saved in: {self.front_cam_dir if camera_type == "front" else self.down_cam_dir}')
            return
        
        if camera_type == 'front':
            with self.front_lock:
                if self.front_image is None:
                    self.get_logger().warn('No front camera image available')
                    return
                image = self.front_image.copy()
            window_name = 'Front Camera - Latest'
        else:
            with self.down_lock:
                if self.down_image is None:
                    self.get_logger().warn('No down camera image available')
                    return
                image = self.down_image.copy()
            window_name = 'Down Camera - Latest'
        
        try:
            # Resize if too large
            h, w = image.shape[:2]
            if w > 1280:
                scale = 1280 / w
                image = cv2.resize(image, (int(w * scale), int(h * scale)))
            
            cv2.imshow(window_name, image)
            cv2.waitKey(0)  # Display for 1 second
            cv2.destroyWindow(window_name)
            cv2.destroyAllWindows()
            
        except Exception as e:
            self.get_logger().error(f'Error displaying image: {e}')
    
    def show_stats(self):
        """Display collection statistics"""
        with self.stats_lock:
            print(f'\n=== Statistics ===')
            print(f'Front camera: {self.front_count} images')
            print(f'Down camera: {self.down_count} images')
            print(f'Depth maps: {self.depth_count} images')
            print(f'Total: {self.front_count + self.down_count + self.depth_count} images')
            print('==================\n')
    
    def stop_auto_capture(self):
        """Stop automatic capture"""
        if self.auto_timer is not None:
            self.auto_timer.cancel()
            self.auto_timer = None
        self.auto_capture_active = False


def user_input_thread(node):
    """Thread for handling user input without blocking ROS"""
    print('\n=== Image Collection Ready ===')
    print('Commands:')
    print('  m - Manual capture')
    print('  a - Automatic capture')
    print('  s - Show statistics')
    print('  q - Quit')
    print('==============================\n')
    
    current_camera = None
    
    while rclpy.ok():
        try:
            cmd = input('Enter command: ').strip().lower()
            
            if cmd == 'q':
                print('Shutting down...')
                node.stop_auto_capture()
                rclpy.shutdown()
                break
            
            elif cmd == 's':
                node.show_stats()
            
            elif cmd == 'm':
                # Manual capture mode
                print('\nManual Capture Mode')
                cam = input('Select camera [f]ront or [d]own: ').strip().lower()
                
                if cam not in ['f', 'd']:
                    print('Invalid camera selection')
                    continue
                
                camera_type = 'front' if cam == 'f' else 'down'
                current_camera = camera_type
                
                print(f'\nManual capture for {camera_type} camera')
                print('Commands: [c]apture, [v]iew latest, [b]ack')
                
                while rclpy.ok():
                    action = input('Action: ').strip().lower()
                    
                    if action == 'c':
                        node.save_image(camera_type)
                    elif action == 'v':
                        node.show_latest_image(camera_type)
                    elif action == 'b':
                        break
                    else:
                        print('Invalid action')
            
            elif cmd == 'a':
                # Automatic capture mode
                print('\nAutomatic Capture Mode')
                cam = input('Select camera [f]ront or [d]own: ').strip().lower()
                
                if cam not in ['f', 'd']:
                    print('Invalid camera selection')
                    continue
                
                camera_type = 'front' if cam == 'f' else 'down'
                
                try:
                    interval = float(input('Interval in seconds (e.g., 2.0): '))
                    if interval <= 0:
                        print('Interval must be positive')
                        continue
                except ValueError:
                    print('Invalid interval')
                    continue
                
                print(f'\nStarting automatic capture every {interval}s')
                print('Press Enter to stop...')
                
                node.auto_capture_active = True
                
                def auto_capture():
                    if node.auto_capture_active and rclpy.ok():
                        node.save_image(camera_type)
                        node.auto_timer = threading.Timer(interval, auto_capture)
                        node.auto_timer.start()
                
                auto_capture()
                input()  # Wait for user to press Enter
                node.stop_auto_capture()
                print('Automatic capture stopped')
            
            else:
                print('Unknown command')
        
        except KeyboardInterrupt:
            print('\nShutting down...')
            node.stop_auto_capture()
            rclpy.shutdown()
            break
        except EOFError:
            # Handle EOF (Ctrl+D) or non-interactive mode
            print('\nInput stream closed. Exiting input thread.')
            break


def main(args=None):
    rclpy.init(args=args)
    node = ImageCollectionNode()
    
    # Start user input thread
    # input_thread = threading.Thread(target=user_input_thread, args=(node,), daemon=True)
    # input_thread.start()
    
    # Spin the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_auto_capture()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
