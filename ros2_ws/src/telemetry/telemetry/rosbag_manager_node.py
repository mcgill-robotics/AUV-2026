#!/usr/bin/env python3
import os
import time
import subprocess
import signal
import rclpy
from rclpy.node import Node
from auv_msgs.srv import RosbagControl
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml

class RosbagManagerNode(Node):
    def __init__(self):
        super().__init__('rosbag_manager')
        self.get_logger().info('Rosbag Manager Node has been started.')

        # Initialize subprocess handle
        self.recorder_process = None

        # Declare parameter for profiles file
        default_config = os.path.join(
            get_package_share_directory('telemetry'),
            'config',
            'rosbag_profiles.yaml'
        )
        self.declare_parameter('profiles_file', default_config)
        config_path = self.get_parameter('profiles_file').get_parameter_value().string_value

        # Load profiles and config
        self.profiles = {}
        self.save_dir = os.path.expanduser('~/AUV-2026/rosbags') # Default
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                if 'save_dir' in config:
                    self.save_dir = os.path.expanduser(config['save_dir'])
                if 'profiles' in config:
                    self.profiles = config['profiles']
        except Exception as e:
            self.get_logger().error(f'Failed to load profiles from {config_path}: {e}')

        # Create service
        self.srv = self.create_service(
            RosbagControl,
            '~/control',
            self.control_callback
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

        # Create directory
        os.makedirs(self.save_dir, exist_ok=True)

    def control_callback(self, request, response):
        action = request.action
        profile_name = request.profile.lower()

        if action == RosbagControl.Request.START_RECORD:
            response.success, response.message = self.start_recording(profile_name, request.bag_name)
        elif action == RosbagControl.Request.STOP_RECORD:
            response.success, response.message = self.stop_recording()
        elif action == RosbagControl.Request.PLAY_BAG: # For future if needed
            response.success = False
            response.message = 'Playing is not yet implemented in this manager node.'
        elif action == RosbagControl.Request.STOP_PLAY: # For future if needed
            response.success = False
            response.message = 'Stop playing is not yet implemented.'
        else:
            response.success = False
            response.message = f'Unknown action: {action}'

        self.get_logger().info(response.message)
        return response

    def start_recording(self, profile_name, bag_name):
        if self.recorder_process is not None and self.recorder_process.poll() is None:
            return False, 'Already recording a bag.'

        # Determine bag name
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        if not bag_name:
            bag_name = f'bag_{timestamp}'
        else:
            bag_name = f'{bag_name}_{timestamp}'
            
        bag_path = os.path.join(self.save_dir, bag_name)

        if os.path.exists(bag_path):
            return False, f'Bag directory already exists: {bag_name}. Please choose a different name.'

        # Determine topics from profile
        if profile_name not in self.profiles:
            return False, f'Profile "{profile_name}" not found.'
        
        profile = self.profiles[profile_name]
        topics = profile.get('topics', [])
        exclude = profile.get('exclude', [])

        if not topics and not exclude:
             return False, f'Profile "{profile_name}" is empty.'

        # Build command
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast', '-o', bag_path]

        # In standard `ros2 bag record`, we can provide explicit topics or regexes
        if ".*" in topics:
            cmd.append('-a') # Record all topics
        else:
            for topic in topics:
                cmd.extend(['-e', topic]) # regex matching
        
        if exclude:
            combined_exclude = "|".join(exclude)
            cmd.extend(['-x', combined_exclude])

        try:
            self.get_logger().info(f'Starting recording with command: {" ".join(cmd)}')
            # Start process in a new session so it doesn't receive sigint from the node directly (we manage it)
            self.recorder_process = subprocess.Popen(
                cmd,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return True, f'Successfully started recording to {bag_path}'
        except Exception as e:
            self.recorder_process = None
            return False, f'Failed to start recording: {e}'

    def stop_recording(self):
        if self.recorder_process is None or self.recorder_process.poll() is not None:
            return False, 'No recording is currently in progress.'
        
        try:
            self.recorder_process.send_signal(signal.SIGINT)
            try:
                self.recorder_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Rosbag record didn't stop cleanly, forcing termination...")
                self.recorder_process.terminate()
                self.recorder_process.wait(timeout=10)
            self.recorder_process = None
            return True, 'Successfully stopped recording.'
        except Exception as e:
            return False, f'Failed to stop recording: {e}'

    def publish_status(self):
        msg = String()
        if self.recorder_process is not None and self.recorder_process.poll() is None:
            msg.data = 'RECORDING'
        else:
            msg.data = 'IDLE'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RosbagManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
