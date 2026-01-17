from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        # No Launch configuration variables

        depth_controller = Node(
                package='controls',
                executable='depth_controller.py',
                name='depth_controller',
                output='screen',
                parameters=[{
                    'control_loop_hz': 10.0, # hz
                    'KP': 2.0,
                    'KD': 1.0,
                    'KI': 0.5,
                    'I_MAX': 10.0,
                    'net_buoyancy': 50.0, # Newtons. TODO: Get actual value
                }]
        )
        return LaunchDescription([
                depth_controller
        ])