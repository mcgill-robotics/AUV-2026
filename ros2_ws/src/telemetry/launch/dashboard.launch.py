# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch
import launch_ros
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Foxglove Bridge for AUV dashboard connections."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='WebSocket port for Foxglove connection'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address to bind Foxglove bridge'
    )

    send_buffer_limit_arg = DeclareLaunchArgument(
        'send_buffer_limit',
        default_value='50000000',
        description='Send buffer limit for Foxglove bridge (50MB)'
    )

    rosbag_profiles_arg = DeclareLaunchArgument(
        'rosbag_profiles_file',
        default_value=[
            launch.substitutions.PathJoinSubstitution([
                launch_ros.substitutions.FindPackageShare('telemetry'),
                'config',
                'rosbag_profiles.yaml'
            ])
        ],
        description='Path to the rosbag profiles YAML file'
    )

    throttle_config_arg = DeclareLaunchArgument(
        'throttle_config_file',
        default_value=[
            launch.substitutions.PathJoinSubstitution([
                launch_ros.substitutions.FindPackageShare('telemetry'),
                'config',
                'throttle_config.yaml'
            ])
        ],
        description='Path to the throttle configuration YAML file'
    )

    # Load the throttle config statically to parse arguments
    telemetry_dir = get_package_share_directory('telemetry')
    throttle_config_path = os.path.join(telemetry_dir, 'config', 'throttle_config.yaml')
    with open(throttle_config_path, 'r') as f:
        throttle_config = yaml.safe_load(f)

    # Foxglove Bridge node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'send_buffer_limit': LaunchConfiguration('send_buffer_limit'),  # Increased buffer for multiple image streams
            'use_compression': True,
            'best_effort_qos_topic_whitelist': [
                '/vision/front_cam/detection_frame/annotated/compressed',
                '/vision/front_cam/detection_frame/annotated/compressed_throttled',
                '/vision/down_cam/detections/annotated/compressed',
                '/vision/down_cam/detections/annotated/compressed_throttled',
                '/vision/front_cam/detection_frame/depth',
                '/vision/front_cam/detection_frame/depth_throttled',
                '/zed/zed_node/rgb/color/rect/image/compressed',
                '/zed/zed_node/depth/depth_registered/compressed'
            ],
        }],
        output='screen'
    )

    dry_test_node = Node(
        package='telemetry',
        executable='drytest_foxglove',
        name='drytest_foxglove',
        output='screen'
    )

    vision_to_foxglove_node = Node(
        package='telemetry',
        executable='vision_to_foxglove',
        name='vision_to_foxglove',
        output='screen'
    )

    setpoint_to_foxglove_node = Node(
        package='telemetry',
        executable='setpoint_to_foxglove',
        name='setpoint_to_foxglove',
        output='screen'
    )

    rosbag_manager_node = Node(
        package='telemetry',
        executable='rosbag_manager_node',
        name='rosbag_manager',
        output='screen',
        parameters=[{
            'profiles_file': LaunchConfiguration('rosbag_profiles_file')
        }]
    )

    front_rgb_throttler_node = Node(
        package='topic_tools',
        executable='throttle',
        name='front_rgb_throttler',
        arguments=[
            throttle_config['front_rgb_throttler']['ros__parameters']['throttle_type'],
            throttle_config['front_rgb_throttler']['ros__parameters']['input_topic'],
            str(throttle_config['front_rgb_throttler']['ros__parameters']['msgs_per_sec']),
            throttle_config['front_rgb_throttler']['ros__parameters']['output_topic']
        ]
    )

    down_rgb_throttler_node = Node(
        package='topic_tools',
        executable='throttle',
        name='down_rgb_throttler',
        arguments=[
            throttle_config['down_rgb_throttler']['ros__parameters']['throttle_type'],
            throttle_config['down_rgb_throttler']['ros__parameters']['input_topic'],
            str(throttle_config['down_rgb_throttler']['ros__parameters']['msgs_per_sec']),
            throttle_config['down_rgb_throttler']['ros__parameters']['output_topic']
        ]
    )

    front_depth_throttler_node = Node(
        package='topic_tools',
        executable='throttle',
        name='front_depth_throttler',
        arguments=[
            throttle_config['front_depth_throttler']['ros__parameters']['throttle_type'],
            throttle_config['front_depth_throttler']['ros__parameters']['input_topic'],
            str(throttle_config['front_depth_throttler']['ros__parameters']['msgs_per_sec']),
            throttle_config['front_depth_throttler']['ros__parameters']['output_topic']
        ]
    )

    return LaunchDescription([
        port_arg,   
        address_arg,
        send_buffer_limit_arg,
        rosbag_profiles_arg,
        throttle_config_arg,
        foxglove_bridge,
        dry_test_node,
        vision_to_foxglove_node,
        setpoint_to_foxglove_node,
        rosbag_manager_node,
        front_rgb_throttler_node,
        down_rgb_throttler_node,
        # front_depth_throttler_node,
    ])
