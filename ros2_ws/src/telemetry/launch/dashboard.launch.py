# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch
import launch_ros
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Foxglove Bridge for AUV dashboard connections."""
    
    telemetry_config = os.path.join(
        get_package_share_directory('telemetry'),
        'config',
        'vision_foxglove.yaml'
    )
    
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
                '/tf',
                '/vision/front_cam/detection_frame/annotated/compressed',
                '/vision/down_cam/detections/annotated/compressed',
                '/vision/front_cam/detection_frame/depth/compressed',
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
        output='screen',
        parameters=[telemetry_config]
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

    return LaunchDescription([
        port_arg,   
        address_arg,
        send_buffer_limit_arg,
        rosbag_profiles_arg,
        foxglove_bridge,
        dry_test_node,
        vision_to_foxglove_node,
        setpoint_to_foxglove_node,
        rosbag_manager_node,
    ])
