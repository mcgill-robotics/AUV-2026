# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
        default_value='10000000',
        description='Send buffer limit for Foxglove bridge'
    )

    # Foxglove Bridge node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'send_buffer_limit': LaunchConfiguration('send_buffer_limit'),  # 10MB buffer for images
            'use_compression': True,
            'best_effort_qos_topic_whitelist': [
                '/tf',
                '/vision/front_cam/detections/annotated/compressed',
                '/vision/down_cam/detections/annotated/compressed',
                '/vision/front_cam/detection_frame/depth'
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

    return LaunchDescription([
        port_arg,
        address_arg,
        send_buffer_limit_arg,
        foxglove_bridge,
        dry_test_node,
        vision_to_foxglove_node,
        setpoint_to_foxglove_node,
    ])
