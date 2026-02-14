# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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

    # Foxglove Bridge node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'send_buffer_limit': 10000000,  # 10MB buffer for images
            'use_compression': True,
        }],
        output='screen'
    )

    dry_test_node = Node(
        package='telemetry',
        executable='drytest_foxglove',
        name='drytest_foxglove',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        foxglove_bridge,
        dry_test_node,
    ])
