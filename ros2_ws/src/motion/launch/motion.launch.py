import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motion'), 'config', 'motion.yaml'
    )

    return LaunchDescription([
        Node(
            package='motion',
            executable='navigation_server',
            name='navigation_server',
            parameters=[config],
            output='screen',
        ),
    ])
