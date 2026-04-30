

import os

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    # Get dynamic paths to packages
    sensors_pkg_path = get_package_share_directory("sensors")
        
    config_file = os.path.join(sensors_pkg_path, "params", "sensor_health.yaml")
    dvl_velocity_monitor = Node(
                    package='sensors',
                    namespace='sensors',
                    executable='dvl_velocity_monitor.py',
                    name='dvl_velocity_monitor',
                    parameters=[config_file],
                    )

    return LaunchDescription(
        [
            dvl_velocity_monitor,
        ]
    )