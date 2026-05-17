import os 
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('controls')

    trajectory_planner_sim = Node(
        package='controls',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
    )

    trajectory_planner_real = Node(
        package='controls',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
    )

    return LaunchDescription([
        trajectory_planner_sim,
        trajectory_planner_real,
    ])