from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="true for simulation params, false for real-world params",
    )
    sim = LaunchConfiguration("sim")

    pkg_share = get_package_share_directory("controls")
    sim_yaml  = os.path.join(pkg_share, "params", "Controller_params_sim.yaml")
    real_yaml = os.path.join(pkg_share, "params", "Controller_params_real.yaml")

    # Depth controller (sim)
    depth_controller_sim = Node(
        package="controls",
        executable="depth_controller.py",
        name="depth_controller",
        output="screen",
        parameters=[sim_yaml],
        condition=IfCondition(sim),
    )

    # Depth controller (real)
    depth_controller_real = Node(
        package="controls",
        executable="depth_controller.py",
        name="depth_controller",
        output="screen",
        parameters=[real_yaml],
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([
        declare_sim,
        depth_controller_sim,
        depth_controller_real,
    ])
