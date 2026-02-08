# launch/attitude_controller.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_condition = DeclareLaunchArgument(
        "sim", default_value="false", description="true for simulation params, false for real-world params"
    )

    pkg_share = get_package_share_directory('controls')
    sim_yaml  = os.path.join(pkg_share, 'params', 'Controller_params_sim.yaml')
    real_yaml = os.path.join(pkg_share, 'params', 'Controller_params_real.yaml')


   # Launch attitude controller with sim parameters
    attitude_controller_sim = Node(
        package='controls',
        executable='attitude_controller',
        name='attitude_controller',
        output='screen',
        parameters=[sim_yaml],
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # Launch attitude controller with real parameters
    attitude_controller_real = Node(
        package='controls',
        executable='attitude_controller',
        name='attitude_controller',
        output='screen',
        parameters=[real_yaml],
        condition=UnlessCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription([
        sim_condition,
        attitude_controller_sim,
        attitude_controller_real,
    ])
