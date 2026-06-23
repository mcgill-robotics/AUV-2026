import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
        pkg_share = get_package_share_directory('sensors')
        real_params = os.path.join(pkg_share, 'params', 'sensors_frames_real.yaml')
        sim_params = os.path.join(pkg_share, 'params', 'sensors_frames_sim.yaml')
        sim_arg = DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Run sensors nodes with simulation time",
        )
        dvl_processor_real = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='dvl_processor',
                    name='dvl_processor',
                    parameters=[real_params, {"use_sim_time": LaunchConfiguration("sim")}],
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration("sim")),
                    )
                ]
            )
        dvl_processor_sim = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='dvl_processor',
                    name='dvl_processor',
                    parameters=[sim_params, {"use_sim_time": LaunchConfiguration("sim")}],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration("sim")),
                    )
                ]
            )
        
        return LaunchDescription([
            sim_arg,
            dvl_processor_real,
            dvl_processor_sim
        ])
