import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition




def generate_launch_description():
        pkg_share = get_package_share_directory('sensors')
        params_real = os.path.join(pkg_share, 'params', 'sensors_frames_real.yaml')
        params_sim = os.path.join(pkg_share, 'params', 'sensors_frames_sim.yaml')
        sim_arg = DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Run sensors nodes with simulation time",
        )

        imu_processor_real = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='imu_processor',
                    name='imu_processor',
                    parameters=[params_real, {"use_sim_time": LaunchConfiguration("sim")}],
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration("sim")),
                    )
                ]
            )
        imu_processor_sim = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='imu_processor',
                    name='imu_processor',
                    parameters=[params_sim, {"use_sim_time": LaunchConfiguration("sim")}],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration("sim")),
                    )
                ]
            )
        
        return LaunchDescription([
            sim_arg,
            imu_processor_real,
            imu_processor_sim
        ])
