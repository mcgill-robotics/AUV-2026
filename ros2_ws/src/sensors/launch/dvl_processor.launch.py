import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
        pkg_share = get_package_share_directory('sensors')
        params = os.path.join(pkg_share, 'params', 'sensors_frames.yaml')
        sim_arg = DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Run sensors nodes with simulation time",
        )
        dvl_processor_node = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='dvl_processor',
                    name='dvl_processor',
                    parameters=[params, {"use_sim_time": LaunchConfiguration("sim")}],
                    output='screen'
                    )
                ]
            )
        
        return LaunchDescription([
            sim_arg,
            dvl_processor_node
        ])
