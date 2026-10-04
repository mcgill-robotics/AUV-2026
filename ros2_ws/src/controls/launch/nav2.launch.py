import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("controls")
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    # Launch arguments
    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation time when true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': sim,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        declare_sim,
        declare_params_file_cmd,
        nav2_bringup_cmd
    ])