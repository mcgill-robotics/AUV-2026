from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default paths inside the installed package share directory
    default_attitude = PathJoinSubstitution([
        FindPackageShare('controls'), 'launch', 'attitude_controller.launch.py'
    ])
    default_depth = PathJoinSubstitution([
        FindPackageShare('controls'), 'launch', 'depth_controller.launch.py'
    ])
    default_superimposer = PathJoinSubstitution([
        FindPackageShare('controls'), 'launch', 'superimposer.launch.py'
    ])

    # Declare launch arguments (optional overrides)
    declare_attitude = DeclareLaunchArgument(
        'attitude_controller_launch_file',
        default_value=default_attitude,
        description='Path to the attitude controller launch file'
    )
    declare_depth = DeclareLaunchArgument(
        'depth_controller_launch_file',
        default_value=default_depth,
        description='Path to the depth controller launch file'
    )
    declare_superimposer = DeclareLaunchArgument(
        'superimposer_launch_file',
        default_value=default_superimposer,
        description='Path to the superimposer launch file'
    )

    # Include sub-launch files
    attitude_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('attitude_controller_launch_file')
        )
    )

    depth_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('depth_controller_launch_file')
        )
    )

    superimposer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration('superimposer_launch_file')
        )
    )

    return LaunchDescription([
        declare_attitude,
        declare_depth,
        declare_superimposer,
        attitude_controller_launch,
        depth_controller_launch,
        superimposer_launch,
    ])
