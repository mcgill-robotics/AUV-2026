from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
        # Include attitude controller launch
        attitude_controller_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([TextSubstitution(text=''),
                LaunchConfiguration('attitude_controller_launch_file', default='ros2_ws/src/controls/launch/attitude_controller.launch.py')])
        )

        # Include depth controller launch
        depth_controller_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([TextSubstitution(text=''),
                LaunchConfiguration('depth_controller_launch_file', default='ros2_ws/src/controls/launch/depth_controller.launch.py')])
        )

        # Include superimposer launch
        superimposer_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([TextSubstitution(text=''),
                LaunchConfiguration('superimposer_launch_file', default='ros2_ws/src/controls/launch/superimposer.launch.py')])
        )
        return LaunchDescription([
                DeclareLaunchArgument(
                        'attitude_controller_launch_file',
                        default_value='ros2_ws/src/controls/launch/attitude_controller.launch.py',
                        description='Path to the attitude controller launch file'
                ),
                DeclareLaunchArgument(
                        'depth_controller_launch_file',
                        default_value='ros2_ws/src/controls/launch/depth_controller.launch.py',
                        description='Path to the depth controller launch file'
                ),
                DeclareLaunchArgument(
                        'superimposer_launch_file',
                        default_value='ros2_ws/src/controls/launch/superimposer.launch.py',
                        description='Path to the superimposer launch file'
                ),
                attitude_controller_launch,
                depth_controller_launch,
                superimposer_launch
        ])