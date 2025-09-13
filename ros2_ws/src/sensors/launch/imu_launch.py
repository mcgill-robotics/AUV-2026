from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import LogInfo


def generate_launch_description():
    included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/root/ros2_ws/install/xsens_mti_ros2_driver/share/xsens_mti_ros2_driver/launch', '/xsens_mti_node.launch.py']))

    return LaunchDescription([
       included_launch,
        Node(
            package='sensors',
            executable='imu',
            name='movella_imu',
            parameters=[
                {"frame_override":"imu"}
            ]
            )
        ])
    
