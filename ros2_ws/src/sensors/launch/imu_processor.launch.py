from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
        imu_processor_node = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='imu_processor',
                    name='imu_processor'
                    )
                ]
            )
        
        return LaunchDescription([
            imu_processor_node
        ])