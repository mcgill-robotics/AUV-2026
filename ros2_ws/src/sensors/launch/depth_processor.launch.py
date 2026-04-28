from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
        pkg_share = get_package_share_directory('sensors')
        params = os.path.join(pkg_share, 'params', 'sensors_frames.yaml')
        depth_processor_node = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='depth_processor',
                    name='depth_processor',
                    parameters=[params],
                    )
                ]
            )
        
        return LaunchDescription([
            depth_processor_node
        ])