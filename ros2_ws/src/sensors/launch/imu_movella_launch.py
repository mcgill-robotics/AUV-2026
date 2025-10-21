from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Set Sim condition
    sim_condition = DeclareLaunchArgument(
            'sim', default_value='false', description='no launch if in sim')
    
    # Find other launch files to launch 
    included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/root/AUV-2026/ros2_ws/install/xsens_mti_ros2_driver/share/xsens_mti_ros2_driver/launch', '/xsens_mti_node.launch.py']))
    
    # Get the sim parameter value
    sim = LaunchConfiguration('sim')

    # Launch :D
    return LaunchDescription([
       sim_condition,
       included_launch,
        Node(
            package='sensors',
            executable='imu',
            name='movella_imu',
            parameters=[
                {"frame_override":"imu"},
                {"framerate_pub" : 20  },
                {"max_interval_comparison_messages" : 0.015}
            ],
            condition=UnlessCondition(sim)
            )
        ])
    
