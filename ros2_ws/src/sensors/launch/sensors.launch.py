from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Set Sim condition
    sim_condition = DeclareLaunchArgument('sim', default_value='false', description='no launch if in sim')
    # Find other launch files to launch
    launch_Xsens_Driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/root/AUV-2026/ros2_ws/install/xsens_mti_ros2_driver/share/xsens_mti_ros2_driver/launch', '/xsens_mti_node.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('sim')))
   
    # Serial connection group 
    serial_group = GroupAction(
            condition=UnlessCondition(LaunchConfiguration('sim')),
            actions=[
                Node(
                    package='micro_ros_agent',
                    executable='micro_ros_agent',
                    name='display_micro_ros_agent',
                    output='screen',
                    arguments=['serial', '--dev', '/dev/display', '--baud-rate', '115200'],
                ),
            ]
     )

    state_aggregator = GroupAction(
            actions=[
                Node(
                    package='sensors',
                    executable='state_aggregator',
                    name='state_aggregator'
                    )
                ]
            )
    
    depth_processor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/root/AUV-2026/ros2_ws/src/sensors/launch', '/depth_processor.launch.py']))
    
    imu_processor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/root/AUV-2026/ros2_ws/src/sensors/launch', '/imu_processor.launch.py']))
    # Get the sim parameter value
    sim = LaunchConfiguration('sim')

    # Launch :D
    return LaunchDescription([
       sim_condition,
       state_aggregator,
       serial_group,
       launch_Xsens_Driver,
       depth_processor,
        imu_processor
        ])
    
