# launch/thrust_mapper.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments
    sim_arg     = DeclareLaunchArgument('sim', default_value='false')
    record_arg  = DeclareLaunchArgument('record', default_value='true')

    pkg_share = get_package_share_directory('propulsion')
    params = os.path.join(pkg_share, 'params', 'propulsion.yaml')

    # --- Thruster mapper node (ROS2 Node)
    thrust_mapper = Node(
        package='propulsion',
        executable='thrust_mapper',     # from setup.py console_scripts
        name='thrust_mapper',
        output='screen',
        parameters=[params]
    )

    # --- Serial connection group 
    # Example below shows micro-ROS agent; replace with whatever we actually have. 
    serial_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=[
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name='power_micro_ros_agent',
                output='screen',
                arguments=["serial", "--dev", "/dev/power", "--baud-rate", "115200"],
            ),
        ]
    )

    # # --- Recording
    # record_bag = ExecuteProcess(
    #     condition=IfCondition(LaunchConfiguration('record')),
    #     cmd=[
    #         'ros2', 'bag', 'record',
    #         '/propulsion/forces',
    #         '/controls/effort'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        sim_arg,
        record_arg,
        serial_group,
        thrust_mapper,
        #record_bag,
    ])
