# launch/thrust_mapper.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments
    sim_arg     = DeclareLaunchArgument('sim', default_value='false')
    record_arg  = DeclareLaunchArgument('record', default_value='true')

    # --- Thruster mapper node (ROS2 Node)
    thrust_mapper = Node(
        package='propulsion',
        executable='thrust_mapper',     # from setup.py console_scripts
        name='thrust_mapper',
        output='screen',
        parameters=[{
            'thruster_PWM_lower_limit': 1228,   # [µs]
            'thruster_PWM_upper_limit': 1768,   # [µs]
            'a': 0.080619,                      # [m]
            'b': 0.226341,                      # [m]
            'c': 0.202572,                      # [m]
            'd': 0.228205,                      # [m]
            'e': 0.004945,                      # [m]
            'alpha': 45.0                       # [deg]
        }]
    )

    # --- Serial connection group 
    # Example below shows micro-ROS agent; replace with whatever we actually have. 
    serial_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=[
            ExecuteProcess(
                cmd=['micro_ros_agent', 'serial', '-D', '/dev/power', '-b', '115200'],
                output='screen'
            ),
        ]
    )

    # --- Recording
    record_bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2', 'bag', 'record',
            '/propulsion/forces',
            '/controls/effort'
        ],
        output='screen'
    )

    return LaunchDescription([
        sim_arg,
        record_arg,
        serial_group,
        thrust_mapper,
        record_bag,
    ])
