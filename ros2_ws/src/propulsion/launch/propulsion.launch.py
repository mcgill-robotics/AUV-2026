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
            'alpha': 45.0,                      # [deg]
            'dx': 0.0,                          # [m]
            'dy': 0.0,                          # [m]
            'disabled_thrusters': []           # put 4 for semi final # List of 1-indexed disabled thrusters (4 = front_right)
        }]
    )

    # --- Serial & power monitor group (Hardware only)
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
            Node(
                package='propulsion',
                executable='jetson_assassin',
                name='jetson_assassin',
                output='screen',
                parameters=[{'enable_host_shutdown': False}]
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
