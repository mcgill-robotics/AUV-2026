from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Launch in simulation mode",
    )

    actuator_serial_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("sim")),
        actions=[
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="actuator_micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", "/dev/actuator", "--baud-rate", "115200"],
            ),
        ],
    )

    actuator_interface = Node(
        package="actuators",
        executable="actuator_interface",
        name="actuator_interface",
        output="screen",
    )

    return LaunchDescription([
        sim_arg,
        actuator_serial_group,
        actuator_interface,
    ])
