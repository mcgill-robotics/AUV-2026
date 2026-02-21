import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get dynamic paths to packages
    xsens_pkg_path = get_package_share_directory("xsens_mti_ros2_driver")
    sensors_pkg_path = get_package_share_directory("sensors")

    # Set Sim condition
    sim_condition = DeclareLaunchArgument(
        "sim", default_value="false", description="Launch sensors in simulation mode"
    )

    # Find other launch files to launch
    xsens_launch_file = os.path.join(
        xsens_pkg_path, "launch", "xsens_mti_node.launch.py"
    )
    launch_Xsens_Driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsens_launch_file),
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    # Serial connection group
    serial_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("sim")),
        actions=[
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="display_micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", "/dev/display", "--baud-rate", "115200"],
            ),
        ],
    )

    state_aggregator = GroupAction(
        actions=[
            Node(
                package="sensors",
                executable="state_aggregator",
                name="state_aggregator",
            )
        ]
    )

    depth_processor_file = os.path.join(
        sensors_pkg_path, "launch", "depth_processor.launch.py"
    )
    depth_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_processor_file)
    )

    imu_processor_file = os.path.join(
        sensors_pkg_path, "launch", "imu_processor.launch.py"
    )
    imu_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_processor_file)
    )

    # Get the sim parameter value
    sim = LaunchConfiguration("sim")

    # Launch :D
    return LaunchDescription(
        [
            sim_condition,
            state_aggregator,
            serial_group,
            launch_Xsens_Driver,
            depth_processor,
            imu_processor,
        ]
    )

    
