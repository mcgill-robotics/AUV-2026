import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get dynamic paths to packages
    xsens_pkg_path = get_package_share_directory("xsens_mti_ros2_driver")
    sensors_pkg_path = get_package_share_directory("sensors")

    # Set Sim condition
    sim_condition = DeclareLaunchArgument(
        "sim", default_value="false", description="Launch sensors in simulation mode"
    )
    sim = LaunchConfiguration("sim")

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

    state_aggregator_file = os.path.join(
        sensors_pkg_path, "launch", "state_aggregator.launch.py"
    )
    state_aggregator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_aggregator_file),
        launch_arguments={"sim": sim}.items(),
    )

    depth_processor_file = os.path.join(
        sensors_pkg_path, "launch", "depth_processor.launch.py"
    )
    depth_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_processor_file),
        launch_arguments={"sim": sim}.items(),
    )

    imu_processor_file = os.path.join(
        sensors_pkg_path, "launch", "imu_processor.launch.py"
    )
    imu_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_processor_file),
        launch_arguments={"sim": sim}.items(),
    )

    dvl_processor_file = os.path.join(
        sensors_pkg_path, "launch", "dvl_processor.launch.py"
    )
    dvl_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dvl_processor_file),
        launch_arguments={"sim": sim}.items(),
    )

    # Launch :D
    return LaunchDescription(
        [
            sim_condition,
            state_aggregator,
            serial_group,
            launch_Xsens_Driver,
            depth_processor,
            imu_processor,
            dvl_processor,
        ]
    )

    
