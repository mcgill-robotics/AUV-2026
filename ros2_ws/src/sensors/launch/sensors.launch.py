import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get dynamic paths to packages
    xsens_pkg_path = get_package_share_directory("xsens_mti_ros2_driver")
    dvl_a50_pkg_path = get_package_share_directory("dvl_a50_serial")
    sensors_pkg_path = get_package_share_directory("sensors")
    params = os.path.join(sensors_pkg_path, "params", "sensors_frames.yaml")

    # Set Sim condition
    sim_condition = DeclareLaunchArgument(
        "sim", default_value="false", description="Launch sensors in simulation mode"
    )
    sim = LaunchConfiguration("sim")
    
    dvl_condition = DeclareLaunchArgument(
        "dvl", default_value="false", description="Launch DVL drivers. THis is disabled by default as DVL might overheat if sensors is launched above water"
    )
    
    # Find other launch files to launch
    xsens_launch_file = os.path.join(
        xsens_pkg_path, "launch", "xsens_mti_node.launch.py"
    )
    launch_Xsens_Driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsens_launch_file),
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    
    dvl_a50_launch_file = os.path.join(
        dvl_a50_pkg_path, "launch", "dvl_a50_serial.launch.py"
    )
    launch_dvl_a50_serial =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dvl_a50_launch_file),
        condition=IfCondition(
            AndSubstitution(
                NotSubstitution(LaunchConfiguration("sim")), 
                LaunchConfiguration("dvl")
            )
        )
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
                parameters=[params, {"use_sim_time": sim}],
            )
        ]
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
            dvl_condition,
            state_aggregator,
            serial_group,
            launch_Xsens_Driver,
            launch_dvl_a50_serial,
            depth_processor,
            imu_processor,
            dvl_processor,
        ]
    )

    
