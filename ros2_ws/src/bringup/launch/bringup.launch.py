import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim_condition = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Launch AUV in simulation mode",
    )

    sim = LaunchConfiguration("sim")

    vision_condition = DeclareLaunchArgument(
        "vision",
        default_value="true",
        description="Launch vision nodes",
    )

    vision = LaunchConfiguration("vision")

    sensors_pkg_path = get_package_share_directory("sensors")
    propulsion_pkg_path = get_package_share_directory("propulsion")
    controls_pkg_path = get_package_share_directory("controls")
    vision_pkg_path = get_package_share_directory("vision")
    ros_tcp_endpoint_pkg_path = get_package_share_directory("ros_tcp_endpoint")
    telemetry_pkg_path = get_package_share_directory("telemetry")

    sensors_launch_file = os.path.join(sensors_pkg_path, "launch", "sensors.launch.py")
    propulsion_pkg_file = os.path.join(propulsion_pkg_path, "launch", "propulsion.launch.py")
    controls_launch_file = os.path.join(controls_pkg_path, "launch", "controls.launch.py")
    vision_launch_file = os.path.join(vision_pkg_path, "launch", "vision_pipeline.launch.py")
    ros_tcp_endpoint_launch_file = os.path.join(ros_tcp_endpoint_pkg_path, "launch", "endpoint.py")
    telemetry_launch_file = os.path.join(telemetry_pkg_path, "launch", "dashboard.launch.py")

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file),
        launch_arguments={"sim": sim}.items(),
    )

    launch_propulsion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(propulsion_pkg_file),
        launch_arguments={"sim": sim}.items(),
    )

    launch_controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controls_launch_file),
        launch_arguments={"sim": sim}.items(),
    )

    launch_vision = GroupAction(
        condition=IfCondition(vision),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch_file),
                launch_arguments={"sim": sim}.items(),
            )
        ],
    )

    launch_telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(telemetry_launch_file)
    )

    sim_group = GroupAction(
        condition=IfCondition(sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_tcp_endpoint_launch_file)
            )
        ],
    )

    return LaunchDescription([
        sim_condition,
        vision_condition,
        launch_sensors,
        launch_propulsion,
        launch_controls,
        launch_vision,
        launch_telemetry,
        sim_group,
    ])
