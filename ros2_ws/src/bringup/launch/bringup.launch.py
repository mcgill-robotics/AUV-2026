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

    enable_object_detection_arg = DeclareLaunchArgument(
        "enable_object_detection",
        default_value="true",
        description="Enable object detection inference in the vision pipeline globally",
    )
    
    controls_condition = DeclareLaunchArgument(
        "controls",
        default_value="true",
        description="Launch controls nodes",
    )
        
    sensors_condition = DeclareLaunchArgument(
        "sensors",
        default_value="true",
        description="Launch sensors nodes",
    )
    
    propulsion_condition = DeclareLaunchArgument(
        "propulsion",
        default_value="true",
        description="Launch propulsion nodes",
    )

    planner_condition = DeclareLaunchArgument(
        "planner", 
        default_value="false", 
        description="Launch the behaviour tree planner"
    )

    vision = LaunchConfiguration("vision")
    enable_object_detection = LaunchConfiguration("enable_object_detection")
    controls = LaunchConfiguration("controls")
    sensors = LaunchConfiguration("sensors")
    propulsion = LaunchConfiguration("propulsion")
    planner = LaunchConfiguration("planner")

    sensors_pkg_path = get_package_share_directory("sensors")
    propulsion_pkg_path = get_package_share_directory("propulsion")
    controls_pkg_path = get_package_share_directory("controls")
    vision_pkg_path = get_package_share_directory("vision")
    planner_pkg_path = get_package_share_directory("planner")
    ros_tcp_endpoint_pkg_path = get_package_share_directory("ros_tcp_endpoint")
    telemetry_pkg_path = get_package_share_directory("telemetry")

    sensors_launch_file = os.path.join(sensors_pkg_path, "launch", "sensors.launch.py")
    propulsion_pkg_file = os.path.join(propulsion_pkg_path, "launch", "propulsion.launch.py")
    controls_launch_file = os.path.join(controls_pkg_path, "launch", "controls.launch.py")
    vision_launch_file = os.path.join(vision_pkg_path, "launch", "vision_pipeline.launch.py")
    planner_launch_file = os.path.join(planner_pkg_path, "launch", "planner.launch.py")
    ros_tcp_endpoint_launch_file = os.path.join(ros_tcp_endpoint_pkg_path, "launch", "endpoint.py")
    telemetry_launch_file = os.path.join(telemetry_pkg_path, "launch", "dashboard.launch.py")

    launch_sensors = GroupAction(
        condition=IfCondition(sensors),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch_file),
                launch_arguments={"sim": sim}.items(),
            )
        ],
    )

    launch_propulsion = GroupAction(
        condition=IfCondition(propulsion),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(propulsion_pkg_file),
                launch_arguments={"sim": sim}.items(),
            )
        ],
    )

    launch_controls = GroupAction(
        condition=IfCondition(controls),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controls_launch_file),
                launch_arguments={"sim": sim}.items(),
            )
        ],
    )

    launch_vision = GroupAction(
        condition=IfCondition(vision),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch_file),
                launch_arguments={
                    "sim": sim,
                    "enable_object_detection": enable_object_detection
                }.items(),
            )
        ],
    )


    launch_planner = GroupAction(
        condition=IfCondition(planner),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planner_launch_file),
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
        enable_object_detection_arg,
        controls_condition,
        sensors_condition,
        propulsion_condition,
        planner_condition,
        launch_sensors,
        launch_propulsion,
        launch_controls,
        launch_vision,
        launch_telemetry,
        launch_planner,
        sim_group,
    ])
