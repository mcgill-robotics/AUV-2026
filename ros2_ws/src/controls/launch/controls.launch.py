from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="true for simulation params, false for real-world params",
    )
    sim = LaunchConfiguration("sim")

    default_attitude = PathJoinSubstitution([
        FindPackageShare("controls"), "launch", "attitude_controller.launch.py"
    ])
    default_depth = PathJoinSubstitution([
        FindPackageShare("controls"), "launch", "depth_controller.launch.py"
    ])
    default_superimposer = PathJoinSubstitution([
        FindPackageShare("controls"), "launch", "superimposer.launch.py"
    ])
    default_planar_controller = PathJoinSubstitution([
        FindPackageShare("controls"), "launch", "planar_controller.launch.py"
    ])

    default_navigation = PathJoinSubstitution([
        FindPackageShare("controls"), "launch", "navigation_server.launch.py"
    ])

    declare_attitude = DeclareLaunchArgument(
        "attitude_controller_launch_file",
        default_value=default_attitude,
        description="Path to the attitude controller launch file",
    )
    declare_depth = DeclareLaunchArgument(
        "depth_controller_launch_file",
        default_value=default_depth,
        description="Path to the depth controller launch file",
    )
    declare_superimposer = DeclareLaunchArgument(
        "superimposer_launch_file",
        default_value=default_superimposer,
        description="Path to the superimposer launch file",
    )
    declare_planar_controller = DeclareLaunchArgument(
        "planar_controller_launch_file",
        default_value=default_planar_controller,
        description="Path to the planar controller launch file",
    )
    declare_navigation = DeclareLaunchArgument(
        "navigation_server_launch_file",
        default_value=default_navigation,
        description="Path to the navigation server launch file",
    )

    attitude_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("attitude_controller_launch_file")
        ),
        launch_arguments={"sim": sim}.items(),
    )

    depth_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("depth_controller_launch_file")
        ),
        launch_arguments={"sim": sim}.items(),
    )

    planar_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("planar_controller_launch_file")
        ),
        launch_arguments={"sim": sim}.items(),
    )

    superimposer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("superimposer_launch_file")
        )
    )

    navigation_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            LaunchConfiguration("navigation_server_launch_file")
        )
    )

    return LaunchDescription([
        declare_sim,
        declare_attitude,
        declare_depth,
        declare_superimposer,
        declare_planar_controller,
        declare_navigation,
        attitude_controller_launch,
        depth_controller_launch,
        superimposer_launch,
        planar_controller_launch,
        navigation_server_launch,
    ])
