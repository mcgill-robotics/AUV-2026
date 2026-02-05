import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    zed_pkg_dir = get_package_share_directory("zed_wrapper")
    common_camera_config_path = os.path.join(zed_pkg_dir, "config", "common.yaml")
    camera_model = "zed2i"
    specific_camera_config_path = os.path.join(zed_pkg_dir, "config", f"{camera_model}.yaml")
    DeclareLaunchArgument("zed_input_topic", default_value="/zed2/left/image_rect_color", description="Input topic for the IPC consumer node")
    
    zed_input_topic = LaunchConfiguration("zed_input_topic")
    
    container = ComposableNodeContainer(
        name="zed_ipc_test_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Producer: ZED ROS2 Wrapper Node
            ComposableNode(
                # the actual node definitions are in zed components packages
                package="zed_components",
                plugin='stereolabs::ZedCamera',
                name="zed_producer",
                parameters=[
                    common_camera_config_path,
                    specific_camera_config_path,
                    # provide camera model ZED2i
                    {"general.camera_model": camera_model}
                ],
                # enable intra-process communication
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # # Consumer: IPC Test Node
            ComposableNode(
                package="vision",
                plugin="ZED_IPC_Consumer",
                name="zed_ipc_consumer",
                parameters=[
                    {"input_topic": zed_input_topic}
                ],
                # enable intra-process communication
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output="screen",
    )
    return LaunchDescription([container])