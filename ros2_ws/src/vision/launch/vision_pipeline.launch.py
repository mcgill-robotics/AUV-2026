import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import SetParameter,Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(vision_dir, "config", "vision_pipeline.yaml"),
        description="Path to the vision pipeline config file."
    )
    
    config_path = str(LaunchConfiguration("config_file"))
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value=default_config["general"]["sim"],
        description=(
            "Whether to run in simulation mode. In simulation mode, input topics "
            "are assumed to be in compressed image format and use_sim_time is enabled."
        )
    )
    debug_arg = DeclareLaunchArgument(
        "debug_logs",
        default_value=default_config["general"]["debug"],
        description="Whether to enable debug logs for vision nodes."
    )
    
    front_model_arg = DeclareLaunchArgument(
        "front_model_relative_path",
        default_value=default_config["object_detection"]["front_model_relative_path"],
        description="Path to the front camera object detection model file."
    )
    down_model_arg = DeclareLaunchArgument(
        "down_model_relative_path",
        default_value=default_config["object_detection"]["down_model_relative_path"],
        description="Path to the down camera object detection model file."
    )
    
    # topic names pulled from config directly
    front_cam_topic = default_config["camera"]["front_cam_topic"]
    down_cam_topic = default_config["camera"]["down_cam_topic"]
    front_enhanced_topic = default_config["image_enhancement"]["front_enhanced_topic"]
    down_enhanced_topic = default_config["image_enhancement"]["down_enhanced_topic"]
    front_detections_topic = default_config["object_detection"]["front_detections_topic"]
    down_detections_topic = default_config["object_detection"]["down_detections_topic"]
    
    
    enhancement_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(vision_dir, "launch", "image_enhancement.launch.xml")),
        # use dictionary unpacking to convert from dict to list of tuples for better readability
        launch_arguments={
            "front_cam_topic": front_cam_topic,
            "down_cam_topic": down_cam_topic,
            "front_enhanced_topic": front_enhanced_topic,
            "down_enhanced_topic": down_enhanced_topic,
            "sim": LaunchConfiguration("sim"),
            "use_sim_time": LaunchConfiguration("sim"),
        }.items()
    )
    
    object_detection_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(vision_dir, "launch", "object_detection.launch.xml")),
        launch_arguments={
            "front_enhanced_topic": front_enhanced_topic,
            "down_enhanced_topic": down_enhanced_topic,
            "front_detections_topic": front_detections_topic,
            "down_detections_topic": down_detections_topic,
            "front_model": LaunchConfiguration("front_model_path"),
            "down_model": LaunchConfiguration("down_model_path"),
            "use_sim_time": LaunchConfiguration("sim"),
        }.items()
    )
    
    object_map_node = Node (
        package="vision",
        executable="object_map_node",
        name="object_map_node",
        ros_arguments={
            "frame_rate": "30",
            "zed_sdk": "true",
            "new_object_min_distance_threshold": "0.5",
            "front_cam_detection_topic": front_detections_topic,
            "confidence_threshold": "0.5",
            "max_range": "10.0",
            "use_stream": "true",
            "stream_ip": "127.0.0.1",
            "stream_port": "30000",
            "show_detections": "true",
            "debug_logs": LaunchConfiguration("debug_logs"),
            "--log-level": "debug" if LaunchConfiguration("debug_logs") else "info"
        }.items(),
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")}
        ]
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(config_arg)
    launch_description.add_action(sim_arg)
    launch_description.add_action(debug_arg)
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(enhancement_launch)
    launch_description.add_action(object_detection_launch)
    launch_description.add_action(object_map_node)
    
    return launch_description