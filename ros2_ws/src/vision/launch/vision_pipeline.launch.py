import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "vision_pipeline.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value=str(default_config["general"]["sim"]),
        description=(
            "Whether to run in simulation mode. In simulation mode, input topics "
            "are assumed to be in compressed image format and use_sim_time is enabled."
        )
    )
    
    front_model_arg = DeclareLaunchArgument(
        "front_model_relative_path",
        default_value=os.path.join(vision_dir, default_config["object_detection"]["front_model_relative_path"]),
        description="Path to the front camera object detection model file."
    )
    down_model_arg = DeclareLaunchArgument(
        "down_model_relative_path",
        default_value=os.path.join(vision_dir, default_config["object_detection"]["down_model_relative_path"]),
        description="Path to the down camera object detection model file."
    )
    # topic names pulled from config directly
    front_cam_topic = default_config["camera"]["front_cam_topic"]
    down_cam_topic = default_config["camera"]["down_cam_topic"]
    front_enhanced_topic = default_config["image_enhancement"]["front_enhanced_topic"]
    down_enhanced_topic = default_config["image_enhancement"]["down_enhanced_topic"]
    front_detections_topic = default_config["object_detection"]["front_detections_topic"]
    down_detections_topic = default_config["object_detection"]["down_detections_topic"]
    object_map_topic = default_config["object_map"]["map_topic"]
    auv_pose_topic = default_config["object_map"]["pose_topic"]
    ie_log_level = default_config["image_enhancement"]["log_level"]
    od_log_level = default_config["object_detection"]["log_level"]
    om_log_level = default_config["object_map"]["log_level"]
    
    enhancement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_dir, "launch", "image_enhancement.launch.py")),
        # use dictionary unpacking to convert from dict to list of tuples for better readability
        launch_arguments={
            "front_cam_topic": front_cam_topic,
            "down_cam_topic": down_cam_topic,
            "front_enhanced_topic": front_enhanced_topic,
            "down_enhanced_topic": down_enhanced_topic,
            "sim": LaunchConfiguration("sim"),
            "use_sim_time": LaunchConfiguration("sim"),
            "log_level": ie_log_level
        }.items()
    )
    
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_dir, "launch", "object_detection.launch.py")),
        launch_arguments={
            "front_enhanced_topic": front_enhanced_topic,
            "down_enhanced_topic": down_enhanced_topic,
            "front_detections_topic": front_detections_topic,
            "down_detections_topic": down_detections_topic,
            "front_model": LaunchConfiguration("front_model_relative_path"),
            "down_model": LaunchConfiguration("down_model_relative_path"),
            "use_sim_time": LaunchConfiguration("sim"),
            "log_level": od_log_level
        }.items()
    )
    
    object_map_node = Node (
        package="vision",
        executable="object_map",
        name="object_map_node",
        parameters=[
            {
                "frame_rate": default_config["object_map"]["frame_rate"],
                "zed_sdk": True,
                "new_object_min_distance_threshold": default_config["object_map"]["new_object_min_distance_threshold"],
                "front_cam_detection_topic": front_detections_topic,
                "object_map_topic": object_map_topic,
                "vio_pose_topic": auv_pose_topic,
                "confidence_threshold": default_config["object_map"]["confidence_threshold"],
                "zed_depth_confidence_threshold": default_config["object_map"]["zed_depth_confidence_threshold"],
                "max_range": default_config["object_map"]["max_range"],
                "use_stream": default_config["object_map"]["use_stream"],
                "stream_ip": default_config["object_map"]["stream_ip"],
                "stream_port": default_config["object_map"]["stream_port"],
                "show_detections": default_config["object_map"]["show_detections"],
                "pool_floor_z": default_config["object_map"]["pool_floor_z"],
                "pool_surface_z": default_config["object_map"]["pool_surface_z"],
                "gating_threshold": default_config["object_map"]["gating_threshold"],
                "min_hits": default_config["object_map"]["min_hits"],
                "max_age": default_config["object_map"]["max_age"],
                "max_position_jump": default_config["object_map"]["max_position_jump"],
                "conf_to_tent_threshold": default_config["object_map"]["conf_to_tent_threshold"],
                "tent_init_buffer": default_config["object_map"]["tent_init_buffer"],
                "sim": LaunchConfiguration("sim"),
                "use_sim_time": LaunchConfiguration("sim")
            }
        ],
        arguments=['--ros-args', '--log-level', om_log_level]
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(sim_arg)
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(enhancement_launch)
    launch_description.add_action(object_detection_launch)
    launch_description.add_action(object_map_node)
    
    return launch_description
