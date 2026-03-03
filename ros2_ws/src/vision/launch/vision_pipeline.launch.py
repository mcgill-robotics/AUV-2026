import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "vision_pipeline.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value=str(default_config["general"]["sim"]),
        description=(
            "Whether to use simulation time. Should be true when running in simulation and false when running on the real AUV."
        )
    )
    
    compressed_arg = DeclareLaunchArgument(
        "compressed",
        default_value=str(default_config["general"]["compressed"]),
        description=(
            "Whether input image topics are compressed image message topics. Appends 'compressed' to the end of expected input topic names if true."
        )
    )
    
    use_enhance_arg = DeclareLaunchArgument(
        "enhance_images",
        default_value=str(default_config["general"]["enhance_images"]),
        description=(
            "Whether to run the image enhancement nodes. If false, object detection nodes will subscribe directly to raw camera topics instead of enhanced image topics."
        )
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
            "compressed": LaunchConfiguration("compressed"),
            "use_sim_time": LaunchConfiguration("sim"),
            "log_level": ie_log_level
        }.items(),
        condition=IfCondition(LaunchConfiguration("enhance_images"))
    )
    
    # because launch configuration parameters are not evaluated in this script but rather passed in to ROS context directly, the ROS manager will evaluate these python expression to determine the actual topic names to remap to for the object detection nodes. If enhance_images is true, remap to the enhanced image topics, otherwise remap to the raw camera topics
    object_detection_front_input = PythonExpression([
        "'", front_enhanced_topic, 
        "' if '", LaunchConfiguration("enhance_images"), "' == 'True' else '", 
        front_cam_topic,"'"," + ('/compressed' if ",LaunchConfiguration('compressed'), " else '')"
    ])
    object_detection_down_input = PythonExpression([
        "'", down_enhanced_topic, 
        "' if '", LaunchConfiguration("enhance_images"), "' == 'True' else '", down_cam_topic, "'"," + ('/compressed' if ",LaunchConfiguration('compressed'), " else '')"
    ])
    
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_dir, "launch", "object_detection.launch.py")),
        launch_arguments={
            "front_enhanced_topic": object_detection_front_input,
            "down_enhanced_topic": object_detection_down_input,
            "front_detections_topic": front_detections_topic,
            "down_detections_topic": down_detections_topic,
            "front_model": PathJoinSubstitution([vision_dir, LaunchConfiguration("front_model_relative_path")]),
            "down_model": PathJoinSubstitution([vision_dir, LaunchConfiguration("down_model_relative_path")]),
            "compressed": LaunchConfiguration("compressed"),
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
                "pool_floor_z": default_config["object_map"]["pool_floor_z"],
                "pool_surface_z": default_config["object_map"]["pool_surface_z"],
                "unique_objects": default_config["object_map"]["unique_objects"],
                "floor_objects": default_config["object_map"]["floor_objects"],
                "surface_objects": default_config["object_map"]["surface_objects"],
                "enable_gate_top_crop": default_config["object_map"]["enable_gate_top_crop"],
                "enable_z_axis_locking": default_config["object_map"]["enable_z_axis_locking"],
                "enable_gate_midpoint_refinement": default_config["object_map"]["enable_gate_midpoint_refinement"],
                "enable_octagon_xy_inheritance": default_config["object_map"]["enable_octagon_xy_inheritance"],
                "gate_top_crop_ratio": default_config["object_map"]["gate_top_crop_ratio"],
                "max_pipe_distance": default_config["object_map"]["max_pipe_distance"],
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
    launch_description.add_action(compressed_arg)
    launch_description.add_action(use_enhance_arg)
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(enhancement_launch)
    launch_description.add_action(object_detection_launch)
    launch_description.add_action(object_map_node)
    
    return launch_description
