import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,TimerAction
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def get_compressed_topic(base_topic: str, compressed: LaunchConfiguration) -> PythonExpression:
    """
    Dynamically determines if the topic name should have '/compressed' appended based on the value of the 'compressed' launch configuration parameter. This allows for seamless switching between compressed and uncompressed image topics without needing to change topic names in multiple places in the code or launch files.
    """
    
    return PythonExpression([
        "'", base_topic,
        "' + ('/compressed' if '", compressed, "' == 'true' else '')"
    ])

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "vision_pipeline.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value=str(default_config["general"]["sim"]).lower(),
        description=(
            "Whether to use simulation time. Should be true when running in simulation and false when running on the real AUV."
        )
    )
    
    wrapper_stream_arg = DeclareLaunchArgument(
        "wrapper_stream_enabled",
        default_value=str(default_config["general"]["use_wrapper_stream"]).lower(),
        description=(
            "Whether to use the stream server for the ZED wrapper."
        )
    )
    
    compressed_arg = DeclareLaunchArgument(
        "compressed",
        default_value=str(default_config["general"]["compressed"]).lower(),
        description=(
            "Whether input image topics are compressed image message topics. Appends '/compressed' to the end of all image topic names if true."
        )
    )
    
    use_enhance_arg = DeclareLaunchArgument(
        "enhance_images",
        default_value=str(default_config["general"]["enhance_images"]).lower(),
        description=(
            "Whether to run the image enhancement nodes. If false, object detection nodes will subscribe directly to raw camera topics instead of enhanced image topics."
        )
    )
    
    front_model_arg = DeclareLaunchArgument(
        "front_model_relative_path",
        default_value=default_config["object_detection"]["front_cam"]["model_relative_path"],
        description="Path to the front camera object detection model file."
    )
    
    down_model_arg = DeclareLaunchArgument(
        "down_model_relative_path",
        default_value=default_config["object_detection"]["down_cam"]["model_relative_path"],
        description="Path to the down camera object detection model file."
    )

    zed_wrapper_log_type_arg = DeclareLaunchArgument(
        "zed_wrapper_log_type",
        default_value=default_config["general"]["zed_wrapper_log_type"],
        description=(
            "The log type for the zed wrapper node. Can be set to 'log' to enable logging to file, 'screen' to log to console, or 'both' to log to both."
        )
    )

    # zed_wrapper_path = get_package_share_directory("zed_wrapper")
    # zed_real_wrapper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(zed_wrapper_path, "launch", "zed_camera.launch.py")),
    #     launch_arguments={
    #         "camera_model": "zed2i",
    #         "ros_params_override_path": PathJoinSubstitution([vision_dir, "config", "zed_wrapper_real.yaml"]),
    #         "node_log_type": LaunchConfiguration("zed_wrapper_log_type")
    #     }.items(),
    #     condition=UnlessCondition(LaunchConfiguration("sim"))
    # )

    # zed_sim_wrapper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(zed_wrapper_path, "launch", "zed_camera.launch.py")),
    #     launch_arguments={
    #         "camera_model": "zedx",
    #         "sim_mode": "true",
    #         "sim_address": default_config["general"]["sim_ip"],
    #         "sim_port": str(default_config["general"]["sim_port"]),
    #         "ros_params_override_path": PathJoinSubstitution([vision_dir, "config", "zed_wrapper_unity_sim.yaml"]),
    #         "use_sim_time": LaunchConfiguration("sim"),
    #         "node_log_type": LaunchConfiguration("zed_wrapper_log_type")
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration("sim"))
    # )
    
    compressed_launch_config = LaunchConfiguration("compressed")
    
    front_cam_topic = get_compressed_topic(default_config["camera"]["front_cam_topic"], compressed_launch_config)
    down_cam_topic = get_compressed_topic(default_config["camera"]["down_cam_topic"], compressed_launch_config)
    front_enhanced_topic = get_compressed_topic(default_config["image_enhancement"]["front_cam"]["enhanced_topic"], compressed_launch_config)
    down_enhanced_topic = get_compressed_topic(default_config["image_enhancement"]["down_cam"]["enhanced_topic"], compressed_launch_config)
    
    front_cam_enhancement_node = Node (
        package="vision",
        executable="front_image_enhancement.py",
        name="front_image_enhancement_node",
        output = "screen",
        parameters=[
            {
                "input_topic": front_cam_topic,
                "output_topic": front_enhanced_topic,
                "compressed": LaunchConfiguration("compressed"),
                "queue_size": default_config["image_enhancement"]["front_cam"]["queue_size"],
                "use_sim_time": LaunchConfiguration("sim"),
                "log_level": default_config["image_enhancement"]["front_cam"]["log_level"]
            }
        ],
        ros_arguments=["--ros-args", "--log-level", default_config["image_enhancement"]["front_cam"]["log_level"]],
        condition=IfCondition(LaunchConfiguration("enhance_images"))
    )
    
    down_cam_enhancement_node = Node (
        package="vision",
        executable="down_image_enhancement.py",
        name="down_image_enhancement_node",
        output = "screen",
        parameters=[
            {
                "input_topic": down_cam_topic,
                "output_topic": down_enhanced_topic,
                "compressed": LaunchConfiguration("compressed"),
                "queue_size": default_config["image_enhancement"]["down_cam"]["queue_size"],
                "use_sim_time": LaunchConfiguration("sim"),
                "log_level": default_config["image_enhancement"]["down_cam"]["log_level"]
            }
        ],
        ros_arguments=["--ros-args", "--log-level", default_config["image_enhancement"]["down_cam"]["log_level"]],
        condition=IfCondition(LaunchConfiguration("enhance_images"))
    )
    
    # because launch configuration parameters are not evaluated in the launch file but rather passed in to ROS context directly, the ROS manager will evaluate these python expression to determine the actual topic names to remap to for the object detection nodes. If enhance_images is true, remap to the enhanced image topics, otherwise remap to the raw camera topics
    object_detection_front_input = PythonExpression([
        "'", front_enhanced_topic, 
        "' if '", LaunchConfiguration("enhance_images"), "' == 'true' else '", 
        front_cam_topic, "'"
    ])
    object_detection_down_input = PythonExpression([
        "'", down_enhanced_topic, 
        "' if '", LaunchConfiguration("enhance_images"), "' == 'true' else '", 
        down_cam_topic, "'"
    ])
    
    front_detection_node = Node(
        package='vision',
        executable='front_cam_object_detection.py',
        name='front_cam_object_detection',
        parameters=[
            {
                'camera_type': "front_cam",
                'detection_topic': default_config["object_detection"]["front_cam"]["detection_topic"],
                'depth_map_topic': default_config["object_detection"]["front_cam"]["depth_map_topic"],
                'model_path': PathJoinSubstitution([vision_dir, LaunchConfiguration("front_model_relative_path")]),
                'class_names': default_config["object_detection"]["front_cam"]["class_names"],
                'queue_size': default_config["object_detection"]["front_cam"]["queue_size"],
                'publish_annotated_image': default_config["object_detection"]["front_cam"]["publish_annotated_image"],
                'model_type': default_config["object_detection"]["front_cam"]["model_type"],
                'confidence_threshold': default_config["object_detection"]["front_cam"]["confidence_threshold"],
                'use_sim_time': LaunchConfiguration("sim"),
                'compressed': LaunchConfiguration("compressed"),
                'log_level': default_config["object_detection"]["front_cam"]["log_level"],
                "sim": LaunchConfiguration("sim"),
                "stream_ip": default_config["general"]["wrapper_stream_ip"],
                "stream_port": default_config["general"]["wrapper_stream_port"],
            }
        ],
        ros_arguments=["--ros-args", "--log-level", default_config["object_detection"]["front_cam"]["log_level"]]
    )
    
    down_detection_node = Node(
        package='vision',
        executable='down_cam_object_detection.py',
        name='down_cam_object_detection',
        parameters=[
            {
                'camera_type': "down_cam",
                'input_topic': object_detection_down_input,
                'detection_topic': default_config["object_detection"]["down_cam"]["detection_topic"],
                'model_path': PathJoinSubstitution([vision_dir, LaunchConfiguration("down_model_relative_path")]),
                'class_names': default_config["object_detection"]["down_cam"]["class_names"],
                'queue_size': default_config["object_detection"]["down_cam"]["queue_size"],
                'publish_annotated_image': default_config["object_detection"]["down_cam"]["publish_annotated_image"],
                'model_type': default_config["object_detection"]["down_cam"]["model_type"],
                'confidence_threshold': default_config["object_detection"]["down_cam"]["confidence_threshold"],
                'use_sim_time': LaunchConfiguration("sim"),
                'compressed': LaunchConfiguration("compressed"),
                'log_level': default_config["object_detection"]["down_cam"]["log_level"],
            }
        ],
        ros_arguments=["--ros-args", "--log-level", default_config["object_detection"]["down_cam"]["log_level"]]
    )
    
    object_map_node = Node (
        package="vision",
        executable="object_map",
        name="object_map_node",
        parameters=[
            {
                "frame_rate": default_config["object_map"]["frame_rate"],
                "class_labels": default_config["object_detection"]["front_cam"]["class_names"],
                "max_per_class_labels": list(default_config["object_map"]["max_per_class"].keys()),
                "max_per_class_values": list(default_config["object_map"]["max_per_class"].values()),
                "zed_sdk": True,
                "new_object_min_distance_threshold": default_config["object_map"]["new_object_min_distance_threshold"],
                "front_cam_detection_topic": default_config["object_detection"]["front_cam"]["detection_topic"],
                "object_map_topic": default_config["object_map"]["map_topic"],
                "vio_pose_topic": default_config["object_map"]["pose_topic"],
                "confidence_threshold": default_config["object_map"]["confidence_threshold"],
                "zed_depth_confidence_threshold": default_config["object_map"]["zed_depth_confidence_threshold"],
                "max_range": default_config["object_map"]["max_range"],
                "use_stream": default_config["general"]["use_wrapper_stream"],
                "stream_address": default_config["general"]["wrapper_stream_ip"],
                "stream_port": default_config["general"]["wrapper_stream_port"],
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
                "use_sim_time": LaunchConfiguration("sim"),
            },
        ],
        ros_arguments=["--ros-args", "--log-level", default_config["object_map"]["log_level"]]
    )
    # wait 3 seconds before launching node to ensure zed wrapper is able to open and stream on port
    object_map_node = TimerAction(
        period=10.0,
        actions=[object_map_node]
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(sim_arg)
    launch_description.add_action(wrapper_stream_arg)
    launch_description.add_action(compressed_arg)
    launch_description.add_action(use_enhance_arg)
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    # launch_description.add_action(zed_wrapper_log_type_arg)
    # launch_description.add_action(zed_real_wrapper_launch)
    # launch_description.add_action(zed_sim_wrapper_launch)
    launch_description.add_action(front_cam_enhancement_node)
    launch_description.add_action(down_cam_enhancement_node)
    launch_description.add_action(front_detection_node)
    launch_description.add_action(down_detection_node)
    launch_description.add_action(object_map_node)
    
    return launch_description
