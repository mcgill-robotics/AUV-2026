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

def normalize_flip_mode(value) -> str:
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    return str(value).upper()

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "vision_pipeline.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description=(
            "Whether to use simulation time. Should be true when running in simulation and false when running on the real AUV."
        )
    )
    
    compressed_arg = DeclareLaunchArgument(
        "compressed",
        default_value="true",
        description=(
            "Whether input image topics are compressed image message topics. Appends '/compressed' to the end of all image topic names if true."
        )
    )
    
    use_enhance_arg = DeclareLaunchArgument(
        "enhance_images",
        default_value="false",
        description=(
            "Whether to run the image enhancement nodes. If false, object detection nodes will subscribe directly to raw camera topics instead of enhanced image topics."
        )
    )
    
    front_model_arg = DeclareLaunchArgument(
        "front_model_relative_path",
        default_value="",
        description="Optional override path to the front camera object detection model relative to vision package."
    )
    
    down_model_arg = DeclareLaunchArgument(
        "down_model_relative_path",
        default_value="",
        description="Optional override path to the down camera object detection model relative to vision package."
    )

    enable_object_detection_arg = DeclareLaunchArgument(
        "enable_object_detection",
        default_value="true",
        description="Whether to enable object detection inference globally. If false, overrides the individual flags below."
    )

    has_zed_sdk_arg = DeclareLaunchArgument(
        "has_zed_sdk",
        default_value=str(default_config["front_cam_object_detection"]["ros__parameters"]["has_zed_sdk"]).lower(),
        description="Whether the ZED SDK is installed and available. If false, the front camera will subscribe to a ROS image topic instead of using hardware capture."
    )


    
    compressed_launch_config = LaunchConfiguration("compressed")
    front_cam_topic = get_compressed_topic(default_config["front_cam_object_detection"]["ros__parameters"]["image_topic"], compressed_launch_config)
    down_cam_topic = get_compressed_topic(default_config["down_cam_object_detection"]["ros__parameters"]["image_topic"], compressed_launch_config)
    front_enhanced_topic = get_compressed_topic(default_config["front_image_enhancement"]["ros__parameters"]["output_topic"], compressed_launch_config)
    down_enhanced_topic = get_compressed_topic(default_config["down_image_enhancement"]["ros__parameters"]["output_topic"], compressed_launch_config)
    
    front_cam_enhancement_node = Node (
        package="vision",
        executable="front_image_enhancement.py",
        name="front_image_enhancement",
        output = "screen",
        parameters=[
            config_path,
            {
                "input_topic": front_cam_topic,
                "output_topic": front_enhanced_topic,
                "compressed": LaunchConfiguration("compressed"),
                "use_sim_time": LaunchConfiguration("sim"),
            }
        ],
        ros_arguments=["--ros-args", "--log-level", "front_image_enhancement:=" + default_config["front_image_enhancement"]["ros__parameters"]["log_level"]],
        condition=IfCondition(LaunchConfiguration("enhance_images"))
    )
    
    down_cam_enhancement_node = Node (
        package="vision",
        executable="down_image_enhancement.py",
        name="down_image_enhancement",
        output = "screen",
        parameters=[
            config_path,
            {
                "input_topic": down_cam_topic,
                "output_topic": down_enhanced_topic,
                "compressed": LaunchConfiguration("compressed"),
                "use_sim_time": LaunchConfiguration("sim"),
            }
        ],
        ros_arguments=["--ros-args", "--log-level", "down_image_enhancement:=" + default_config["down_image_enhancement"]["ros__parameters"]["log_level"]],
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
    
    enable_front_detection = PythonExpression([
        "'true' if '", LaunchConfiguration("enable_object_detection"), "' == 'true' and '", 
        str(default_config["front_cam_object_detection"]["ros__parameters"]["enable_object_detection"]).lower(), "' == 'true' else 'false'"
    ])
    enable_down_detection = PythonExpression([
        "'true' if '", LaunchConfiguration("enable_object_detection"), "' == 'true' and '", 
        str(default_config["down_cam_object_detection"]["ros__parameters"]["enable_object_detection"]).lower(), "' == 'true' else 'false'"
    ])
    
    front_detection_node = Node(
        package='vision',
        executable='front_cam_object_detection.py',
        name='front_cam_object_detection',
        parameters=[
            config_path,
            {
                "image_topic": front_cam_topic,
                'model_relative_path_override': LaunchConfiguration("front_model_relative_path"),
                'zed_flip_mode': normalize_flip_mode(default_config["front_cam_object_detection"]["ros__parameters"]["zed_flip_mode"]),                
                'use_sim_time': LaunchConfiguration("sim"),
                "sim": LaunchConfiguration("sim"),
                'compressed': LaunchConfiguration("compressed"),
                "enable_object_detection": enable_front_detection,
                "has_zed_sdk": LaunchConfiguration("has_zed_sdk"),
                "depth_estimation.known_planes.floor": default_config["object_map"]["ros__parameters"]["z_axis_locking"]["pool_floor_z"],
            }
        ],
        ros_arguments=["--ros-args", "--log-level", "front_cam_object_detection:=" + default_config["front_cam_object_detection"]["ros__parameters"]["log_level"]]
    )

    # TF2's static_transform_publisher is just an executable and not a ROS node, and thus it cannot read from a ROS config file but instead reads from argv.
    # Therefore we must explicitly pass in the transform arguments as arguments to the launch file.
    auv_to_camera_center_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="auv_to_zed_camera_center_tf",
        arguments=[
            "--x", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["xyz"][0]),
            "--y", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["xyz"][1]),
            "--z", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["xyz"][2]),
            "--roll", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["rpy"][0]),
            "--pitch", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["rpy"][1]),
            "--yaw", str(default_config["front_cam_object_detection"]["ros__parameters"]["auv_to_camera_center"]["rpy"][2]),
            "--frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["auv_frame_id"],
            "--child-frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_frame_id"],
        ],
    )
    
    # TF2's static_transform_publisher is just an executable and not a ROS node, and thus it cannot read from a ROS config file but instead reads from argv.
    # Therefore we must explicitly pass in the transform arguments as arguments to the launch file.
    camera_center_to_detection_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_camera_center_to_left_frame_tf",
        arguments=[
            "--x", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["xyz"][0]),
            "--y", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["xyz"][1]),
            "--z", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["xyz"][2]),
            "--roll", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["rpy"][0]),
            "--pitch", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["rpy"][1]),
            "--yaw", str(default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_to_detection"]["rpy"][2]),
            "--frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["camera_center_frame_id"],
            "--child-frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["detection_frame_id"],
        ],
    )
    
    # TF2's static_transform_publisher is just an executable and not a ROS node, and thus it cannot read from a ROS config file but instead reads from argv.
    # Therefore we must explicitly pass in the transform arguments as arguments to the launch file.
    detection_to_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_left_frame_to_optical_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "-1.57079632679",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["detection_frame_id"],
            "--child-frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["image_frame_id"],
        ],
    )

    # TF2 static transform for the downward facing camera (sensors/down_cam)
    auv_to_down_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="auv_to_down_cam_tf",
        arguments=[
            "--x", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["xyz"][0]),
            "--y", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["xyz"][1]),
            "--z", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["xyz"][2]),
            "--roll", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["rpy"][0]),
            "--pitch", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["rpy"][1]),
            "--yaw", str(default_config["down_cam_object_detection"]["ros__parameters"]["auv_to_down_cam"]["rpy"][2]),
            "--frame-id", default_config["front_cam_object_detection"]["ros__parameters"]["auv_frame_id"],
            "--child-frame-id", default_config["down_cam_object_detection"]["ros__parameters"]["base_frame_id"],
        ],
    )

    # TF2 static transform to create the optical frame where Z points into the scene
    down_cam_to_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="down_cam_to_optical_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "-1.57079632679",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", default_config["down_cam_object_detection"]["ros__parameters"]["base_frame_id"],
            "--child-frame-id", default_config["down_cam_object_detection"]["ros__parameters"]["camera_frame_id"],
        ],
    )

    
    down_detection_node = Node(
        package='vision',
        executable='down_cam_object_detection.py',
        name='down_cam_object_detection',
        parameters=[
            config_path,
            {
                "image_topic": down_cam_topic,
                'model_relative_path_override': LaunchConfiguration("down_model_relative_path"),
                'compressed': LaunchConfiguration("compressed"),
                "sim": LaunchConfiguration("sim"),                
                'use_sim_time': LaunchConfiguration("sim"),
                'enable_object_detection': enable_down_detection,
            }
        ],
        ros_arguments=["--ros-args", "--log-level", "down_cam_object_detection:=" + default_config["down_cam_object_detection"]["ros__parameters"]["log_level"]]
    )
    
    sizes_config = default_config["object_map"]["ros__parameters"].get("object_sizes", {})
    size_labels = list(sizes_config.keys())
    size_x = [float(v[0]) for v in sizes_config.values()]
    size_y = [float(v[1]) for v in sizes_config.values()]
    size_z = [float(v[2]) for v in sizes_config.values()]
    
    object_map_node = Node (
        package="vision",
        executable="object_map",
        name="object_map",
        parameters=[
            config_path,
            {
                "front_cam_detection_frame_topic": default_config["front_cam_object_detection"]["ros__parameters"]["detection_frame_topic"],
                "down_cam_detection_topic": default_config["down_cam_object_detection"]["ros__parameters"]["detection_topic"],
                "down_cam_info_topic": default_config["down_cam_object_detection"]["ros__parameters"]["camera_info_topic"],
                "frame_id_auv": default_config["front_cam_object_detection"]["ros__parameters"]["auv_frame_id"],
                "max_per_class_labels": list(default_config["object_map"]["ros__parameters"]["max_per_class"].keys()),
                "max_per_class_values": list(default_config["object_map"]["ros__parameters"]["max_per_class"].values()),
                "object_size_labels": size_labels,
                "object_size_x": size_x,
                "object_size_y": size_y,
                "object_size_z": size_z,
                "sim": LaunchConfiguration("sim"),
                "use_sim_time": LaunchConfiguration("sim"),
            },
        ],
        ros_arguments=["--ros-args", "--log-level", "object_map:=" + default_config["object_map"]["ros__parameters"]["log_level"]]
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(sim_arg)
    launch_description.add_action(compressed_arg)
    launch_description.add_action(use_enhance_arg)
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(enable_object_detection_arg)
    launch_description.add_action(has_zed_sdk_arg)
    


    launch_description.add_action(auv_to_camera_center_tf)
    launch_description.add_action(camera_center_to_detection_tf)
    launch_description.add_action(detection_to_optical_tf)
    launch_description.add_action(auv_to_down_cam_tf)
    launch_description.add_action(down_cam_to_optical_tf)
    # launch_description.add_action(front_cam_enhancement_node)
    # launch_description.add_action(down_cam_enhancement_node)
    launch_description.add_action(front_detection_node)
    launch_description.add_action(down_detection_node)
    launch_description.add_action(object_map_node)

    return launch_description
