import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import SetParameter,Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description=(
            "Whether to run in simulation mode. In simulation mode, input topics "
            "are assumed to be in compressed image format and use_sim_time is enabled."
        )
    )
    
    front_model_arg = DeclareLaunchArgument(
        "front_model_path",
        default_value=os.path.join(vision_dir, "models", "frontcam.pt"),
        description="Path to the front camera object detection model file."
    )
    down_model_arg = DeclareLaunchArgument(
        "down_model_path",
        default_value=os.path.join(vision_dir, "models", "downcam.pt"),
        description="Path to the down camera object detection model file."
    )
    
    # Topic name will be set as constant here for now
    front_cam_topic = "/zed/zed_node/rgb/color/rect/image"
    down_cam_topic = "/down_cam/image_raw"
    front_enhanced_topic = "/vision/front_cam/image_enhanced"
    down_enhanced_topic = "/vision/down_cam/image_enhanced"
    front_detections_topic = "/vision/front_cam/detections"
    down_detections_topic = "/vision/down_cam/detections"
    
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
        }.items(),
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")}
        ]
    )
    return LaunchDescription([
        sim_arg,
        front_model_arg,
        down_model_arg,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("sim")),
        enhancement_launch,
        object_detection_launch,
        object_map_node
    ])