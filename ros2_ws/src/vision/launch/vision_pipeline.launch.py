import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description=(
            "Whether to run in simulation mode. In simulation mode, input topics "
            "are assumed to be in compressed image format."
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
    
    enhancement_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(vision_dir, "launch", "image_enhancement.launch.xml")),
        # use dictionary unpacking to convert from dict to list of tuples for better readability
        launch_arguments={
            "sim": LaunchConfiguration("sim"),
        }.items()
    )
    
    object_detection_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(vision_dir, "launch", "object_detection.launch.xml")),
        launch_arguments={
            "sim": LaunchConfiguration("sim"),
            "front_model": LaunchConfiguration("front_model_path"),
            "down_model": LaunchConfiguration("down_model_path"),
        }.items()
    )
    return LaunchDescription([
        sim_arg,
        front_model_arg,
        down_model_arg,
        enhancement_launch,
        object_detection_launch,
    ])