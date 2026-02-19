import yaml
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "object_detection.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
    
    front_enhanced_topic_arg = DeclareLaunchArgument(
        "front_enhanced_topic",
        default_value=default_config["front_cam"]["enhanced_topic"],
        description="Front enhanced image topic name."
    )
    down_enhanced_topic_arg = DeclareLaunchArgument(
        "down_enhanced_topic",
        default_value=default_config["down_cam"]["enhanced_topic"],
        description="Down enhanced image topic name."
    )
    front_detections_topic_arg = DeclareLaunchArgument(
        "front_detections_topic",
        default_value=default_config["front_cam"]["detection_topic"],
        description="Front camera detections topic name."
    )
    down_detections_topic_arg = DeclareLaunchArgument(
        "down_detections_topic",
        default_value=default_config["down_cam"]["detection_topic"],
        description="Down camera detections topic name."
    )   
    front_model_arg = DeclareLaunchArgument(
        "front_model",
        default_value=os.path.join(vision_dir, default_config["front_cam"]["model_relative_path"]),
        description="Path to the front camera object detection model file."
    )
    down_model_arg = DeclareLaunchArgument(
        "down_model",
        default_value=os.path.join(vision_dir, default_config["down_cam"]["model_relative_path"]),
        description="Path to the down camera object detection model file."
    )
    
    front_detection_node = Node(
        package='vision',
        executable='front_cam_object_detection.py',
        name='front_cam_object_detection',
        parameters=[
            {'class_names': default_config["front_cam"]["class_names"]},
            {'model_path': LaunchConfiguration('front_model')},
            {'input_topic': LaunchConfiguration('front_enhanced_topic')},
            {'output_topic': LaunchConfiguration('front_detections_topic')},
            {'queue_size': default_config["front_cam"]["queue_size"]},
        ]
    )
    down_detection_node = Node(
        package='vision',
        executable='down_cam_object_detection.py',
        name='down_cam_object_detection',
        parameters=[
            {'class_names': default_config["down_cam"]["class_names"]},
            {'model_path': LaunchConfiguration('down_model')},
            {'input_topic': LaunchConfiguration('down_enhanced_topic')},
            {'output_topic': LaunchConfiguration('down_detections_topic')},
            {'queue_size': default_config["down_cam"]["queue_size"]},
        ]
    )
    launch_description = LaunchDescription()
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(front_enhanced_topic_arg)
    launch_description.add_action(down_enhanced_topic_arg)
    launch_description.add_action(front_detections_topic_arg)
    launch_description.add_action(down_detections_topic_arg)
    
    launch_description.add_action(front_detection_node)
    launch_description.add_action(down_detection_node)
    
    return launch_description