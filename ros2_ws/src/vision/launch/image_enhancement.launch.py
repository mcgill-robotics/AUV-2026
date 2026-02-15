import yaml
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    vision_dir = get_package_share_directory("vision")
    
    config_path = os.path.join(vision_dir, "config", "image_enhancement.yaml")
    with open(config_path, 'r') as f:
        default_config:dict = yaml.safe_load(f)
        
    front_cam_topic_arg = DeclareLaunchArgument(
        'front_cam_topic',
        default_value=default_config["front_cam"]["cam_topic"],
        description='Front camera topic'
    )
    down_cam_topic_arg = DeclareLaunchArgument(
        'down_cam_topic',
        default_value=default_config["down_cam"]["cam_topic"],
        description='Down camera topic'
    )
    front_enhanced_topic_arg = DeclareLaunchArgument(
        'front_enhanced_topic',
        default_value=default_config["front_cam"]["enhanced_topic"],
        description='Front enhanced image topic'
    )
    down_enhanced_topic_arg = DeclareLaunchArgument(
        'down_enhanced_topic',
        default_value=default_config["down_cam"]["enhanced_topic"],
        description='Down enhanced image topic'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value=str(default_config["general"]["sim"]),
        description='Whether running in simulation mode'
    )
    
    front_cam_topic = PythonExpression([
        "'",
        LaunchConfiguration('front_cam_topic'),
        "'",
        " + '/compressed' if ",LaunchConfiguration('sim'), " else ''"
    ])
    down_cam_topic = PythonExpression([
        "'",
        LaunchConfiguration('down_cam_topic'),
        "'",
        " + '/compressed' if ",LaunchConfiguration('sim'), " else ''",
    ])

    front_cam_enhancement_node = Node(
        package='vision',
        executable='front_image_enhancement.py',
        name='front_image_enhancement',
        output='screen',
        parameters=[
            {'input_topic': front_cam_topic},
            {'output_topic': LaunchConfiguration('front_enhanced_topic')},
            {'sim': LaunchConfiguration('sim')},
        ]
    )
    down_cam_enhancement_node = Node(
        package='vision',
        executable='down_image_enhancement.py',
        name='down_image_enhancement',
        output='screen',
        parameters=[
            {'input_topic': down_cam_topic},
            {'output_topic': LaunchConfiguration('down_enhanced_topic')},
            {'sim': LaunchConfiguration('sim')},
        ]
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(front_cam_topic_arg)
    launch_description.add_action(down_cam_topic_arg)
    launch_description.add_action(front_enhanced_topic_arg)
    launch_description.add_action(down_enhanced_topic_arg)
    launch_description.add_action(sim_arg)
    
    launch_description.add_action(front_cam_enhancement_node)
    launch_description.add_action(down_cam_enhancement_node)
    
    
    return launch_description
