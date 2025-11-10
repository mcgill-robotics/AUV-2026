from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    front_cam_topic = DeclareLaunchArgument(
        "front_cam_topic",
        default_value="/sensors/zed/zed_node/stereo/image_rect_color",
        description="Front cam topic"
    )
    
    down_cam_topic = DeclareLaunchArgument(
        "down_cam_topic",
        default_value="/sensors/down_cam/image_raw",
        description="Down cam topic"
    )
    
    front_enhanced_topic = DeclareLaunchArgument(
        "front_enhanced_topic",
        default_value="/vision/front_cam/image_enhanced",
        description="Front cam enhanced topic"
    )

    down_enhanced_topic = DeclareLaunchArgument(
        "down_enhanced_topic",
        default_value="/vision/down_cam/image_enhanced",
        description="Down cam enhanced topic"
    )
    
    # Image collection node
    front_enhance_node = Node(
        package="vision",  # Change to your package name
        executable="front_image_enhancement",  
        name="front_image_enhancement",
        parameters=[{
            "input_topic": LaunchConfiguration("front_cam_topic"),
            "output_topic": LaunchConfiguration("front_enhanced_topic"),
        }],
    )
    
    down_enhance_node = Node(
        package="vision",  # Change to your package name
        executable="down_image_enhancement",  
        name="down_image_enhancement",
        parameters=[{
            "input_topic": LaunchConfiguration("down_cam_topic"),
            "output_topic": LaunchConfiguration("down_enhanced_topic"),
        }],
    )
    

    return LaunchDescription([
            front_cam_topic, 
            down_cam_topic,
            front_enhanced_topic, 
            down_enhanced_topic,
            front_enhance_node,
            down_enhance_node
    ])