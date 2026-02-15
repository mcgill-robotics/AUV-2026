from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_pkg_share = FindPackageShare('vision')
    
    front_model_arg = DeclareLaunchArgument(
        "front_model",
        default_value=PathJoinSubstitution([vision_pkg_share, 'models', 'frontcam.pt']),
        description="Path to the front camera object detection model file."
    )
    down_model_arg = DeclareLaunchArgument(
        "down_model",
        default_value=PathJoinSubstitution([vision_pkg_share, 'models', 'downcam.pt']),
        description="Path to the down camera object detection model file."
    )
    front_erhanced_topic_arg = DeclareLaunchArgument(
        "front_enhanced_topic",
        default_value='/vision/front_cam/image_enhanced',
        description="Front enhanced image topic name."
    )
    down_enhanced_topic_arg = DeclareLaunchArgument(
        "down_enhanced_topic",
        default_value='/vision/down_cam/image_enhanced',
        description="Down enhanced image topic name."
    )
    front_detections_topic_arg = DeclareLaunchArgument(
        "front_detections_topic",
        default_value='/vision/front_cam/detections',
        description="Front camera detections topic name."
    )
    down_detections_topic_arg = DeclareLaunchArgument(
        "down_detections_topic",
        default_value='/vision/down_cam/detections',
        description="Down camera detections topic name."
    )
    
    front_detection_node = Node(
        package='vision',
        executable='front_cam_object_detection.py',
        name='front_cam_object_detection',
        parameters=[
            {'class_names': ['gate', 'lane_marker', 'red_pipe', 'white_pipe', 'octagon', 'table', 'bin', 'board', 'shark', 'sawfish']},
            {'model_path': LaunchConfiguration('front_model')},
            {'input_topic': LaunchConfiguration('front_enhanced_topic')},
            {'output_topic': LaunchConfiguration('front_detections_topic')},
            {'queue_size': 10},
        ]
    )
    down_detection_node = Node(
        package='vision',
        executable='down_cam_object_detection.py',
        name='down_cam_object_detection',
        parameters=[
            {'class_names': ['gate', 'octagon_table', 'octagon_top', 'path_marker', 'sawfish', 'shark']},
            {'model_path': LaunchConfiguration('down_model')},
            {'input_topic': LaunchConfiguration('down_enhanced_topic')},
            {'output_topic': LaunchConfiguration('down_detections_topic')},
            {'queue_size': 10},
        ]
    )
    launch_description = LaunchDescription()
    launch_description.add_action(front_model_arg)
    launch_description.add_action(down_model_arg)
    launch_description.add_action(front_erhanced_topic_arg)
    launch_description.add_action(down_enhanced_topic_arg)
    launch_description.add_action(front_detections_topic_arg)
    launch_description.add_action(down_detections_topic_arg)
    
    launch_description.add_action(front_detection_node)
    launch_description.add_action(down_detection_node)
    
    return launch_description