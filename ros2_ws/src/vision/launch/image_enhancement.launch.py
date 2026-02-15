from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    front_cam_topic_arg = DeclareLaunchArgument(
        'front_cam_topic',
        default_value='/zed/zed_node/rgb/color/rect/image',
        description='Front camera topic'
    )
    down_cam_topic_arg = DeclareLaunchArgument(
        'down_cam_topic',
        default_value='/down_cam/image_raw',
        description='Down camera topic'
    )
    front_enhanced_topic_arg = DeclareLaunchArgument(
        'front_enhanced_topic',
        default_value='/vision/front_cam/image_enhanced',
        description='Front enhanced image topic'
    )
    down_enhanced_topic_arg = DeclareLaunchArgument(
        'down_enhanced_topic',
        default_value='/vision/down_cam/image_enhanced',
        description='Down enhanced image topic'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Whether running in simulation mode'
    )
    
    

    
    front_cam_enhancement_node = Node(
        package='vision',
        executable='front_image_enhancement.py',
        name='front_image_enhancement',
        output='screen',
        parameters=[
            {'input_topic': LaunchConfiguration('front_cam_topic')},
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
            {'input_topic': LaunchConfiguration('down_cam_topic')},
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
