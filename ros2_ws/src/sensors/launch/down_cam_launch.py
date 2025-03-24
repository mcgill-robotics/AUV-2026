from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='down_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/down_cam',
                'image_width': 1280,
                'image_height': 720,
                'pixel_format': 'yuyv',
                'camera_frame_id': '/vision/down_cam',
                'io_method': 'mmap'
            }],
            remappings=[
                ('/down_cam', '/vision/down_cam')
            ]
        )
    ])
