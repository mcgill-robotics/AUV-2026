from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument('video_device', default_value='/dev/down_cam')
    width_arg        = DeclareLaunchArgument('image_width', default_value='1280')
    height_arg       = DeclareLaunchArgument('image_height', default_value='720')
    fps_arg          = DeclareLaunchArgument('framerate', default_value='30.0')
    pixfmt_arg       = DeclareLaunchArgument('pixel_format', default_value='yuyv') 
    frame_id_arg     = DeclareLaunchArgument('camera_frame_id', default_value='sensors/down_cam')

    cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='down_cam_driver',
        output='screen',
        parameters=[{
            'video_device':    LaunchConfiguration('video_device'),
            'image_width':     LaunchConfiguration('image_width'),
            'image_height':    LaunchConfiguration('image_height'),
            'framerate':       LaunchConfiguration('framerate'),
            'pixel_format':    LaunchConfiguration('pixel_format'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'io_method':       'mmap',
            
        }],
        remappings=[
            ('image_raw',  '/down_cam/image_raw'),
            ('camera_info','/down_cam/camera_info'),
        ],
    )

    return LaunchDescription([
        video_device_arg, width_arg, height_arg, fps_arg, pixfmt_arg, frame_id_arg,
        cam
    ])