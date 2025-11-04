#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch arguments
    front_cam_dir_arg = DeclareLaunchArgument(
        'front_cam_data_dir',
        default_value='data_front_cam',
        description='Directory to save front camera images'
    )
    
    down_cam_dir_arg = DeclareLaunchArgument(
        'down_cam_data_dir',
        default_value='data_down_cam',
        description='Directory to save down camera images'
    )
    
    front_cam_topic_arg = DeclareLaunchArgument(
        'front_cam_topic',
        default_value='/zed2i/zed_node/stereo/image_rect_color',
        description='Front camera topic'
    )
    
    down_cam_topic_arg = DeclareLaunchArgument(
        'down_cam_topic',
        default_value='/vision/down_cam/image_raw',
        description='Down camera topic'
    )
    
    # Image collection node
    image_collection_node = Node(
        package='vision',  # Change to your package name
        executable='image_collection',  
        name='image_collection',
        output='screen',
        parameters=[{
            'front_cam_data_dir': LaunchConfiguration('front_cam_data_dir'),
            'down_cam_data_dir': LaunchConfiguration('down_cam_data_dir'),
            'front_cam_topic': LaunchConfiguration('front_cam_topic'),
            'down_cam_topic': LaunchConfiguration('down_cam_topic'),
        }],
        emulate_tty=True,  # Important for interactive input
    )
    
    return LaunchDescription([
        front_cam_dir_arg,
        down_cam_dir_arg,
        front_cam_topic_arg,
        down_cam_topic_arg,
        image_collection_node,
    ])
