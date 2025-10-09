#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('dexi_camera')

    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/cam0/image_raw/compressed',
        description='Compressed image topic to subscribe to'
    )

    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='~/dexi_recordings',
        description='Directory to save recorded videos'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='Frames per second for output video'
    )

    codec_arg = DeclareLaunchArgument(
        'codec',
        default_value='mp4v',
        description='Video codec (mp4v, avc1, h264)'
    )

    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='320',
        description='Output video frame width'
    )

    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='240',
        description='Output video frame height'
    )

    record_duration_arg = DeclareLaunchArgument(
        'record_duration',
        default_value='10.0',
        description='Default duration for timed recording (seconds)'
    )

    # Create the video recorder node
    video_recorder_node = Node(
        package='dexi_camera',
        executable='video_recorder_node.py',
        name='video_recorder_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_directory': LaunchConfiguration('output_directory'),
            'fps': LaunchConfiguration('fps'),
            'codec': LaunchConfiguration('codec'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'record_duration': LaunchConfiguration('record_duration')
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        output_directory_arg,
        fps_arg,
        codec_arg,
        frame_width_arg,
        frame_height_arg,
        record_duration_arg,
        video_recorder_node,
    ])
