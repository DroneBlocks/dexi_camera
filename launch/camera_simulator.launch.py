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
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera width in pixels'
    )

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera height in pixels'
    )

    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='75',
        description='JPEG quality (1-100)'
    )

    timer_interval_arg = DeclareLaunchArgument(
        'timer_interval',
        default_value='0.033',  # ~30 FPS
        description='Timer interval in seconds'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='csi_camera',
        description='Camera name for topic namespace'
    )

    # Create the camera simulator node
    camera_simulator_node = Node(
        package='dexi_camera',
        executable='camera_simulator_node.py',
        name='camera_simulator_node',
        output='screen',
        parameters=[{
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'timer_interval': LaunchConfiguration('timer_interval'),
            'camera_name': LaunchConfiguration('camera_name')
        }]
    )

    return LaunchDescription([
        camera_width_arg,
        camera_height_arg,
        jpeg_quality_arg,
        timer_interval_arg,
        camera_name_arg,
        camera_simulator_node,
    ])