from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch arguments
    camera_id = LaunchConfiguration('camera_id')
    camera_name = LaunchConfiguration('camera_name')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    timer_interval = LaunchConfiguration('timer_interval')
    camera_info_url = LaunchConfiguration('camera_info_url')
    
    # Get the package share directory
    pkg_share = get_package_share_directory('dexi_camera')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Camera device ID'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='cam0',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='1280',
            description='Camera width'
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='720',
            description='Camera height'
        ),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='75',
            description='JPEG compression quality (0-100)'
        ),
        DeclareLaunchArgument(
            'timer_interval',
            default_value='0.033',  # 1/30.0
            description='Timer interval in seconds'
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file://' + os.path.join(pkg_share, 'config', 'arducam_12mp_uvc.yaml'),
            description='Path to camera calibration file'
        ),
        
        # Camera node
        Node(
            package='dexi_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_id': camera_id,
                'camera_width': camera_width,
                'camera_height': camera_height,
                'jpeg_quality': jpeg_quality,
                'timer_interval': timer_interval,
                'camera_name': camera_name,
                'camera_info_url': camera_info_url
            }]
        )
    ])