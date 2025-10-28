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
    tag_family = LaunchConfiguration('tag_family')
    tag_size = LaunchConfiguration('tag_size')
    throttle_rate = LaunchConfiguration('throttle_rate')

    # Get the package share directory
    pkg_share = get_package_share_directory('dexi_camera')

    return LaunchDescription([
        # Declare camera arguments
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

        # Declare AprilTag arguments
        DeclareLaunchArgument(
            'tag_family',
            default_value='36h11',
            description='AprilTag family (e.g., 36h11, 25h9, 16h5)'
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.1',
            description='Size of the AprilTag in meters'
        ),
        DeclareLaunchArgument(
            'throttle_rate',
            default_value='2.0',
            description='Rate (Hz) to throttle images for AprilTag detection'
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
        ),

        # Image throttle node for AprilTag detection
        # Throttles compressed images to reduce CPU load
        Node(
            package='topic_tools',
            executable='throttle',
            name='image_throttle_apriltag',
            arguments=['messages', '/cam0/image_raw/compressed', throttle_rate, '/cam0/image_raw/compressed_throttled']
        ),

        # AprilTag node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect/compressed', '/cam0/image_raw/compressed_throttled'),
                ('camera_info', '/cam0/camera_info'),
                ('detections', '/apriltag_detections')
            ],
            parameters=[{
                'image_transport': 'compressed',
                'tag_family': tag_family,
                'tag_size': tag_size,
            }]
        )
    ])
