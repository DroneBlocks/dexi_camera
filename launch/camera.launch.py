from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_id': 0,
                'camera_width': 1280,
                'camera_height': 720,
                'jpeg_quality': 80,
                'timer_interval': 1.0/30.0,
                'camera_name': 'camera',
                'camera_info_url': '/home/dexi/dexi_ws/install/dexi_camera/share/dexi_camera/camera_calibration.yaml'
            }]
        )
    ]) 