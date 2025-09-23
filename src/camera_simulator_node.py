#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class CameraSimulatorNode(Node):
    def __init__(self):
        super().__init__('camera_simulator_node')

        # Declare parameters (same as CSI camera node)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('timer_interval', 0.033)  # ~30 FPS
        self.declare_parameter('camera_name', 'csi_camera')

        # Get parameters
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.timer_interval = self.get_parameter('timer_interval').value
        self.camera_name = self.get_parameter('camera_name').value

        # Image counter
        self.image_counter = 1

        # Create publishers (same topics as real camera)
        self.image_pub = self.create_publisher(
            CompressedImage,
            f'{self.camera_name}/image_raw/compressed',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            f'{self.camera_name}/camera_info',
            10
        )

        # Create timer
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

        self.get_logger().info('Camera Simulator node initialized with parameters:')
        self.get_logger().info(f'  Resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'  JPEG Quality: {self.jpeg_quality}')
        self.get_logger().info(f'  Timer Interval: {self.timer_interval}')
        self.get_logger().info(f'  Camera Name: {self.camera_name}')
        self.get_logger().info(f'  Publishing to: {self.camera_name}/image_raw/compressed')

    def generate_test_image(self):
        """Generate a test image with the current counter number"""
        # Create a blank image
        image = np.ones((self.camera_height, self.camera_width, 3), dtype=np.uint8) * 50

        # Add counter text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = min(self.camera_width, self.camera_height) / 200.0
        thickness = max(1, int(font_scale * 2))
        text = f"Frame: {self.image_counter}"

        # Get text size to center it
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = (self.camera_width - text_size[0]) // 2
        text_y = (self.camera_height + text_size[1]) // 2

        # Add main counter text
        cv2.putText(image, text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)

        # Add timestamp
        timestamp_text = f"Time: {time.time():.2f}"
        cv2.putText(image, timestamp_text, (10, 30), font, font_scale * 0.5, (200, 200, 200), 1)

        # Add some visual pattern to make it more interesting
        center_x, center_y = self.camera_width // 2, self.camera_height // 2
        radius = min(self.camera_width, self.camera_height) // 8
        color_intensity = (self.image_counter * 20) % 255
        cv2.circle(image, (center_x, center_y), radius, (color_intensity, 100, 255 - color_intensity), 2)

        return image

    def timer_callback(self):
        try:
            # Generate test image
            frame = self.generate_test_image()

            # Encode to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)

            # Create compressed image message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            msg.format = 'jpeg'
            msg.data = jpeg_data.tobytes()

            # Publish image
            self.image_pub.publish(msg)

            # Create and publish basic camera info
            camera_info = CameraInfo()
            camera_info.header.stamp = msg.header.stamp
            camera_info.header.frame_id = 'camera_optical_frame'
            camera_info.width = self.camera_width
            camera_info.height = self.camera_height

            # Basic camera model (no distortion)
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            camera_info.k = [
                self.camera_width, 0.0, self.camera_width/2.0,
                0.0, self.camera_width, self.camera_height/2.0,
                0.0, 0.0, 1.0
            ]
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info.p = [
                self.camera_width, 0.0, self.camera_width/2.0, 0.0,
                0.0, self.camera_width, self.camera_height/2.0, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            self.camera_info_pub.publish(camera_info)

            # Increment counter
            self.image_counter += 1

            # Log occasionally
            if self.image_counter % 30 == 0:  # Every second at 30fps
                self.get_logger().info(f'Published frame {self.image_counter}')

        except Exception as e:
            self.get_logger().error(f'Error generating test frame: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()