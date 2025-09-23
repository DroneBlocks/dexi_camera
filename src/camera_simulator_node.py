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

        # Create compressed image publisher
        self.image_pub = self.create_publisher(
            CompressedImage,
            f'{self.camera_name}/image_raw/compressed',
            10
        )

        # Create timer for publishing every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)  # Every 2 seconds

        self.get_logger().info('Camera Simulator node initialized with parameters:')
        self.get_logger().info(f'  Resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'  JPEG Quality: {self.jpeg_quality}')
        self.get_logger().info(f'  Camera Name: {self.camera_name}')
        self.get_logger().info(f'  Publishing to: {self.camera_name}/image_raw/compressed every 2 seconds')

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

            # Publish image every 2 seconds
            self.image_pub.publish(msg)

            # Increment counter
            self.image_counter += 1

            # Log each frame
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
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()