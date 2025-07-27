#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from camera_info_manager import CameraInfoManager
from picamera2 import Picamera2
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import yaml

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('timer_interval', 0.033)  # ~30 FPS
        self.declare_parameter('camera_name', 'csi_camera')
        self.declare_parameter('camera_info_url', '')
        
        # Get parameters
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.timer_interval = self.get_parameter('timer_interval').value
        self.camera_name = self.get_parameter('camera_name').value
        self.camera_info_url = self.get_parameter('camera_info_url').value
        
        # Create publishers
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
        
        # Initialize camera info manager
        self.camera_info_manager = CameraInfoManager(self, self.camera_name)
        # Note: Camera info will be published with default values
        # Calibration can be loaded later if needed
        
        # Initialize picamera2
        self.picam2 = None
        if not self.initialize_camera():
            self.get_logger().error('Failed to initialize picamera2')
            return
        
        # Create timer
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)
        
        self.get_logger().info('CSI Camera node initialized with parameters:')
        self.get_logger().info(f'  Resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'  JPEG Quality: {self.jpeg_quality}')
        self.get_logger().info(f'  Timer Interval: {self.timer_interval}')
        self.get_logger().info(f'  Camera Name: {self.camera_name}')
        self.get_logger().info(f'  Camera Info URL: {self.camera_info_url}')
    
    def initialize_camera(self):
        try:
            self.picam2 = Picamera2()
            
            # Configure camera
            config = self.picam2.create_preview_configuration(
                main={"size": (self.camera_width, self.camera_height)},
                buffer_count=4
            )
            self.picam2.configure(config)
            
            # Start camera
            self.picam2.start()
            
            self.get_logger().info('picamera2 initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize picamera2: {str(e)}')
            return False
    

    
    def timer_callback(self):
        if not self.picam2:
            self.get_logger().warn_once('picamera2 not initialized, skipping frame capture')
            return
        
        try:
            # Capture frame
            frame = self.picam2.capture_array()
            
            if frame is None or frame.size == 0:
                self.get_logger().warn('Captured empty frame, skipping')
                return
            
            # Convert BGR to RGB if needed
            if len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
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
            
            # Publish camera info (always try, like the C++ node)
            # camera_info = self.camera_info_manager.getCameraInfo()
            # camera_info.header.stamp = msg.header.stamp
            # camera_info.header.frame_id = 'camera_optical_frame'
            # self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {str(e)}')
    
    def __del__(self):
        if hasattr(self, 'picam2') and self.picam2:
            try:
                self.picam2.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 