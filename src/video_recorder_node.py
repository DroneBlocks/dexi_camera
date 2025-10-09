#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
import cv2
import numpy as np
from datetime import datetime
import threading
import os


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/cam0/image_raw/compressed')
        self.declare_parameter('output_directory', '/home/dexi/dexi_recordings')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('codec', 'avc1')  # avc1 (H.264) for better Mac/QuickTime compatibility
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.output_directory = self.get_parameter('output_directory').value
        self.fps = self.get_parameter('fps').value
        self.codec = self.get_parameter('codec').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value

        # Create output directory if it doesn't exist
        os.makedirs(self.output_directory, exist_ok=True)

        # Recording state
        self.is_recording = False
        self.video_writer = None
        self.current_filename = None
        self.recording_timer = None
        self.lock = threading.Lock()
        self.frame_count = 0

        # Subscribe to compressed image topic
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        # Create services
        self.start_service = self.create_service(
            Trigger,
            '~/start_recording',
            self.start_recording_callback
        )

        self.stop_service = self.create_service(
            Trigger,
            '~/stop_recording',
            self.stop_recording_callback
        )

        # Service for timed recording (uses SetBool with duration as custom parameter)
        # We'll create a custom service for this
        from example_interfaces.srv import SetBool as RecordDuration
        self.timed_service = self.create_service(
            Trigger,
            '~/record_timed',
            self.record_timed_callback
        )

        # Parameter for timed recording duration
        self.declare_parameter('record_duration', 10.0)

        self.get_logger().info('Video Recorder Node initialized')
        self.get_logger().info(f'  Subscribed to: {self.image_topic}')
        self.get_logger().info(f'  Output directory: {self.output_directory}')
        self.get_logger().info(f'  FPS: {self.fps}')
        self.get_logger().info(f'  Codec: {self.codec}')
        self.get_logger().info(f'  Resolution: {self.frame_width}x{self.frame_height}')
        self.get_logger().info('Services available:')
        self.get_logger().info(f'  - ~/start_recording')
        self.get_logger().info(f'  - ~/stop_recording')
        self.get_logger().info(f'  - ~/record_timed (uses record_duration parameter)')

    def image_callback(self, msg):
        """Process incoming compressed images and write to video if recording."""
        if not self.is_recording:
            return

        try:
            with self.lock:
                if self.video_writer is None:
                    return

                # Decode compressed image
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    self.get_logger().warn('Failed to decode image frame')
                    return

                # Resize frame if needed
                if frame.shape[1] != self.frame_width or frame.shape[0] != self.frame_height:
                    frame = cv2.resize(frame, (self.frame_width, self.frame_height))

                # Write frame to video
                self.video_writer.write(frame)
                self.frame_count += 1

                # Log every 30 frames
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Recorded {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def start_recording_callback(self, request, response):
        """Start recording video."""
        try:
            with self.lock:
                if self.is_recording:
                    response.success = False
                    response.message = 'Already recording'
                    return response

                # Generate filename with timestamp
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.current_filename = os.path.join(
                    self.output_directory,
                    f'recording_{timestamp}.mp4'
                )

                # Reset frame counter
                self.frame_count = 0

                # Create video writer
                fourcc = cv2.VideoWriter_fourcc(*self.codec)
                self.video_writer = cv2.VideoWriter(
                    self.current_filename,
                    fourcc,
                    int(self.fps),  # Convert to int for VideoWriter
                    (self.frame_width, self.frame_height)
                )

                if not self.video_writer.isOpened():
                    response.success = False
                    response.message = 'Failed to open video writer'
                    self.video_writer = None
                    return response

                self.is_recording = True
                self.get_logger().info(f'VideoWriter opened successfully. Codec: {self.codec}, FPS: {int(self.fps)}, Size: {self.frame_width}x{self.frame_height}')

            response.success = True
            response.message = f'Started recording to {self.current_filename}'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error starting recording: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def stop_recording_callback(self, request, response):
        """Stop recording video."""
        try:
            with self.lock:
                if not self.is_recording:
                    response.success = False
                    response.message = 'Not currently recording'
                    return response

                # Stop recording
                self.is_recording = False

                if self.video_writer is not None:
                    self.video_writer.release()
                    self.video_writer = None

                # Cancel timed recording timer if active
                if self.recording_timer is not None:
                    self.recording_timer.cancel()
                    self.recording_timer = None

            response.success = True
            response.message = f'Stopped recording. Video saved to {self.current_filename}. Total frames: {self.frame_count}'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error stopping recording: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def record_timed_callback(self, request, response):
        """Start recording for a specified duration (uses record_duration parameter)."""
        try:
            # Get duration from parameter
            duration = self.get_parameter('record_duration').value

            # Start recording
            start_response = Trigger.Response()
            self.start_recording_callback(Trigger.Request(), start_response)

            if not start_response.success:
                response.success = False
                response.message = start_response.message
                return response

            # Schedule stop after duration
            def stop_after_duration():
                stop_response = Trigger.Response()
                self.stop_recording_callback(Trigger.Request(), stop_response)
                self.get_logger().info(f'Timed recording completed after {duration} seconds')

            self.recording_timer = threading.Timer(duration, stop_after_duration)
            self.recording_timer.start()

            response.success = True
            response.message = f'Started timed recording for {duration} seconds'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error starting timed recording: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def __del__(self):
        """Cleanup when node is destroyed."""
        if self.is_recording and self.video_writer is not None:
            with self.lock:
                self.video_writer.release()

        if self.recording_timer is not None:
            self.recording_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to stop recording on shutdown
        if node.is_recording:
            node.get_logger().info('Shutting down, stopping recording...')
            response = Trigger.Response()
            node.stop_recording_callback(Trigger.Request(), response)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
