# DEXI Camera

A ROS2 package for interfacing with the Arducam 12 MP UVC Module on Raspberry Pi 5.

## Features

- MJPEG camera streaming
- Configurable resolution and frame rate
- Camera calibration support
- Compressed image publishing
- Camera info publishing

## Dependencies

- ROS2 (tested on Humble)
- OpenCV
- camera_info_manager
- camera_calibration_parsers

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/droneblocks/dexi_camera.git
   ```

2. Install dependencies:
   ```bash
   sudo apt-get install ros-humble-camera-info-manager ros-humble-camera-calibration-parsers
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select dexi_camera
   source install/setup.bash
   ```

## Usage

### Basic Usage

Launch the camera node with default parameters:
```bash
ros2 launch dexi_camera camera.launch.py
```

### Custom Parameters

You can override default parameters when launching:
```bash
ros2 launch dexi_camera camera.launch.py --ros-args -p camera_id:=1 -p camera_width:=1920 -p camera_height:=1080
```

Available parameters:
- `camera_id`: Camera device ID (default: 0)
- `camera_width`: Image width (default: 1280)
- `camera_height`: Image height (default: 720)
- `jpeg_quality`: JPEG compression quality (default: 80)
- `timer_interval`: Frame interval in seconds (default: 1/30)
- `camera_name`: Camera name for topics (default: "camera")
- `camera_info_url`: Path to camera calibration file

### Topics

The node publishes the following topics:
- `/<camera_name>/image_raw/compressed`: Compressed image data
- `/<camera_name>/camera_info`: Camera calibration information

## Camera Calibration

### Using Default Calibration

The package includes a default calibration file in `config/camera_calibration.yaml`. This is a template with reasonable default values, but for best results, you should calibrate your camera.

### Calibrating Your Camera

1. Print a chessboard pattern (8x6 or similar)
2. Launch the camera node:
   ```bash
   ros2 launch dexi_camera camera.launch.py
   ```

3. Run the calibration tool:
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 image:=/camera/image_raw
   ```
   (Adjust the chessboard size and square size to match your calibration target)

4. Move the chessboard around in front of the camera to collect calibration data
5. When enough data is collected, click "Calibrate" in the calibration window
6. Save the calibration file and copy it to `config/camera_calibration.yaml`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Author

Dennis Baldwin (db@droneblocks.io) 