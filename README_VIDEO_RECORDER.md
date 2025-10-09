# DEXI Video Recorder

A ROS2 node for recording compressed camera images to MP4 video files with service-based start/stop control and timed recording capabilities.

## Overview

The `video_recorder_node` subscribes to compressed image topics and writes frames directly to MP4 files in real-time. It provides three control services for flexible recording management:
- Manual start/stop recording
- Timed recording (automatically stops after N seconds)

## Features

- ✅ Real-time MP4 recording from compressed image topics
- ✅ Service-based recording control (start, stop, timed)
- ✅ Automatic timestamp-based file naming
- ✅ Configurable video parameters (fps, codec, resolution)
- ✅ Thread-safe recording operations
- ✅ Automatic cleanup on node shutdown

## Installation

The video recorder is included in the `dexi_camera` package. Build it with:

```bash
cd ~/your_ros2_workspace
colcon build --packages-select dexi_camera
source install/setup.bash
```

## Quick Start

### 1. Launch the Recorder Node

```bash
ros2 launch dexi_camera video_recorder.launch.py
```

### 2. Start Recording

```bash
ros2 service call /video_recorder_node/start_recording std_srvs/srv/Trigger
```

### 3. Stop Recording

```bash
ros2 service call /video_recorder_node/stop_recording std_srvs/srv/Trigger
```

## Usage

### Launch File Parameters

The `video_recorder.launch.py` launch file accepts the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_topic` | `/cam0/image_raw/compressed` | Compressed image topic to subscribe to |
| `output_directory` | `/home/dexi/dexi_recordings` | Directory to save recorded videos |
| `fps` | `30.0` | Frames per second for output video |
| `codec` | `avc1` | Video codec (`avc1`, `mp4v`, `h264`) - avc1 recommended for Mac |
| `frame_width` | `320` | Output video frame width |
| `frame_height` | `240` | Output video frame height |
| `record_duration` | `10.0` | Default duration for timed recording (seconds) |

#### Example: Custom Launch Configuration

```bash
ros2 launch dexi_camera video_recorder.launch.py \
  image_topic:=/my_camera/image_raw/compressed \
  output_directory:=~/flight_videos \
  fps:=60.0 \
  frame_width:=1280 \
  frame_height:=720 \
  codec:=avc1
```

### Services

The recorder node provides three services:

#### 1. Start Recording
```bash
ros2 service call /video_recorder_node/start_recording std_srvs/srv/Trigger
```

**Response:**
```
success: True
message: "Started recording to /home/dexi/dexi_recordings/recording_20251009_162014.mp4"
```

#### 2. Stop Recording
```bash
ros2 service call /video_recorder_node/stop_recording std_srvs/srv/Trigger
```

**Response:**
```
success: True
message: "Stopped recording. Video saved to /home/dexi/dexi_recordings/recording_20251009_162014.mp4"
```

#### 3. Timed Recording

Record for a specific duration (uses `record_duration` parameter):

```bash
# Set duration to 30 seconds
ros2 param set /video_recorder_node record_duration 30.0

# Start timed recording
ros2 service call /video_recorder_node/record_timed std_srvs/srv/Trigger
```

The recording will automatically stop after the specified duration.

**Response:**
```
success: True
message: "Started timed recording for 30.0 seconds"
```

### Runtime Parameter Updates

You can change parameters while the node is running:

```bash
# Change default timed recording duration
ros2 param set /video_recorder_node record_duration 60.0

# Change output directory (requires node restart to take effect)
ros2 param set /video_recorder_node output_directory ~/new_videos
```

## Output Files

Videos are saved with automatic timestamp-based naming:

```
/home/dexi/dexi_recordings/recording_YYYYMMDD_HHMMSS.mp4
```

Example:
```
/home/dexi/dexi_recordings/recording_20251009_162014.mp4
/home/dexi/dexi_recordings/recording_20251009_163522.mp4
```

## Integration Examples

### Example 1: Recording with Camera Launch

Launch both camera and recorder together:

```bash
# Terminal 1: Launch camera
ros2 launch dexi_camera csi_camera.launch.py camera_name:=cam0

# Terminal 2: Launch recorder
ros2 launch dexi_camera video_recorder.launch.py image_topic:=/cam0/image_raw/compressed
```

### Example 2: Quick 5-Second Recording Script

Create a bash script for quick recordings:

```bash
#!/bin/bash
# quick_record.sh

# Set duration
ros2 param set /video_recorder_node record_duration 5.0

# Start timed recording
ros2 service call /video_recorder_node/record_timed std_srvs/srv/Trigger

echo "Recording for 5 seconds..."
```

### Example 3: Python Service Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RecordingController(Node):
    def __init__(self):
        super().__init__('recording_controller')
        self.start_client = self.create_client(Trigger, '/video_recorder_node/start_recording')
        self.stop_client = self.create_client(Trigger, '/video_recorder_node/stop_recording')

    def start_recording(self):
        request = Trigger.Request()
        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def stop_recording(self):
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    controller = RecordingController()

    # Start recording
    response = controller.start_recording()
    print(f"Start: {response.message}")

    # Do something...
    import time
    time.sleep(10)

    # Stop recording
    response = controller.stop_recording()
    print(f"Stop: {response.message}")

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Codec Options

The node supports multiple video codecs:

| Codec | Description | Quality | Compatibility |
|-------|-------------|---------|---------------|
| `avc1` | H.264 | Excellent | Best for Mac/QuickTime |
| `mp4v` | MPEG-4 Part 2 | Good | Universal (but not Mac) |
| `h264` | H.264 variant | Excellent | Most players |
| `XVID` | Xvid MPEG-4 | Good | Universal |

**Recommendation:** Use `avc1` for Mac/QuickTime compatibility (default). Use `mp4v` only if avc1 doesn't work on your system.

## Troubleshooting

### Issue: "No such file or directory" for output directory

**Solution:** The node creates the directory automatically, but ensure the parent directory exists and you have write permissions.

```bash
mkdir -p /home/dexi/dexi_recordings
chmod 755 /home/dexi/dexi_recordings
```

### Issue: "Failed to open video writer"

**Possible causes:**
1. Invalid codec specified
2. Insufficient disk space
3. Invalid resolution

**Solution:** Check available codecs on your system:
```bash
python3 -c "import cv2; print(cv2.videoio_registry.getBackendName(cv2.CAP_FFMPEG))"
```

### Issue: Recording stops unexpectedly

**Possible causes:**
1. Disk full
2. Node crashed (check logs)
3. Camera stopped publishing

**Solution:** Check node logs:
```bash
ros2 node info /video_recorder_node
ros2 topic hz /cam0/image_raw/compressed
```

### Issue: Video playback is choppy

**Possible causes:**
1. FPS setting doesn't match actual frame rate
2. Dropped frames during recording

**Solution:** Match FPS to your camera's actual frame rate:
```bash
# Check actual camera frame rate
ros2 topic hz /cam0/image_raw/compressed

# Adjust recorder fps parameter accordingly
ros2 param set /video_recorder_node fps 30.0
```

## Technical Details

### Node Architecture

- **Node Name:** `video_recorder_node`
- **Subscribed Topics:** Configurable compressed image topic (default: `/cam0/image_raw/compressed`)
- **Services Provided:**
  - `~/start_recording` (std_srvs/srv/Trigger)
  - `~/stop_recording` (std_srvs/srv/Trigger)
  - `~/record_timed` (std_srvs/srv/Trigger)

### Thread Safety

The node uses threading locks to ensure thread-safe access to the video writer during concurrent callback executions.

### Graceful Shutdown

The node automatically stops recording and releases video resources when shut down (Ctrl+C or node termination).

## Performance Considerations

- **Disk I/O:** Writing video to disk is I/O intensive. Use fast storage (SSD preferred) for best performance.
- **CPU Usage:** Video encoding uses CPU resources. On Raspberry Pi, consider using hardware acceleration codecs if available.
- **Frame Drops:** If the recorder can't keep up with the camera frame rate, frames may be dropped. Monitor CPU usage and adjust frame rate/resolution as needed.

## Dependencies

- ROS2 (tested with Humble)
- Python 3
- OpenCV (python3-opencv)
- sensor_msgs
- std_srvs

## License

Apache License 2.0

## Contributing

For issues or feature requests related to the video recorder, please submit an issue to the dexi_camera repository.
