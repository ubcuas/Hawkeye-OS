# imaging_realsense

ROS2 package for Intel RealSense camera integration with synchronized image and GPS/IMU data.

## Nodes

- `camera_publisher` - Publishes images and telemetry from RealSense camera
- `image_processor` - Processes images and telemetry data

## Topics

### camera_publisher
- Publishes: `/camera/image_raw/compressed` (sensor_msgs/CompressedImage)
- Publishes: `/camera/telemetry` (std_msgs/String)

### image_processor
- Subscribes: `/camera/image_raw/compressed`
- Subscribes: `/camera/telemetry`
- Publishes: `/detections` (std_msgs/String)

## Installation

```bash
cd /path/to/hawkeye
pip install -r src/imaging_realsense/requirements.txt
colcon build --packages-select imaging_realsense
source install/setup.bash
```

## Usage

```bash
ros2 run imaging_realsense camera_publisher
ros2 run imaging_realsense image_processor
```
