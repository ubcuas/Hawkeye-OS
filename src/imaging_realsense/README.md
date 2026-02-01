# imaging_realsense

ROS2 package for processing RealSense camera data using the official realsense2_camera wrapper.

## Architecture

This package uses the **official Intel RealSense ROS2 wrapper** for camera interface and provides custom processing nodes.

## Dependencies

- `realsense2_camera` - Official RealSense ROS2 wrapper
- External GPS module (optional, for geolocation)

## Installation

### Install Official RealSense Wrapper

```bash
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-image-transport
```

### Build This Package

```bash
cd /path/to/hawkeye
pip install -r src/imaging_realsense/requirements.txt
colcon build --packages-select imaging_realsense
source install/setup.bash
```

## Nodes

### image_processor
Subscribes to official RealSense topics and processes data

**Subscribes:**
- `/camera/camera/color/image_raw/compressed` (sensor_msgs/CompressedImage)
- `/camera/camera/imu` (sensor_msgs/Imu)
- `/gps/fix` (sensor_msgs/NavSatFix) - external GPS

**Publishes:**
- `/detections` (std_msgs/String)

## Usage

### Option 1: Launch Everything Together

```bash
ros2 launch imaging_realsense rs_hawkeye_launch.py
```

### Option 2: Launch Separately

```bash
# Terminal 1: Start RealSense camera
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_color:=true

# Terminal 2: Start processor
ros2 run imaging_realsense image_processor
```

## Topics Reference

### Official RealSense Topics
- `/camera/camera/color/image_raw/compressed` - Compressed RGB images
- `/camera/camera/depth/image_rect_raw` - Depth images
- `/camera/camera/imu` - Combined gyro + accel
- `/camera/camera/aligned_depth_to_color/image_raw` - Depth aligned to color

See [RealSense ROS2 Documentation](https://github.com/IntelRealSense/realsense-ros) for full topic list.
