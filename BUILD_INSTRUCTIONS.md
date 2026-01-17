# Build Instructions for Hawkeye ROS2 Workspace

## Architecture Support

The Dockerfile now automatically detects and uses the correct ArenaSDK based on the build architecture.

### Available ArenaSDK Files

Currently available in `src/imaging/external/`:
- `ArenaSDK_v0.1.78_Linux_ARM64.tar.gz` - for ARM64/aarch64 systems

### Building for Different Architectures

#### For ARM64 (Raspberry Pi, Apple Silicon, etc.)

The current setup is configured for ARM64. Just build normally:

```bash
docker compose build
docker compose up -d
```

#### For AMD64/x86_64 Systems

If you have an x86_64 ArenaSDK file:

1. Place the ArenaSDK tar file in `src/imaging/external/`
   - File should be named: `ArenaSDK_v0.1.78_Linux_x64.tar.gz`

2. Update `docker-compose.yml`:
   ```yaml
   platforms:
     - linux/amd64  # Changed from linux/arm64
   platform: linux/amd64  # Changed from linux/arm64
   ```

3. Build:
   ```bash
   docker compose build
   docker compose up -d
   ```

#### Multi-Architecture Build (Advanced)

To build for both architectures:

```bash
# Enable buildx if not already enabled
docker buildx create --use

# Build for multiple platforms
docker buildx build --platform linux/amd64,linux/arm64 -t hawkeye-ros:latest .
```

**Note:** Multi-arch builds require both ArenaSDK tar files to be present.

## Building the ROS2 Workspace

Once inside the container:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Or skip imaging if ArenaSDK is not available
colcon build --packages-skip imaging

# Source the workspace
source install/setup.bash
```

## Testing

### Test Orchestrator

```bash
# Terminal 1 - Run orchestrator
ros2 run orchestrator orchestrator

# Terminal 2 - Mock image capture
ros2 run orchestrator mock_image_capture

# Terminal 3 - Mock object detection
ros2 run orchestrator mock_object_detection
```

### Test C++ PubSub

```bash
# Terminal 1
ros2 run cpp_pubsub simple_publisher_cpp

# Terminal 2
ros2 run cpp_pubsub simple_subscriber_cpp
```

### Test Python PubSub

```bash
# Terminal 1
ros2 run py_pubsub simple_publisher_py

# Terminal 2
ros2 run py_pubsub simple_subscriber_py
```

### Test Imaging (when built successfully)

The imaging node will publish telemetry data to `/telemetry` topic when running with camera hardware.

```bash
# Run imaging node (requires ArenaSDK and camera hardware)
ros2 run imaging camerafeed --help

# In another terminal, listen to telemetry
ros2 topic echo /telemetry
```

## Troubleshooting

### Architecture Mismatch Error

If you see:
```
/usr/bin/ld: /opt/arena_sdk/lib/libarena.so: error adding symbols: file in wrong format
```

This means you're building for the wrong architecture. Check:
1. Your ArenaSDK tar file architecture (ARM64 vs x64)
2. Your `docker-compose.yml` platform setting
3. Make sure they match!

### Container Exits Immediately

Check logs:
```bash
docker logs hawkeye-ros
```

The build may have failed. Try building without imaging:
```bash
docker exec hawkeye-ros colcon build --packages-skip imaging
```

