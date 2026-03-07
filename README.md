# Hawkeye-OS

## Repository Structure 
```bash 
Hawkeye-OS/
├── .dockerignore 
├── .gitignore 
├── docker-compose.yml 
├── Dockerfile
├── mock_gcom.py 
├── README.md
├── start_system.sh
├── stop_system.sh 
├── test-hawkeye-os.sh 
├── received_stream/
└── src/
    └── orchestrator
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        ├── orchestrator/
            ├── __init__.py
            ├── mock_object_detection.py
            └── orchestrator.py 
        └── resource/
            └── orchestrator
└── test_images
    ├── anpanman_wooddadandan_hero.jpg
    └── test_video.mp4
```

### Installing Dependencies 
Install `docker, docker-compose, tmux`.

// I think we can ignore this since we have the requirements.txt
Install required Python Packages: 
```bash
pip install websockets aiortc av opencv-python numpy
```

### Building the Image for the First Time (Manual)
Build the image (in project root)
```bash
docker-compose build
```

Run the container
```bash
docker-compose up -d
```

Get the bash (On each terminal to test)
```bash
docker-compose exec ros2_workspace bash
```

Inside the terminal, build the workplace
```bash
colcon build
```

### Testing (Manual) 
Make sure you're in the project's root directory (../Hawkeye-OS)

In a Normal Terminal: 
```bash 
py mock_gcom.py
```

Inside the Docker Workspace (see previous section for setup): 
```bash 
ros2 run orchestrator orchestrator
```

For testing, mock queues are available 
```bash
ros2 run orchestrator mock_object_detection ("on another terminal")
```

### Automated Build/Test
For bash shells, there are files you can run to automate the test setups. 

To run the script, make sure you make it executable with:
```bash
chmod +x start_system.sh stop_system.sh
```

Run the script (in project root):
```bash
./start_system.sh 
```

To stop the script: 
```bash 
./stop_system.sh 
```

---

## ROS Topics

### Subscribed Topics

| Topic | Message Type | Publisher Node | Subscriber Node | Description |
|---|---|---|---|---|
| `/depth/image_rect_raw/compressed` | `CompressedImage` | `imaging_realsense` | `object_detection` | Synchronized depth frames from RealSense |
| `color/image_raw/compressed` | `CompressedImage` | `imaging_realsense` | `object_detection` | Synchronized color frames from RealSense |
| `/camera/camera/color/image_raw` | `Image` | `imaging_realsense` | `image_processor` | Raw color image from RealSense |
| `/camera/camera/imu` | `Imu` | `imaging_realsense` | `image_processor` | IMU data from RealSense |
| `/gps/fix` | `NavSatFix` | External / MAVLink | `image_processor` | GPS fix data |
| `color/image_raw/compressed` | `Image` | `object_detection` / `image_processor` | `orchestrator`, `streaming` | Processed image output from object detection |
| `object_detection/tagged_image` | `TaggedImage` | `image_processor` | `streaming` | Image with bounding box and detection metadata |

### Published Topics

| Topic | Message Type | Publisher Node | Description |
|---|---|---|---|
| `/color/image_raw/compressed` | `CompressedImage` | `object_detection` | Detection results published downstream |
| `color/image_raw/compressed` | `Image` | `image_processor` | Forwarded image for streaming/orchestration |
| `object_detection/tagged_image` | `TaggedImage` | `image_processor` | Tagged image with color, bounding box, confidence |
| `/detections` | `String` | `image_processor` | Raw detection string output |

---

## MQTT Topics (Orchestrator ↔ GCOM)

| Topic | Direction | Description |
|---|---|---|
| `ubc_uas/drone_01/commands` | GCOM → Orchestrator | Commands sent from ground control |
| `ubc_uas/drone_01/status` | Orchestrator → GCOM | Drone status updates |
| `ubc_uas/drone_01/command_ack` | Orchestrator → GCOM | Acknowledgement of received commands |