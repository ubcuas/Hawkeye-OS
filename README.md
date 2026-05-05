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

---

## 1) Running on Jetson with RealSense

### Build the image (one time)

From the project root:

```bash
docker compose build
```

### Development workflow

**Start the workspace container** (on Jetson, include the GPU overlay):

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d
```

**Open a shell in the container:**

```bash
docker compose exec ros2_workspace bash
```

**Inside the container**, build and source the workspace:

```bash
colcon build
source install/setup.sh
```

**Then run the stack in separate terminals** (after `source install/setup.sh` anywhere you launch ROS nodes):

1. **Terminal 1 — on the host**, from the project root, start the VSLAM deploy container (requires the deployed VSLAM image and environment expected by that script, e.g. `ISAAC_ROS_WS` if applicable):

   ```bash
   ./vslam/deploy_docker_vslam.sh
   ```

2. **Terminal 2 — inside the Hawkeye ROS container** (or another `docker compose exec ros2_workspace bash` session with the workspace sourced):

   ```bash
   ros2 launch imaging_realsense rs_hawkeye_launch.py
   ```

Use the same `ROS_DOMAIN_ID` on the host and in Compose so VSLAM and the Hawkeye nodes can see each other (the deploy script passes through `ROS_DOMAIN_ID`).

### Tailscale exit node (legacy)

🔴 **Warning:** Exit-node-based routing via Tailscale is **being migrated away from** for this stack. Use this only where you still must; it will be replaced.

Run in this order (substitute your ground machine’s Tailscale IP or tailnet name from `tailscale status` where noted):

1. **Ground machine** — join the tailnet and log in when the command prints a URL:

   ```bash
   sudo tailscale up
   ```

2. **Ground machine** — advertise as exit node (approve in the [Tailscale admin console](https://login.tailscale.com/admin/machines) if required):

   ```bash
   tailscale up --advertise-exit-node
   ```

3. **Ground machine** — confirm the address the client will use:

   ```bash
   tailscale status
   ```

4. **Jetson (tailnet)** — join and log in when prompted:

   ```bash
   sudo tailscale up
   ```

5. **Jetson (tailnet)** — use the ground machine as exit node:

   ```bash
   tailscale up --exit-node=<GROUND_MACHINE_TAILSCALE_IP_OR_NAME>
   ```

6. If **streaming** fails or is flaky, on the affected machine sign out and back in, then redo the steps that set exit routing for that machine (**2** on ground, **5** on client):

   ```bash
   tailscale logout
   sudo tailscale up
   ```

---

## 2) Running locally without hardware

🔴 **Warning:** Running locally without hardware is **currently broken**.

Use the same **development workflow** as above, but start the workspace with the base compose file only (no GPU overlay unless you need it):

```bash
docker compose up -d
docker compose exec ros2_workspace bash
```

Inside the container:

```bash
colcon build
source install/setup.sh
```

**Instead of VSLAM and RealSense**, run only the mock perception publisher:

```bash
ros2 run orchestrator mock_object_detection
```


---

## ROS topics

### Subscribed topics

| Topic | Message type | Publisher node | Subscriber node | Description |
| --- | --- | --- | --- | --- |
| `/depth/image_rect_raw/compressed` | `CompressedImage` | `imaging_realsense` | `object_detection` | Synchronized depth frames from RealSense |
| `color/image_raw/compressed` | `CompressedImage` | `imaging_realsense` | `object_detection` | Synchronized color frames from RealSense |
| `/camera/camera/color/image_raw` | `Image` | `imaging_realsense` | `image_processor` | Raw color image from RealSense |
| `/camera/camera/imu` | `Imu` | `imaging_realsense` | `image_processor` | IMU data from RealSense |
| `/gps/fix` | `NavSatFix` | External / MAVLink | `image_processor` | GPS fix data |
| `color/image_raw/compressed` | `Image` | `object_detection` / `image_processor` | `orchestrator`, `streaming` | Processed image output from object detection |
| `object_detection/tagged_image` | `TaggedImage` | `image_processor` | `streaming` | Image with bounding box and detection metadata |

### Published topics

| Topic | Message type | Publisher node | Description |
| --- | --- | --- | --- |
| `/color/image_raw/compressed` | `CompressedImage` | `object_detection` | Detection results published downstream |
| `color/image_raw/compressed` | `Image` | `image_processor` | Forwarded image for streaming/orchestration |
| `object_detection/tagged_image` | `TaggedImage` | `image_processor` | Tagged image with color, bounding box, confidence |
| `/detections` | `String` | `image_processor` | Raw detection string output |

---

## MQTT topics (orchestrator ↔ GCOM)

| Topic | Direction | Description |
| --- | --- | --- |
| `ubc_uas/drone_01/commands` | GCOM → orchestrator | Commands sent from ground control |
| `ubc_uas/drone_01/status` | Orchestrator → GCOM | Drone status updates |
| `ubc_uas/drone_01/command_ack` | Orchestrator → GCOM | Acknowledgement of received commands |
