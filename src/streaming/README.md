# Streaming

The streaming module enables real-time video transmission from the Hawkeye OS object detection system to GCOM (Ground Control) using WebRTC.

## Overview

This module subscribes to the `object_detection/image` ROS topic and streams the processed video frames to GCOM over a WebRTC peer connection. The streaming is low-latency and works across networks using ICE/STUN for NAT traversal.

## Architecture

- **ROS Integration**: Subscribes to `object_detection/image` topic to receive processed frames
- **WebRTC Connection**: Establishes peer-to-peer connection for efficient video streaming
- **Signaling Server**: Uses Socket.IO-based signaling server for WebRTC handshake and ICE candidate exchange
- **STUN Servers**: Utilizes Google STUN servers for NAT traversal and connection establishment

## Setup

1. Set the signaling server URL via environment variable:
   ```bash
   export WEBRTC_STREAMING_SERVER=http://your-signaling-server:port
   ```

2. Ensure the signaling server is running - this is the one in `infrastructure/webrtc_signaling_server/server.py` - this needs to be run separately outside of ROS!

3. In the running ROS container run `ros2 run streaming streaming`
4. Goated

## WebRTC Flow

1. Node connects to signaling server via Socket.IO
2. When a peer (GCOM) joins, creates RTCPeerConnection with STUN configuration
3. Generates SDP offer and exchanges via signaling server
4. Exchanges ICE candidates for optimal connection path
5. Establishes peer connection and begins streaming video frames

## Common Issues
`Failed to connect to signaling server: Connection error`
Common Solutions:
-  Make sure to set the WEBRTC_STREAMING_SERVER env variable