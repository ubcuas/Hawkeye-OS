import asyncio
import os
import traceback
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import socketio
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.sdp import candidate_from_sdp
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
import numpy as np
from sensor_msgs.msg import Image

from streaming.constants import WEBRTC_SIGNALING_URL

"""
Streaming Node

Handles WebRTC-based video streaming to GCOM via a signaling server.
Manages peer connection establishment and data channel communication.
"""


class ROSVideoStreamTrack(VideoStreamTrack):
    """
    Custom video track that reads frames from a ROS topic via asyncio queue.
    Converts ROS Image messages to VideoFrames for WebRTC transmission.
    """

    def __init__(self):
        super().__init__()
        self.frame_queue = asyncio.Queue(maxsize=30)
        self._timestamp = 0
        self._frame_count = 0

    async def recv(self):
        """
        Receive the next video frame.
        Called by aiortc when it needs a frame to send.
        """
        try:
            # Get frame from queue (blocks if empty)
            frame_data = await asyncio.wait_for(self.frame_queue.get(), timeout=1.0)

            # Convert numpy array to VideoFrame
            video_frame = VideoFrame.from_ndarray(frame_data, format="rgb24")

            # Set presentation timestamp
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            self._frame_count += 1

            return video_frame

        except asyncio.TimeoutError:
            # No frame available, generate a blank frame
            blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            video_frame = VideoFrame.from_ndarray(blank_frame, format="rgb24")

            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            return video_frame

    def put_frame(self, frame_data):
        """
        Add a frame to the queue (called from ROS callback).
        Non-blocking - drops frame if queue is full.
        """
        try:
            self.frame_queue.put_nowait(frame_data)
        except asyncio.QueueFull:
            # Drop frame if queue is full to prevent blocking
            pass


class StreamingNode(Node):
    def __init__(self, signaling_url: Optional[str] = None):
        super().__init__("streaming")

        # Configuration
        self.signaling_url = signaling_url

        # Connection state
        self.sio = socketio.AsyncClient(
            logger=False,
            engineio_logger=False,
            reconnection=False,  # We'll handle reconnection manually
        )
        self.connected = False
        self.peer_id: Optional[str] = None

        # Retry configuration
        self.max_retry_delay = 30.0  # Maximum delay between retries (seconds)
        self.initial_retry_delay = 1.0  # Initial retry delay (seconds)
        self.retry_backoff_factor = 2.0  # Exponential backoff multiplier

        # WebRTC state
        self.peer_connection = None
        self.data_channel = None
        self.ice_candidate_queue = []  # Queue ICE candidates until ready
        self.ice_gathering_complete = False
        self.video_track = None

        # WebRTC configuration with STUN servers
        self.rtc_configuration = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
            ]
        )

        # Subscribe to video feed from object detection
        self.image_subscription = self.create_subscription(
            Image,
            'object_detection/image',
            self._image_callback,
            10
        )

        # Register Socket.IO event handlers
        self._register_socketio_handlers()

        self.get_logger().info("Streaming node initialized")
        self.get_logger().info(f"Signaling server URL: {self.signaling_url}")
        self.get_logger().info("Subscribed to: object_detection/image")

    def _image_callback(self, msg: Image):
        """
        Callback for receiving images from ROS topic.
        Converts ROS Image message to numpy array and adds to video track queue.
        """
        try:
            # Convert ROS Image message to numpy array
            # ROS Image data is in bytes, reshape according to dimensions
            height = msg.height
            width = msg.width
            channels = 3 if msg.encoding == 'rgb8' else 1

            # Convert bytes to numpy array
            frame_data = np.frombuffer(msg.data, dtype=np.uint8)
            frame_data = frame_data.reshape((height, width, channels))

            # Add frame to video track if it exists
            if self.video_track:
                self.video_track.put_frame(frame_data)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _register_socketio_handlers(self):
        """Register Socket.IO event handlers"""

        @self.sio.event
        async def connect():
            """Handle successful connection to signaling server"""
            self.connected = True
            self.get_logger().info("Connected to signaling server")

        @self.sio.event
        async def disconnect():
            """Handle disconnection from signaling server"""
            self.connected = False
            self.get_logger().warn("Disconnected from signaling server")

        @self.sio.event
        async def peer_joined(data):
            """Handle notification of peer joining"""
            peer_id = data.get("peer_id")
            self.get_logger().info(f"Peer joined: {peer_id}")
            print(f"Peer joined: {peer_id}")
            self.peer_id = peer_id

            # Initiate WebRTC offer when peer joins
            await self._send_webrtc_offer()

        @self.sio.event
        async def signal(data):
            """Handle incoming WebRTC signaling messages"""
            message_type = data.get("type")
            self.get_logger().info(f"Received signal: {message_type}")

            if message_type == 'answer':
                await self._handle_answer(data.get('data'))
            elif message_type == 'ice-candidate':
                await self._handle_ice_candidate(data.get('data'))

        @self.sio.event
        async def error(data):
            """Handle error messages from signaling server"""
            self.get_logger().error(f"Signaling server error: {data}")

    async def _send_webrtc_offer(self):
        """Create peer connection, generate SDP offer, and send to peer via signaling server"""
        try:
            self.get_logger().info("Creating WebRTC peer connection")

            # Create peer connection
            self.peer_connection = RTCPeerConnection(configuration=self.rtc_configuration)

            # Capture in local variable for type narrowing
            pc = self.peer_connection

            # Set up ICE candidate handler
            @pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    self.get_logger().info(f"Sending ICE candidate: {candidate.candidate}")
                    await self.sio.emit("signal", {
                        "to": self.peer_id,
                        "type": "ice-candidate",
                        "data": {
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex,
                        }
                    })

            # Set up connection state change handler
            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                self.get_logger().info(f"Connection state: {pc.connectionState}")

            # Set up ICE connection state change handler
            @pc.on("iceconnectionstatechange")
            async def on_iceconnectionstatechange():
                self.get_logger().info(f"ICE connection state: {pc.iceConnectionState}")

            # Set up ICE gathering state change handler
            @pc.on("icegatheringstatechange")
            async def on_icegatheringstatechange():
                self.get_logger().info(f"ICE gathering state: {pc.iceGatheringState}")
                if pc.iceGatheringState == "complete":
                    self.ice_gathering_complete = True

            # Create and add video track
            self.video_track = ROSVideoStreamTrack()
            pc.addTrack(self.video_track)
            self.get_logger().info("Video track added to peer connection")

            # Create data channel
            self.data_channel = pc.createDataChannel("streaming")
            self.get_logger().info("Data channel created")

            @self.data_channel.on("open")
            def on_open():
                self.get_logger().info("Data channel opened")

            @self.data_channel.on("close")
            def on_close():
                self.get_logger().info("Data channel closed")

            @self.data_channel.on("message")
            def on_message(message):
                self.get_logger().info(f"Received message on data channel: {message}")

            # Create offer
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)

            self.get_logger().info("Sending SDP offer to peer")

            # Send offer to peer via signaling server
            await self.sio.emit("signal", {
                "to": self.peer_id,
                "type": "offer",
                "data": {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }
            })

        except Exception as e:
            self.get_logger().error(f"Error creating WebRTC offer: {e}")
            self.get_logger().error(traceback.format_exc())

    async def _handle_answer(self, answer_data):
        """Handle SDP answer from peer"""
        try:
            self.get_logger().info("Received SDP answer from peer")

            if not self.peer_connection:
                self.get_logger().error("Cannot handle answer: peer connection not initialized")
                return

            # Set remote description
            answer = RTCSessionDescription(
                sdp=answer_data.get("sdp"),
                type=answer_data.get("type")
            )
            await self.peer_connection.setRemoteDescription(answer)

            self.get_logger().info("Remote description set successfully")

            # Process queued ICE candidates now that we have remote description
            if self.ice_candidate_queue:
                self.get_logger().info(f"Processing {len(self.ice_candidate_queue)} queued ICE candidates")
                for candidate_data in self.ice_candidate_queue:
                    await self._add_ice_candidate(candidate_data)
                self.ice_candidate_queue.clear()

        except Exception as e:
            self.get_logger().error(f"Error handling answer: {e}")
            self.get_logger().error(traceback.format_exc())

    async def _handle_ice_candidate(self, candidate_data):
        """Handle ICE candidate from peer"""
        try:
            # If we don't have remote description yet, queue the candidate
            if not self.peer_connection or not self.peer_connection.remoteDescription:
                self.get_logger().info("Queueing ICE candidate (no remote description yet)")
                self.ice_candidate_queue.append(candidate_data)
                return

            # Otherwise, add it immediately
            await self._add_ice_candidate(candidate_data)

        except Exception as e:
            self.get_logger().error(f"Error handling ICE candidate: {e}")
            self.get_logger().error(traceback.format_exc())

    async def _add_ice_candidate(self, candidate_data):
        """Add an ICE candidate to the peer connection"""
        try:
            if not candidate_data.get("candidate"):
                self.get_logger().info("Received end-of-candidates signal")
                return

            # Parse the candidate string
            candidate_str = candidate_data.get("candidate")

            # Remove "candidate:" prefix if present
            if candidate_str.startswith("candidate:"):
                candidate_str = candidate_str.split(":", 1)[1]

            # Parse using aiortc's SDP parser
            candidate = candidate_from_sdp(candidate_str)

            # Set the media stream identification
            candidate.sdpMid = candidate_data.get("sdpMid")
            candidate.sdpMLineIndex = candidate_data.get("sdpMLineIndex")

            if self.peer_connection:
                await self.peer_connection.addIceCandidate(candidate)
            self.get_logger().info(f"Added ICE candidate: {candidate_data.get('candidate')[:50]}...")

        except Exception as e:
            self.get_logger().error(f"Error adding ICE candidate: {e}")
            self.get_logger().error(traceback.format_exc())

    async def connect_to_signaling_server(self):
        """
        Connect to signaling server with exponential backoff retry logic.

        This coroutine will continuously attempt to connect to the signaling
        server, logging each attempt and backing off exponentially on failure.
        """
        retry_delay = self.initial_retry_delay
        attempt = 0

        while rclpy.ok():
            attempt += 1

            try:
                self.get_logger().info(
                    f"Attempting to connect to signaling server "
                    f"(attempt {attempt}): {self.signaling_url}"
                )

                await self.sio.connect(self.signaling_url, transports=["websocket"])

                # Connection successful
                self.get_logger().info(
                    f"Successfully connected to signaling server "
                    f"after {attempt} attempt(s)"
                )
                retry_delay = self.initial_retry_delay  # Reset retry delay

                # Wait for disconnection
                await self.sio.wait()

            except Exception as e:
                # Log the main error
                self.get_logger().error(
                    f"Failed to connect to signaling server: {e}"
                )

                # Log the cause if available (this is where the real error often is)
                if e.__cause__:
                    self.get_logger().error(f"Caused by: {e.__cause__}")

                # Log full traceback for debugging
                self.get_logger().debug(
                    f"Full traceback:\n{traceback.format_exc()}"
                )

                self.get_logger().info(f"Retrying in {retry_delay:.1f} seconds...")

                # Wait before retrying
                await asyncio.sleep(retry_delay)

                # Exponential backoff
                retry_delay = min(
                    retry_delay * self.retry_backoff_factor, self.max_retry_delay
                )

    async def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down streaming node...")

        if self.sio.connected:
            await self.sio.disconnect()

        # Close WebRTC connections
        if self.peer_connection:
            await self.peer_connection.close()
            self.get_logger().info("WebRTC peer connection closed")


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    streaming_node = StreamingNode(WEBRTC_SIGNALING_URL)

    executor = SingleThreadedExecutor()
    executor.add_node(streaming_node)

    async def spin():
        """Spin the ROS node"""
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        # Run both the ROS spin loop and signaling server connection
        await asyncio.gather(spin(), streaming_node.connect_to_signaling_server())
    except KeyboardInterrupt:
        pass
    finally:
        await streaming_node.shutdown()
        streaming_node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == "__main__":
    main()
