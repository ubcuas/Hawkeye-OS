import asyncio
import os
import traceback
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import socketio
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCIceCandidate,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.sdp import candidate_from_sdp
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
import numpy as np
from sensor_msgs.msg import Image

from streaming.constants import WEBRTC_SIGNALING_URL
from streaming.signaling_handler import SignalingHandler
from streaming.video_track import ROSVideoStreamTrack

"""
Streaming Node

Handles WebRTC-based video streaming to GCOM via a signaling server.
Manages peer connection establishment and data channel communication.
"""


class StreamingNode(Node):
    def __init__(self, signaling_url: str):
        super().__init__("streaming")

        # Configuration
        self.signaling_url = signaling_url

        # WebRTC state
        self.peer_connection = None
        self.data_channel = None
        self.ice_candidate_queue = []  # Queue ICE candidates until ready
        self.ice_gathering_complete = False

        self._received_frame_count = 0

        self.video_track = ROSVideoStreamTrack(self.get_logger())

        # WebRTC configuration with STUN servers
        self.rtc_configuration = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
            ]
        )

        # Subscribe to video feed from object detection
        self.image_subscription = self.create_subscription(
            Image, "object_detection/image", self.video_track.put_image, 10
        )

        # Register Socket.IO event handlers
        self.signaling_handler = SignalingHandler(
            signaling_url=self.signaling_url, logger=self.get_logger(), node=self
        )
        self.signaling_handler._register_socketio_handlers()

        self.get_logger().info("Streaming node initialized")
        self.get_logger().info(f"Signaling server URL: {self.signaling_url}")
        self.get_logger().info("Subscribed to: object_detection/image")

    async def connect_to_signaling_server(self):
        await self.signaling_handler.connect_to_signaling_server()

    async def _send_webrtc_offer(self):
        """Create peer connection, generate SDP offer, and send to peer via signaling server"""
        try:
            # Check if peer connection already exists
            if self.peer_connection:
                current_state = self.peer_connection.connectionState
                self.get_logger().info(
                    f"Peer connection already exists with state: {current_state}\n"
                    + "Closing old peer connection before creating new one"
                )
                await self.peer_connection.close()
                self.peer_connection = None
                self.data_channel = None
                # ! maybe set video track to none - potentially dbuious change

            self.get_logger().info("Creating WebRTC peer connection")

            # Create peer connection
            self.peer_connection = RTCPeerConnection(
                configuration=self.rtc_configuration
            )

            pc = self.peer_connection

            # This handles NAT traversal from OUR side
            # So this is called when we have a new ICE candidate to send
            @pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    self.get_logger().info(
                        f"Sending ICE candidate: {candidate.candidate}"
                    )
                    await self.signaling_handler.emit_message(
                        message_type="ice-candidate",
                        data={
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex,
                        },
                    )

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
            pc.addTrack(self.video_track)
            self.get_logger().info("Video track added to peer connection")
            self.get_logger().info(
                f"Video track queue size: {self.video_track.frame_queue.qsize()}"
            )

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
            await self.signaling_handler.emit_message(
                message_type="offer",
                data={
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                },
            )

        except Exception as e:
            self.get_logger().error(f"Error creating WebRTC offer: {e}")
            self.get_logger().error(traceback.format_exc())

    async def _handle_answer(self, answer_data):
        """Handle SDP answer from peer"""
        try:
            self.get_logger().info("Received SDP answer from peer")

            if not self.peer_connection:
                self.get_logger().error(
                    "Cannot handle answer: peer connection not initialized"
                )
                return

            # Set remote description
            answer = RTCSessionDescription(
                sdp=answer_data.get("sdp"), type=answer_data.get("type")
            )
            await self.peer_connection.setRemoteDescription(answer)

            self.get_logger().info("Remote description set successfully")

            # Process queued ICE candidates now that we have remote description
            if self.ice_candidate_queue:
                self.get_logger().info(
                    f"Processing {len(self.ice_candidate_queue)} queued ICE candidates"
                )
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
                self.get_logger().info(
                    "Queueing ICE candidate (no remote description yet)"
                )
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
            self.get_logger().info(
                f"Added ICE candidate: {candidate_data.get('candidate')[:50]}..."
            )

        except Exception as e:
            self.get_logger().error(f"Error adding ICE candidate: {e}")
            self.get_logger().error(traceback.format_exc())

    async def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down streaming node...")

        if self.signaling_handler.connected:
            await self.signaling_handler.disconnect()

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
            await asyncio.sleep(1e-4)

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
