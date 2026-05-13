import asyncio
import base64
import json
import logging
import math
import os
import traceback
import cv2
import numpy as np

# Suppress noisy internal logs from socketio/engineio/aiohttp
logging.getLogger("socketio").setLevel(logging.WARNING)
logging.getLogger("engineio").setLevel(logging.WARNING)
logging.getLogger("aiohttp").setLevel(logging.WARNING)

import rclpy
from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.sdp import candidate_from_sdp
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

from hawkeye_msgs.msg import TaggedImage
from streaming.constants import WEBRTC_SIGNALING_URL
from streaming.signaling_handler import SignalingHandler
from streaming.video_track import ROSVideoStreamTrack

"""
Streaming Node

Handles WebRTC-based video streaming to GCOM via a signaling server.
Manages peer connection establishment and data channel communication.
"""

DEPTH_PATCH_RADIUS = 3


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
        self.latest_tagged_image_msg = None

        self.video_track = ROSVideoStreamTrack(self.get_logger())

        # WebRTC configuration with STUN servers
        self.rtc_configuration = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
            ]
        )

        # Subscribe to video feed — mock pipeline (mock_object_detection publishes here)
        self.image_subscription = self.create_subscription(
            CompressedImage,
            "color/image_raw/compressed",
            self._route_image_to_track,
            10,
        )

        # Subscribe to image_processor output — real camera pipeline (rs_hawkeye_launch)
        # image_processor synchronizes color+depth and publishes TaggedImage; we pull the
        # color frame from it to drive the WebRTC video track.
        self.image_processor_subscription = self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            self._route_image_to_track,
            10,
        )

        # Same topic as imaging_realsense image_processor OUTPUT_TOPIC (capture cache)
        self.image_processor_tagged_subscription = self.create_subscription(
            TaggedImage,
            "/image_processor/tagged_image",
            self._on_image_processor_tagged_cache,
            10,
        )

        self.tagged_image_subscription = self.create_subscription(
            TaggedImage,
            "object_detection/tagged_image",
            self._on_object_detection_tagged,
            10,
        )

        # Subscribe to manual capture requests from the orchestrator
        # import os
        # self.capture_request_subscription = self.create_subscription(
        #     String,
        #     os.getenv("IMAGE_REQUEST_TOPIC", "image_request"),
        #     self._on_capture_request,
        #     10,
        # )

        # Register Socket.IO event handlers
        self.signaling_handler = SignalingHandler(
            signaling_url=self.signaling_url, logger=self.get_logger(), node=self
        )
        self.signaling_handler._register_socketio_handlers()

        self.get_logger().info("Streaming node initialized")
        self.get_logger().info(f"Signaling server URL: {self.signaling_url}")
        self.get_logger().info("Subscribed to: color/image_raw/compressed (mock)")
        self.get_logger().info(
            "Capture cache refreshed from TaggedImage on: /image_processor/tagged_image"
        )
        self.get_logger().info("Subscribed to: object_detection/tagged_image")

    def _route_image_to_track(self, msg: CompressedImage):
        """Route incoming CompressedImage to the WebRTC video track (mock pipeline)"""
        if self.video_track:
            self.video_track.put_image(msg)

    def _route_camera_tagged_image(self, msg: TaggedImage):
        """Extract color frame from image_processor TaggedImage and send to WebRTC (real camera pipeline)"""
        if self.video_track and msg.image_data.data:
            self.video_track.put_image(msg.image_data)

    def _on_capture_request(self):
        """Send the latest cached TaggedImage when a manual capture is requested."""
        if self.latest_tagged_image_msg is None:
            self.get_logger().warn("Capture requested but no tagged image cached yet.")
            return
        self._send_tagged_image_over_datachannel(self.latest_tagged_image_msg)

    def _on_depth_sample_request(self, u_norm: float, v_norm: float):
        """Sample depth (meters) around a normalized click on the latest cached frame and reply."""
        if self.latest_tagged_image_msg is None:
            self.get_logger().warn("Depth sample requested but no tagged image cached yet.")
            self._send_depth_result(u_norm, v_norm, None)
            return

        depth_msg = self.latest_tagged_image_msg.depth_data
        if not depth_msg.data:
            self.get_logger().warn("Depth sample requested but cached depth data is empty.")
            self._send_depth_result(u_norm, v_norm, None)
            return

        depth_img_m = self._decode_depth_image_to_meters(depth_msg)
        if depth_img_m is None:
            self._send_depth_result(u_norm, v_norm, None)
            return

        h, w = depth_img_m.shape[:2]
        u = int(round(u_norm * (w - 1)))
        v = int(round(v_norm * (h - 1)))

        depth_m = self._sample_depth_median(depth_img_m, u, v, DEPTH_PATCH_RADIUS)
        self._send_depth_result(u_norm, v_norm, depth_m)

    def _send_depth_result(self, u_norm: float, v_norm: float, depth_m):
        if not self.data_channel or self.data_channel.readyState != "open":
            return
        payload = json.dumps(
            {
                "action": "DEPTH_RESULT",
                "u": u_norm,
                "v": v_norm,
                "depth_m": None if depth_m is None else float(depth_m),
            }
        )
        self.data_channel.send(payload)

    def _decode_depth_image_to_meters(self, depth_msg: CompressedImage):
        np_arr = np.frombuffer(depth_msg.data, np.uint8)
        depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        if depth_img is None:
            self.get_logger().error(
                f"Failed to decode depth image, format='{depth_msg.format}'"
            )
            return None

        if depth_img.dtype == np.uint16:
            return depth_img.astype(np.float32) / 1000.0
        if depth_img.dtype == np.float32:
            return depth_img

        self.get_logger().error(f"Unsupported depth dtype: {depth_img.dtype}")
        return None

    def _sample_depth_median(self, depth_img_m, u: int, v: int, radius: int):
        h, w = depth_img_m.shape[:2]
        if w == 0 or h == 0:
            return None

        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        u0 = max(0, u - radius)
        u1 = min(w, u + radius + 1)
        v0 = max(0, v - radius)
        v1 = min(h, v + radius + 1)

        patch = depth_img_m[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _on_image_processor_tagged_cache(self, msg: TaggedImage):
        """Latest synchronized TaggedImage from image_processor; feeds TAKE_PHOTO / capture."""
        self.latest_tagged_image_msg = msg

    def _on_object_detection_tagged(self, msg: TaggedImage):
        """OD-annotated stream to GCOM when detections publish; capture cache stays from image_processor."""
        self._send_tagged_image_over_datachannel(msg)

    def _send_tagged_image_over_datachannel(self, msg: TaggedImage):
        """Encode TaggedImage metadata + frames and push to GCOM."""
        if not self.data_channel or self.data_channel.readyState != "open":
            return

        if not msg.image_data.data:
            self.get_logger().warn("Empty color image data. Skipping.")
            return

        color_arr = np.frombuffer(msg.image_data.data, dtype=np.uint8)
        if color_arr.size == 0:
            return

        color_frame = cv2.imdecode(color_arr, cv2.IMREAD_COLOR)
        if color_frame is None:
            self.get_logger().error("Failed to decode color CompressedImage")
            return

        _, color_jpeg = cv2.imencode(".jpg", color_frame)
        color_b64 = base64.b64encode(color_jpeg.tobytes()).decode("utf-8")

        depth_b64 = None
        if msg.depth_data.data:
            depth_arr = np.frombuffer(msg.depth_data.data, dtype=np.uint8)
            if depth_arr.size > 0:
                depth_frame = cv2.imdecode(depth_arr, cv2.IMREAD_ANYDEPTH)
                if depth_frame is not None:
                    # Using PNG for depth to preserve 16-bit values losslessly
                    _, depth_png = cv2.imencode(".png", depth_frame)
                    depth_b64 = base64.b64encode(depth_png.tobytes()).decode("utf-8")
            else:
                self.get_logger().warn("malformed depth data. Skipping.")
        # --- 3. SEND PAYLOAD ---
        q = msg.imu_orientation
        yaw = msg.yaw_deg
        payload = json.dumps(
            {
                "image_data": color_b64,  # Now sending the actual color image
                "depth_data": depth_b64,  # Added a new key for the depth image
                "color_detection": [msg.color_r, msg.color_g, msg.color_b],
                # Normalize bbox coords to 0-1 range; GCOM renders against viewBox="0 0 1 1"
                "bounding_box": [
                    [pt.x / color_frame.shape[1], pt.y / color_frame.shape[0]]
                    for pt in msg.bounding_box
                ],
                "confidence_level": msg.confidence_level,
                "imu_orientation": {
                    "x": q.x,
                    "y": q.y,
                    "z": q.z,
                    "w": q.w,
                },
                "yaw_deg": None if math.isnan(yaw) else yaw,
            }
        )
        self.data_channel.send(payload)

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

            self.get_logger().info("Creating WebRTC peer connection")

            # Create a new video track for this connection
            # (tracks can't be reused after the peer connection closes)
            self.video_track = ROSVideoStreamTrack(self.get_logger())

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
            self.data_channel = pc.createDataChannel("odlc_images")
            self.get_logger().info("Data channel created")

            @self.data_channel.on("open")
            def on_open():
                self.get_logger().info("Data channel opened")

            @self.data_channel.on("close")
            def on_close():
                self.get_logger().info("Data channel closed")

            @self.data_channel.on("message")
            def on_message(message):
                try:
                    payload = json.loads(message)
                    action = payload.get("action")
                    if action == "TAKE_PHOTO":
                        self.get_logger().info(
                            "TAKE_PHOTO command received via data channel"
                        )
                        self._on_capture_request()
                    elif action == "SAMPLE_DEPTH":
                        u = float(payload.get("u"))
                        v = float(payload.get("v"))
                        self.get_logger().info(
                            f"SAMPLE_DEPTH command received via data channel (u={u:.3f}, v={v:.3f})"
                        )
                        self._on_depth_sample_request(u, v)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to handle data channel message: {e}"
                    )

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
