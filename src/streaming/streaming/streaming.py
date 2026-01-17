import asyncio
import os
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import socketio

from streaming.streaming.constants import WEBRTC_SIGNALING_URL

"""
Streaming Node

Handles WebRTC-based video streaming to GCOM via a signaling server.
Manages peer connection establishment and data channel communication.
"""


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

        # WebRTC state (to be implemented)
        self.peer_connection = None
        self.data_channel = None
        self.ice_candidate_queue = []  # Queue ICE candidates until ready

        # Register Socket.IO event handlers
        self._register_socketio_handlers()

        self.get_logger().info("Streaming node initialized")
        self.get_logger().info(f"Signaling server URL: {self.signaling_url}")

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
            self.peer_id = peer_id

            # TODO: In next step, initiate WebRTC offer when peer joins
            # await self._send_webrtc_offer()

        @self.sio.event
        async def signal(data):
            """Handle incoming WebRTC signaling messages"""
            message_type = data.get("type")
            self.get_logger().info(f"Received signal: {message_type}")

            # TODO: In next step, handle SDP answer and ICE candidates
            # if message_type == 'answer':
            #     await self._handle_answer(data.get('data'))
            # elif message_type == 'ice-candidate':
            #     await self._handle_ice_candidate(data.get('data'))

        @self.sio.event
        async def error(data):
            """Handle error messages from signaling server"""
            self.get_logger().error(f"Signaling server error: {data}")

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
                self.get_logger().error(f"Failed to connect to signaling server: {e}")
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

        # TODO: In next step, close WebRTC connections
        # if self.peer_connection:
        #     await self.peer_connection.close()


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
