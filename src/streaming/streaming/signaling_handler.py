import asyncio
import socketio
import traceback

import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger

"""Signaling handler for managing WebRTC signaling via Socket.IO

Handles setting up webrtc
"""


class SignalingHandler:
    def __init__(self, signaling_url: str, logger: RcutilsLogger, node):
        self.logger = logger
        self._connected = False
        self.peer_id = None
        self.node = node

        # Retry configuration
        self.initial_retry_delay = 1.0  # Initial retry delay (seconds)
        self.max_retry_delay = 30.0  # Maximum delay between retries (seconds)
        self.retry_backoff_factor = 2.0  # Exponential backoff multiplier

        self.signaling_url = signaling_url

        self.sio = socketio.AsyncClient(
            logger=False,
            engineio_logger=False,
            reconnection=False,
        )

    def _register_socketio_handlers(self):
        """Register Socket.IO event handlers"""

        @self.sio.event
        async def connect():
            """Handle successful connection to signaling server"""
            self._connected = True
            self.logger.info("Connected to signaling server")

        @self.sio.event
        async def disconnect():
            """Handle disconnection from signaling server"""
            self._connected = False
            self.logger.warn("Disconnected from signaling server")

        @self.sio.event
        async def peer_joined(data):
            """Handle notification of peer joining"""
            peer_id = data.get("peer_id")
            self.logger.info(f"Peer joined: {peer_id}")
            print(f"Peer joined: {peer_id}")
            self.peer_id = peer_id

            # Initiate WebRTC offer when peer joins
            await self.node._send_webrtc_offer()

        @self.sio.event
        async def signal(data):
            """Handle incoming WebRTC signaling messages"""
            message_type = data.get("type")
            self.logger.info(f"Received signal: {message_type}")

            if message_type == "answer":
                await self.node._handle_answer(data.get("data"))
            elif message_type == "ice-candidate":
                await self.node._handle_ice_candidate(data.get("data"))

        @self.sio.event
        async def error(data):
            """Handle error messages from signaling server"""
            self.logger.error(f"Signaling server error: {data}")

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
                self.logger.info(
                    f"Attempting to connect to signaling server "
                    f"(attempt {attempt}): {self.signaling_url}"
                )

                await self.sio.connect(self.signaling_url, transports=["websocket"])

                # Connection successful
                self.logger.info(
                    f"Successfully connected to signaling server "
                    f"after {attempt} attempt(s)"
                )
                retry_delay = self.initial_retry_delay  # Reset retry delay

                # Wait for disconnection
                await self.sio.wait()

            except Exception as e:
                # Log the main error
                self.logger.error(f"Failed to connect to signaling server: {e}")

                # Log the cause if available (this is where the real error often is)
                if e.__cause__:
                    self.logger.error(f"Caused by: {e.__cause__}")

                # Log full traceback for debugging
                self.logger.debug(f"Full traceback:\n{traceback.format_exc()}")

                self.logger.info(f"Retrying in {retry_delay:.1f} seconds...")

                # Wait before retrying
                await asyncio.sleep(retry_delay)

                # Exponential backoff
                retry_delay = min(
                    retry_delay * self.retry_backoff_factor, self.max_retry_delay
                )

    async def emit_message(self, message_type: str, data: dict):
        return await self.sio.emit(
            "signal",
            {
                "to": self.peer_id,
                "type": message_type,
                "data": data,
            },
        )

    def disconnect(self):
        return self.sio.disconnect()

    @property
    def connected(self):
        return self._connected
