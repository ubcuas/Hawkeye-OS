import socketio

from rclpy.impl.rcutils_logger import RcutilsLogger

"""Signaling handler for managing WebRTC signaling via Socket.IO

Handles setting up webrtc
"""
class SignalingHandler:
    def __init__(self, logger: RcutilsLogger, node):
        self.logger = logger
        self.connected = False
        self.peer_id = None
        self.node = node

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
            self.connected = True
            self.logger.info("Connected to signaling server")

        @self.sio.event
        async def disconnect():
            """Handle disconnection from signaling server"""
            self.connected = False
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
