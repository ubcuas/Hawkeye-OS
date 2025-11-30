#!/usr/bin/env python3
"""
Mock GCOM Server - Simplified for Testing

Two servers:
1. WebSocket :8765 - Send commands to orchestrator
2. WebRTC :8766 - Receive video stream from orchestrator
"""

import asyncio
import json
import websockets
from websockets.server import serve
from pathlib import Path
from aiortc import RTCPeerConnection, RTCSessionDescription
import cv2
import numpy as np


class MockGCOM:
    def __init__(self, ws_port=8765, signaling_port=8766):
        self.ws_port = ws_port
        self.signaling_port = signaling_port
        
        # WebSocket clients
        self.ws_clients = set()
        self.signaling_clients = set()
        
        # WebRTC
        self.pc = None
        
        # Storage
        self.stream_dir = Path('received_stream')
        self.stream_dir.mkdir(exist_ok=True)
        
        self.frame_count = 0

    # ===== WebSocket Server (Commands) =====
    
    async def websocket_handler(self, websocket):
        """Handle WebSocket connections for commands"""
        self.ws_clients.add(websocket)
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        print(f"[WS] Orchestrator connected: {client_id}")
        
        try:
            # Listen for messages from orchestrator
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get('type')
                    
                    if msg_type == 'status':
                        print(f"[WS] Status: {data.get('message')}")
                    elif msg_type == 'image':
                        print(f"[WS] Received single image: {data.get('width')}x{data.get('height')}")
                    else:
                        print(f"[WS] Received: {data}")
                        
                except json.JSONDecodeError:
                    print(f"[WS] Invalid JSON")
                    
        except websockets.exceptions.ConnectionClosed:
            print(f"[WS] Orchestrator disconnected: {client_id}")
        finally:
            self.ws_clients.remove(websocket)

    # ===== WebRTC Signaling Server =====
    
    async def signaling_handler(self, websocket):
        """Handle WebRTC signaling"""
        self.signaling_clients.add(websocket)
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        print(f"[SIG] Orchestrator connected: {client_id}")
        
        try:
            print(f"[SIG] Waiting for offer from orchestrator...")

            
            # Wait for answer
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    if data.get('type') == 'offer':
                        print(f"[SIG] Received offer from orchestrator")
                        await self.handle_webrtc_offer(websocket, data)
                        
                except json.JSONDecodeError:
                    print(f"[SIG] Invalid JSON")
                    
        except websockets.exceptions.ConnectionClosed:
            print(f"[SIG] Orchestrator disconnected: {client_id}")
        finally:
            self.signaling_clients.remove(websocket)

    async def handle_webrtc_offer(self, websocket, data):
        """Handle WebRTC offer from orchestrator and send answer"""
        print("[SIG] Creating WebRTC peer connection")
        self.pc = RTCPeerConnection()
        
        # Add connection state handlers
        @self.pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"[RTC] *** Connection state: {self.pc.connectionState} ***")
        
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"[RTC] *** ICE connection state: {self.pc.iceConnectionState} ***")
        
        # Set up to receive video
        @self.pc.on("track")
        async def on_track(track):
            print(f"[RTC] !!! TRACK RECEIVED: {track.kind} track !!!")
            
            if track.kind == "video":
                print(f"[RTC] Starting video frame receiver...")
                asyncio.ensure_future(self.receive_video_frames(track))
        
        # Set remote description (the offer)
        await self.pc.setRemoteDescription(
            RTCSessionDescription(sdp=data['sdp'], type='offer')
        )
        print("[RTC] Remote description set")
        
        # Create answer
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        # Send answer back
        await websocket.send(json.dumps({
            'type': 'answer',
            'sdp': self.pc.localDescription.sdp
        }))
        print("[RTC] Sent WebRTC answer")
        print("[RTC] Waiting for connection to establish...")

    async def receive_video_frames(self, track):
        """Receive and process video frames from WebRTC stream"""
        print("[RTC] !!! Video stream receiver started! !!!")
        
        try:
            while True:
                frame = await track.recv()
                self.frame_count += 1
                
                # Convert to numpy array
                img = frame.to_ndarray(format="bgr24")
                
                # Display frame info every 30 frames (once per second at 30 FPS)
                if self.frame_count % 30 == 0:
                    print(f"[RTC] Received frame #{self.frame_count}: {img.shape[1]}x{img.shape[0]}")
                    
                    # Save this frame
                    filename = f"stream_frame_{self.frame_count:06d}.jpg"
                    filepath = self.stream_dir / filename
                    cv2.imwrite(str(filepath), img)
                    print(f"[RTC] Saved: {filepath}")
                    
        except Exception as e:
            print(f"[RTC] Stream ended: {e}")

    # ===== Command Sender =====
    
    async def send_test_commands(self):
        """Send test commands to orchestrator"""
        await asyncio.sleep(3)  # Wait for connections
        
        if not self.ws_clients:
            print("\n[CMD] No orchestrator connected yet, waiting...")
            await asyncio.sleep(2)
        
        if not self.ws_clients:
            print("[CMD] Still no connection. Make sure orchestrator is running!")
            return
        
        print("\n" + "="*60)
        print("Starting automated test sequence...")
        print("="*60 + "\n")
        
        # Test 1: Start WebRTC stream
        print("[CMD] → Sending: start_stream")
        await self.broadcast_ws({'command': 'start_stream'})
        print("[CMD] ! Stream should start now")
        print("[CMD]   Watch for '[RTC] Received frame' messages...")
        await asyncio.sleep(10)
        
        # Test 2: Capture single image (during stream)
        print("\n[CMD] → Sending: capture_image")
        await self.broadcast_ws({'command': 'capture_image'})
        print("[CMD] ! Single image requested")
        print("[CMD]   Stream continues in background...")
        await asyncio.sleep(3)
        
        # Test 3: Stop stream
        print("\n[CMD] → Sending: stop_stream")
        await self.broadcast_ws({'command': 'stop_stream'})
        print("[CMD] ! Stream stopped")
        await asyncio.sleep(2)
        
        # Test 4: Another single image (no stream)
        print("\n[CMD] → Sending: capture_image")
        await self.broadcast_ws({'command': 'capture_image'})
        print("[CMD] ! Single image requested (no stream)")
        
        print("\n" + "="*60)
        print("Test sequence complete!")
        print("="*60)
        print(f"\n[RESULTS]")
        print(f"  Total frames received: {self.frame_count}")
        print(f"  Frames saved to: {self.stream_dir.absolute()}")
        print(f"\nPress Ctrl+C to stop the server")

    async def broadcast_ws(self, message):
        """Send message to all WebSocket clients"""
        if self.ws_clients:
            await asyncio.gather(
                *[client.send(json.dumps(message)) for client in self.ws_clients]
            )

    # ===== Main Server =====
    
    async def start(self):
        """Start both WebSocket and signaling servers"""
        print("="*60)
        print("Mock GCOM Server Starting...")
        print("="*60)
        print(f"WebSocket (commands):  ws://localhost:{self.ws_port}")
        print(f"WebRTC Signaling:      ws://localhost:{self.signaling_port}")
        print(f"Stream frames saved:   {self.stream_dir.absolute()}")
        print("="*60)
        print("\nWaiting for orchestrator to connect...")
        print("(Make sure to run: ros2 run orchestrator orchestrator)\n")
        
        async with serve(self.websocket_handler, "0.0.0.0", self.ws_port, max_size = 20 * 1024 * 1024), \
                   serve(self.signaling_handler, "0.0.0.0", self.signaling_port):
            
            await asyncio.gather(
                self.send_test_commands(),
                asyncio.Future()  # Run forever
            )


async def main():
    gcom = MockGCOM()
    await gcom.start()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n[GCOM] Shutting down...")
        print("Goodbye!")