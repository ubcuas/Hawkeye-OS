#!/usr/bin/env python3
"""
Test client to simulate frontend connection to signaling server.
Helps debug peer_joined event firing.
"""

import asyncio
import socketio

# Create Socket.IO client
sio = socketio.AsyncClient(logger=True, engineio_logger=True)


@sio.event
async def connect():
    """Handle successful connection"""
    print("[TEST CLIENT] Connected to signaling server")
    print(f"[TEST CLIENT] Session ID: {sio.sid}")


@sio.event
async def disconnect():
    """Handle disconnection"""
    print("[TEST CLIENT] Disconnected from signaling server")


@sio.event
async def peer_joined(data):
    """Handle peer_joined event"""
    print(f"[TEST CLIENT] peer_joined event received: {data}")


@sio.event
async def signal(data):
    """Handle signal event"""
    print(f"[TEST CLIENT] signal event received: {data}")


@sio.event
async def error(data):
    """Handle error event"""
    print(f"[TEST CLIENT] error event received: {data}")


async def main():
    """Main test function"""
    signaling_url = "http://localhost:8081"

    print(f"[TEST CLIENT] Connecting to {signaling_url}")
    print("[TEST CLIENT] This simulates the frontend connecting")
    print("[TEST CLIENT] If streaming node is already connected, it should receive peer_joined")
    print()

    try:
        await sio.connect(signaling_url, transports=["websocket"])

        # Stay connected for a while to observe events
        print("[TEST CLIENT] Connected. Waiting for events...")
        await asyncio.sleep(30)

    except Exception as e:
        print(f"[TEST CLIENT] Error: {e}")
    finally:
        if sio.connected:
            await sio.disconnect()
        print("[TEST CLIENT] Disconnected")


if __name__ == "__main__":
    asyncio.run(main())
