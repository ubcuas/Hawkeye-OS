#!/usr/bin/env python3
"""
Test client to simulate frontend connection to signaling server.
Handles full WebRTC connection flow with the streaming node.
"""

import asyncio
import socketio
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
import cv2
import numpy as np

# Create Socket.IO client
sio = socketio.AsyncClient(logger=True, engineio_logger=True)

# WebRTC state
peer_connection = None
ice_candidate_queue = []
received_frames = 0

# WebRTC configuration with STUN servers
rtc_configuration = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
        RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
    ]
)


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

    message_type = data.get("type")

    if message_type == "offer":
        await handle_offer(data.get("data"), data.get("from"))
    elif message_type == "ice-candidate":
        await handle_ice_candidate(data.get("data"))


@sio.event
async def error(data):
    """Handle error event"""
    print(f"[TEST CLIENT] error event received: {data}")


async def receive_video_track(track):
    """
    Receive and process video frames from the track.
    Displays frames using OpenCV and logs statistics.
    """
    global received_frames

    print(f"[TEST CLIENT] Starting video track receiver for: {track.kind}")

    try:
        while True:
            # Receive frame from track
            frame = await track.recv()

            # Convert to numpy array
            img = frame.to_ndarray(format="rgb24")

            # Convert RGB to BGR for OpenCV
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Display frame
            cv2.imshow("WebRTC Video Stream", img_bgr)
            cv2.waitKey(1)

            received_frames += 1

            # Log every 30 frames
            if received_frames % 30 == 0:
                print(f"[TEST CLIENT] Received {received_frames} frames ({img.shape[1]}x{img.shape[0]})")

    except Exception as e:
        print(f"[TEST CLIENT] Video track receiver ended: {e}")
    finally:
        cv2.destroyAllWindows()


async def handle_offer(offer_data, from_peer_id):
    """Handle incoming SDP offer and send answer"""
    global peer_connection

    try:
        print(f"[TEST CLIENT] Handling offer from peer: {from_peer_id}")

        # Create peer connection
        peer_connection = RTCPeerConnection(configuration=rtc_configuration)

        # Set up ICE candidate handler
        @peer_connection.on("icecandidate")
        async def on_icecandidate(candidate):
            if candidate:
                print(f"[TEST CLIENT] Sending ICE candidate: {candidate.candidate}")
                await sio.emit("signal", {
                    "to": from_peer_id,
                    "type": "ice-candidate",
                    "data": {
                        "candidate": candidate.candidate,
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex,
                    }
                })

        # Set up connection state change handler
        @peer_connection.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"[TEST CLIENT] Connection state: {peer_connection.connectionState}")

        # Set up ICE connection state change handler
        @peer_connection.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"[TEST CLIENT] ICE connection state: {peer_connection.iceConnectionState}")

        # Set up ICE gathering state change handler
        @peer_connection.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            print(f"[TEST CLIENT] ICE gathering state: {peer_connection.iceGatheringState}")

        # Set up track handler for receiving video
        @peer_connection.on("track")
        def on_track(track):
            print(f"[TEST CLIENT] Received track: {track.kind}")
            if track.kind == "video":
                # Start receiving video in background task
                asyncio.create_task(receive_video_track(track))

        # Set up data channel handler
        @peer_connection.on("datachannel")
        def on_datachannel(channel):
            print(f"[TEST CLIENT] Data channel received: {channel.label}")

            @channel.on("open")
            def on_open():
                print(f"[TEST CLIENT] Data channel opened: {channel.label}")
                # Send a test message
                channel.send("Hello from test client!")

            @channel.on("close")
            def on_close():
                print(f"[TEST CLIENT] Data channel closed: {channel.label}")

            @channel.on("message")
            def on_message(message):
                print(f"[TEST CLIENT] Received message on data channel: {message}")

        # Set remote description with the offer
        offer = RTCSessionDescription(
            sdp=offer_data.get("sdp"),
            type=offer_data.get("type")
        )
        await peer_connection.setRemoteDescription(offer)

        print("[TEST CLIENT] Remote description set")

        # Process queued ICE candidates
        if ice_candidate_queue:
            print(f"[TEST CLIENT] Processing {len(ice_candidate_queue)} queued ICE candidates")
            for candidate_data in ice_candidate_queue:
                await add_ice_candidate(candidate_data)
            ice_candidate_queue.clear()

        # Create answer
        answer = await peer_connection.createAnswer()
        await peer_connection.setLocalDescription(answer)

        print("[TEST CLIENT] Sending SDP answer")

        # Send answer to peer
        await sio.emit("signal", {
            "to": from_peer_id,
            "type": "answer",
            "data": {
                "sdp": peer_connection.localDescription.sdp,
                "type": peer_connection.localDescription.type,
            }
        })

    except Exception as e:
        print(f"[TEST CLIENT] Error handling offer: {e}")
        import traceback
        traceback.print_exc()


async def handle_ice_candidate(candidate_data):
    """Handle incoming ICE candidate"""
    global peer_connection

    try:
        # If we don't have remote description yet, queue the candidate
        if not peer_connection or not peer_connection.remoteDescription:
            print("[TEST CLIENT] Queueing ICE candidate (no remote description yet)")
            ice_candidate_queue.append(candidate_data)
            return

        # Otherwise, add it immediately
        await add_ice_candidate(candidate_data)

    except Exception as e:
        print(f"[TEST CLIENT] Error handling ICE candidate: {e}")
        import traceback
        traceback.print_exc()


async def add_ice_candidate(candidate_data):
    """Add an ICE candidate to the peer connection"""
    global peer_connection

    try:
        if not candidate_data.get("candidate"):
            print("[TEST CLIENT] Received end-of-candidates signal")
            return

        candidate = RTCIceCandidate(
            candidate=candidate_data.get("candidate"),
            sdpMid=candidate_data.get("sdpMid"),
            sdpMLineIndex=candidate_data.get("sdpMLineIndex")
        )

        await peer_connection.addIceCandidate(candidate)
        print(f"[TEST CLIENT] Added ICE candidate: {candidate_data.get('candidate')[:50]}...")

    except Exception as e:
        print(f"[TEST CLIENT] Error adding ICE candidate: {e}")
        import traceback
        traceback.print_exc()


async def main():
    """Main test function"""
    signaling_url = "http://localhost:8081"

    print(f"[TEST CLIENT] Connecting to {signaling_url}")
    print("[TEST CLIENT] This simulates the frontend connecting")
    print("[TEST CLIENT] If streaming node is already connected, it should receive peer_joined")
    print()

    try:
        await sio.connect(signaling_url, transports=["websocket"])

        # Stay connected for a while to observe events and complete WebRTC negotiation
        print("[TEST CLIENT] Connected. Waiting for WebRTC connection to establish...")
        await asyncio.sleep(60)

    except Exception as e:
        print(f"[TEST CLIENT] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        global peer_connection
        if peer_connection:
            await peer_connection.close()
            print("[TEST CLIENT] Peer connection closed")

        if sio.connected:
            await sio.disconnect()
        print("[TEST CLIENT] Disconnected")


if __name__ == "__main__":
    asyncio.run(main())
