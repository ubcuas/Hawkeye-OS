# WebRTC recv() Debug Investigation

## The Issue

When launching the streaming node via `ros2 launch orchestrator mock_image_streaming_launch.py`:
- The WebRTC connection establishes successfully
- The data channel opens
- ROS frames are being received and queued (30 frames at 30 FPS)
- BUT: `ROSVideoStreamTrack.recv()` is **not being called** during normal operation
- The `recv()` method only appears in logs during Ctrl-C shutdown
- No frames are transmitted to the remote peer

When launching via `ros2 run streaming streaming` directly, recv() is called (but queue is empty due to no mock_object_detection).

---

## Attempts & Findings

### 1. Event Loop Sleep Duration (FAILED)
**Hypothesis**: The asyncio event loop wasn't yielding control frequently enough for aiortc tasks.

**Action**: Changed `await asyncio.sleep(0.01)` to `await asyncio.sleep(1e-4)` in the ROS spin loop.

**Result**: No change. recv() still not called.

**Revealed**: The issue is NOT related to event loop yield frequency.

---

### 2. Background Thread for ROS (FAILED)
**Hypothesis**: ROS executor blocking the main event loop prevented aiortc tasks from running.

**Action**: Moved ROS executor to a dedicated background thread, leaving main event loop free for asyncio/socketio/aiortc.

**Result**: No change. recv() still not called.

**Revealed**: The issue is NOT caused by ROS blocking the event loop.

---

### 3. Comprehensive Debug Logging (CRITICAL INSIGHT)
**Hypothesis**: Need to understand WebRTC connection lifecycle and track state.

**Action**: Added extensive debug logging for:
- Video track creation
- Peer connection events (connection state, ICE state, etc.)
- recv() entry/exit with frame details
- Periodic connection monitoring (every 5 seconds)

**Result**: Logs revealed:
```
Connection State: connected ✓
Data Channel: opened ✓
ROS Frames Received: 890+ ✓
Video Track Queue: 30/30 (full) ✓
recv() calls: ONLY 1 (at initial connection)
sender.track: None (5 seconds after connection)
Frames served: 0
```

**Revealed**:
- recv() IS called once initially
- sender.track becomes None shortly after
- This prevents further recv() calls

---

### 4. Test Script Creation (VERIFICATION)
**Hypothesis**: Need to verify connection from client perspective.

**Action**: Created `test_webrtc_connection.py` - a Python test client that:
- Connects to signaling server
- Accepts offer from streaming node
- Sends back answer with `recvonly` transceiver
- Attempts to receive video frames
- Logs all SDP and transceiver details

**Result**:
- Client receives video track successfully
- Connection state: connected
- But receives **zero frames** (timeout after 5s)

**Revealed**:
- SDP negotiation is correct
- Remote peer is properly configured
- Problem is on the sender side, not receiver

---

### 5. Sender Reference Storage (FAILED)
**Hypothesis**: The RTCRtpSender object was being garbage collected.

**Action**: Stored sender as instance variable `self.video_sender = pc.addTrack(self.video_track)`.

**Result**: No change. sender.track still becomes None.

**Revealed**: Sender object itself persists, but its `.track` property becomes None.

---

### 6. Transceiver Direction to sendonly (FAILED)
**Hypothesis**: Offering `sendrecv` instead of `sendonly` confused aiortc's negotiation.

**Action**: Set transceiver direction to "sendonly" before creating offer.

**Result**: SDP offer correctly shows `a=sendonly`, but sender.track still becomes None.

**Revealed**: The direction negotiation is correct but doesn't prevent the track reference loss.

---

### 7. SDP Analysis (IMPORTANT INSIGHT)
**Hypothesis**: SDP negotiation reveals track direction mismatch.

**Action**: Analyzed complete SDP offer/answer exchange and transceiver states.

**Result**:
```
Offer: a=sendonly ✓
Answer: a=recvonly ✓
Negotiated currentDirection: sendonly ✓
BUT: sender.track = None after 5 seconds
```

**Revealed**: SDP negotiation is perfect. The issue occurs AFTER successful negotiation.

---

### 8. Sender Internal Task State Logging (BREAKTHROUGH)
**Hypothesis**: aiortc's RTCRtpSender internal task might not be starting or is crashing.

**Action**: Added logging to check:
- sender._task state
- sender._run_rtp task existence
- Task exceptions
- Sender state immediately after answer vs 5 seconds later

**Result**:
```
recv() CALLED - frame_count=0, queue_size=30
Frame retrieved from queue, shape=(360, 640, 3)
[Expected "Frame converted successfully" log NEVER appears]
sender.track = None (immediately after)
```

**Revealed**:
- recv() was called by aiortc's sender task
- Frame was retrieved from queue successfully
- **An exception was raised during frame processing** (between lines 60-74)
- The exception crashed the sender task
- aiortc set sender.track = None as cleanup
- No further recv() calls possible

---

## Root Cause

The `recv()` method only catches `asyncio.TimeoutError` but NOT other exceptions that can be raised by:
1. `VideoFrame.from_ndarray()` - can raise ValueError
2. `await self.next_timestamp()` - can raise any exception
3. Setting pts/time_base - can fail

When an uncaught exception is raised, the aiortc sender task crashes, causing:
- sender._task to complete with exception
- aiortc to set sender.track = None
- No further recv() calls
- Complete streaming failure

---

## Current Solution

Added comprehensive exception handling in recv() method:
- Wrap frame processing in try-except
- Catch ALL exceptions (not just TimeoutError)
- Log exception details for debugging
- Fall back to blank frame on error
- Keep sender task alive

Status: **Testing in progress**

---

## Alternative Solution (NOT YET TRIED)

### 9. Explicit Track Consumer Task
**Hypothesis**: aiortc's RTCRtpSender may not automatically consume outbound tracks. We may need to explicitly create a consumer task.

**Theory**: According to aiortc documentation:
> "If the aiortc application doesn't call await track.recv() to consume the frames, it can lead to a memory leak and processing stall, causing tracks to fail or appear frozen."

**Proposed Action**: Create an explicit consumer task for our outbound video track:
```python
async def _consume_track(self):
    while True:
        try:
            await self.video_track.recv()
        except Exception:
            break

# After adding track to peer connection:
asyncio.create_task(self._consume_track())
```

**Why this might work**:
- We've been assuming RTCRtpSender automatically calls recv()
- But we may need to explicitly create a task that consumes frames
- This would explain why recv() is only called once (during initial connection check) but not continuously

**Status**: NOT YET IMPLEMENTED

This should be the next thing to try if the exception handling fix doesn't resolve the issue.
