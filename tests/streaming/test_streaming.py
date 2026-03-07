"""
Unit tests for streaming.streaming.StreamingNode
=================================================

What we are testing
-------------------
StreamingNode manages the full WebRTC pipeline from the moment a peer joins
the signaling session to the point where video frames are flowing. It has
five responsibilities:

  1. Initial state — sensible defaults before any peer connects.
  2. _route_image_to_track — forward ROS Image messages to the video track.
  3. _send_webrtc_offer — create a peer connection, attach the video track,
     negotiate SDP, and send the offer to the peer via the signaling server.
  4. _handle_answer / _handle_ice_candidate / _add_ice_candidate — complete
     the WebRTC handshake after receiving the peer's response.
  5. shutdown — cleanly close the socket and peer connection.

Test approach
-------------
- fresh_sio (autouse): same pattern as test_signaling_handler.py. socketio
  is a global MagicMock so all StreamingNode instances share one sio mock;
  fresh_sio makes each test get its own independent instance.

- fresh_rtc (autouse): aiortc is a global MagicMock. Methods like
  pc.createOffer() and pc.setLocalDescription() are awaited in the source,
  but await MagicMock() raises TypeError — MagicMock does not implement
  __await__. fresh_rtc replaces RTCPeerConnection.return_value with a
  properly configured mock that has AsyncMock for every awaited method.

- For _handle_answer / _handle_ice_candidate / _add_ice_candidate tests we
  set node.peer_connection directly to a pre-built mock. This avoids going
  through the full _send_webrtc_offer flow and keeps each test focused.

- For _handle_answer's ICE-queue-draining test we patch _add_ice_candidate
  as AsyncMock. This keeps the test focused on the caller's loop-and-clear
  logic rather than the callee's internals.

- signaling_handler.emit_message is patched as AsyncMock in tests that
  trigger _send_webrtc_offer, because emit_message awaits sio.emit which
  would otherwise be an un-awaitable MagicMock.

- shutdown tests patch signaling_handler.disconnect as AsyncMock because the
  real disconnect() is a sync def that returns sio.disconnect() (a MagicMock)
  — and the source awaits the return value.
"""
import sys
import pytest
from unittest.mock import MagicMock, AsyncMock, patch

from streaming.streaming import StreamingNode
from streaming.video_track import ROSVideoStreamTrack


def get_pc_handler(pc, name):
    """
    Recover a closure registered on the peer connection via @pc.on(...).

    _send_webrtc_offer uses @pc.on("event") as a decorator, which calls
    pc.on("event")(func). pc.on("event") always returns the same
    pc.on.return_value mock, so every registered handler ends up in
    pc.on.return_value.call_args_list. We match on func.__name__.
    Same pattern as get_event_handler in test_signaling_handler.py.
    """
    for call in pc.on.return_value.call_args_list:
        func = call.args[0]
        if func.__name__ == name:
            return func
    return None


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fresh_sio():
    """Give each test its own independent sio MagicMock."""
    sys.modules['socketio'].AsyncClient.side_effect = lambda **kwargs: MagicMock()
    yield
    sys.modules['socketio'].AsyncClient.side_effect = None


@pytest.fixture(autouse=True)
def fresh_rtc():
    """
    Give each test a clean RTCPeerConnection mock with AsyncMock on every
    method the source awaits.

    Without this, await pc.createOffer() raises:
        TypeError: object MagicMock can't be used in 'await' expression
    because MagicMock does not implement __await__.

    We return the configured pc mock so individual tests can inspect it
    (e.g. assert pc.addTrack.called).
    """
    pc = MagicMock()
    pc.close = AsyncMock()
    pc.createOffer = AsyncMock(return_value=MagicMock())
    pc.setLocalDescription = AsyncMock()
    pc.setRemoteDescription = AsyncMock()
    pc.addIceCandidate = AsyncMock()
    sys.modules['aiortc'].RTCPeerConnection.return_value = pc
    yield pc


@pytest.fixture
def node():
    return StreamingNode('ws://test.local')


# ---------------------------------------------------------------------------
# __init__
# ---------------------------------------------------------------------------

def test_peer_connection_starts_none(node):
    """No peer connection exists before a peer joins the session."""
    assert node.peer_connection is None


def test_data_channel_starts_none(node):
    """No data channel exists before the WebRTC offer is sent."""
    assert node.data_channel is None


def test_ice_candidate_queue_starts_empty(node):
    """ICE candidates received before the remote description is set are
    queued — this list must start empty."""
    assert node.ice_candidate_queue == []


def test_ice_gathering_complete_starts_false(node):
    """ICE gathering is not complete until the peer connection reports it."""
    assert node.ice_gathering_complete is False


def test_signaling_url_stored(node):
    """The signaling URL must be stored so the handler can connect to it."""
    assert node.signaling_url == 'ws://test.local'


# ---------------------------------------------------------------------------
# _route_image_to_track
# ---------------------------------------------------------------------------

def test_route_image_calls_put_image_on_track(node):
    """Incoming ROS Image messages must be forwarded to the video track
    so the WebRTC stream has frames to send."""
    msg = MagicMock()
    node.video_track = MagicMock()
    node._route_image_to_track(msg)
    node.video_track.put_image.assert_called_once_with(msg)


def test_route_image_does_nothing_when_track_is_none(node):
    """If the video track is not yet set, routing must silently do nothing
    rather than raising an AttributeError."""
    node.video_track = None
    node._route_image_to_track(MagicMock())  # must not raise


# ---------------------------------------------------------------------------
# _send_webrtc_offer
# ---------------------------------------------------------------------------

async def test_send_webrtc_offer_creates_new_video_track(node, fresh_rtc):
    """Each offer must use a fresh ROSVideoStreamTrack because tracks
    cannot be reused after a peer connection closes."""
    original_track = node.video_track
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()
    assert node.video_track is not original_track
    assert isinstance(node.video_track, ROSVideoStreamTrack)


async def test_send_webrtc_offer_creates_peer_connection(node, fresh_rtc):
    """A new RTCPeerConnection must be created and stored so subsequent
    methods (_handle_answer, etc.) can operate on it."""
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()
    assert node.peer_connection is fresh_rtc


async def test_send_webrtc_offer_closes_existing_connection(node, fresh_rtc):
    """If a peer connection already exists when a new peer joins, the old
    one must be closed before creating the new one — leaving it open would
    leak resources and confuse the STUN/TURN servers."""
    old_pc = MagicMock()
    old_pc.close = AsyncMock()
    node.peer_connection = old_pc
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()
    old_pc.close.assert_called_once()


async def test_send_webrtc_offer_adds_video_track_to_connection(node, fresh_rtc):
    """The video track must be added to the peer connection so aiortc
    includes it in the SDP offer and the peer can receive video."""
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()
    fresh_rtc.addTrack.assert_called_once_with(node.video_track)


async def test_send_webrtc_offer_emits_offer_via_signaling(node, fresh_rtc):
    """After creating and setting the local description, the offer must be
    sent to the peer through the signaling server with message_type='offer'."""
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()
    node.signaling_handler.emit_message.assert_called_once()
    assert node.signaling_handler.emit_message.call_args.kwargs['message_type'] == 'offer'


# ---------------------------------------------------------------------------
# _handle_answer
# ---------------------------------------------------------------------------

async def test_handle_answer_logs_error_when_no_peer_connection(node):
    """If the answer arrives before a peer connection exists, the node must
    log an error and return rather than crashing on a NoneType call."""
    node.peer_connection = None
    await node._handle_answer({'sdp': 'v=0...', 'type': 'answer'})
    node.get_logger().error.assert_called()


async def test_handle_answer_sets_remote_description(node):
    """A valid answer must be applied to the peer connection as the remote
    description so ICE negotiation can proceed."""
    pc = MagicMock()
    pc.setRemoteDescription = AsyncMock()
    pc.remoteDescription = None
    node.peer_connection = pc
    await node._handle_answer({'sdp': 'v=0...', 'type': 'answer'})
    pc.setRemoteDescription.assert_called_once()


async def test_handle_answer_drains_ice_candidate_queue(node):
    """ICE candidates that arrived before the remote description was set
    are held in ice_candidate_queue. Once the answer is processed, all
    queued candidates must be passed to _add_ice_candidate and the queue
    must be cleared — otherwise the connection will never establish."""
    pc = MagicMock()
    pc.setRemoteDescription = AsyncMock()
    node.peer_connection = pc
    node.ice_candidate_queue = [{'candidate': 'a'}, {'candidate': 'b'}]

    with patch.object(node, '_add_ice_candidate', AsyncMock()) as mock_add:
        await node._handle_answer({'sdp': 'v=0...', 'type': 'answer'})
        assert mock_add.call_count == 2
        assert node.ice_candidate_queue == []


# ---------------------------------------------------------------------------
# _handle_ice_candidate
# ---------------------------------------------------------------------------

async def test_handle_ice_candidate_queues_when_no_peer_connection(node):
    """ICE candidates that arrive before the peer connection exists must be
    queued so they can be applied once the connection is ready."""
    node.peer_connection = None
    candidate = {'candidate': 'candidate:1 udp 12345'}
    await node._handle_ice_candidate(candidate)
    assert candidate in node.ice_candidate_queue


async def test_handle_ice_candidate_adds_immediately_when_ready(node):
    """When a peer connection with a remote description already exists,
    the candidate must be applied immediately rather than queued."""
    pc = MagicMock()
    pc.remoteDescription = MagicMock()  # truthy — remote desc is set
    node.peer_connection = pc
    candidate = {'candidate': 'candidate:1 udp 12345'}

    with patch.object(node, '_add_ice_candidate', AsyncMock()) as mock_add:
        await node._handle_ice_candidate(candidate)
        mock_add.assert_called_once_with(candidate)
        assert node.ice_candidate_queue == []


# ---------------------------------------------------------------------------
# _add_ice_candidate
# ---------------------------------------------------------------------------

async def test_add_ice_candidate_returns_early_for_empty_candidate(node):
    """An empty candidate string signals end-of-candidates. The node must
    log this and return without calling addIceCandidate."""
    pc = MagicMock()
    pc.addIceCandidate = AsyncMock()
    node.peer_connection = pc
    await node._add_ice_candidate({'candidate': ''})
    pc.addIceCandidate.assert_not_called()


async def test_add_ice_candidate_strips_candidate_prefix(node):
    """The browser sends candidates with a 'candidate:' prefix but
    aiortc's candidate_from_sdp expects it without. The prefix must be
    removed before parsing."""
    pc = MagicMock()
    pc.addIceCandidate = AsyncMock()
    node.peer_connection = pc
    candidate_from_sdp = sys.modules['aiortc.sdp'].candidate_from_sdp
    await node._add_ice_candidate({'candidate': 'candidate:1 1 udp 123'})
    parsed_str = candidate_from_sdp.call_args[0][0]
    assert not parsed_str.startswith('candidate:')


async def test_add_ice_candidate_passes_unprefixed_string_as_is(node):
    """If the candidate string has no prefix, it must be passed directly
    to candidate_from_sdp without modification."""
    pc = MagicMock()
    pc.addIceCandidate = AsyncMock()
    node.peer_connection = pc
    candidate_from_sdp = sys.modules['aiortc.sdp'].candidate_from_sdp
    candidate_from_sdp.reset_mock()
    await node._add_ice_candidate({'candidate': '1 1 udp 123'})
    parsed_str = candidate_from_sdp.call_args[0][0]
    assert parsed_str == '1 1 udp 123'


async def test_add_ice_candidate_calls_add_ice_candidate_on_pc(node):
    """The parsed ICE candidate must be applied to the peer connection
    so the WebRTC engine can attempt that network path."""
    pc = MagicMock()
    pc.addIceCandidate = AsyncMock()
    node.peer_connection = pc
    await node._add_ice_candidate({'candidate': '1 1 udp 123', 'sdpMid': '0', 'sdpMLineIndex': 0})
    pc.addIceCandidate.assert_called_once()


# ---------------------------------------------------------------------------
# shutdown
# ---------------------------------------------------------------------------

async def test_shutdown_disconnects_when_signaling_handler_connected(node):
    """If the socket is still connected when shutdown is called, it must
    be closed so the signaling server can clean up the session."""
    node.signaling_handler._connected = True
    node.signaling_handler.disconnect = AsyncMock()
    await node.shutdown()
    node.signaling_handler.disconnect.assert_called_once()


async def test_shutdown_skips_disconnect_when_not_connected(node):
    """If the socket is already closed, calling disconnect again would
    be a no-op at best and an error at worst — it must be skipped."""
    node.signaling_handler._connected = False
    node.signaling_handler.disconnect = AsyncMock()
    await node.shutdown()
    node.signaling_handler.disconnect.assert_not_called()


async def test_shutdown_closes_peer_connection_if_present(node, fresh_rtc):
    """An open peer connection holds network resources. Shutdown must
    close it so STUN/TURN sessions are released."""
    node.peer_connection = fresh_rtc
    node.signaling_handler._connected = False
    await node.shutdown()
    fresh_rtc.close.assert_called_once()


# ---------------------------------------------------------------------------
# _send_webrtc_offer — on_icecandidate closure
# ---------------------------------------------------------------------------

async def test_on_icecandidate_emits_candidate_to_peer(node, fresh_rtc):
    """When the WebRTC engine generates an ICE candidate, the on_icecandidate
    closure must forward it to the peer via emit_message. Without this, NAT
    traversal never completes and the connection silently stays broken."""
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()

    on_icecandidate = get_pc_handler(fresh_rtc, 'on_icecandidate')
    assert on_icecandidate is not None, "on_icecandidate handler was not registered"

    candidate = MagicMock()
    candidate.candidate = '1 1 udp 12345 192.168.1.1 50000 typ host'
    candidate.sdpMid = '0'
    candidate.sdpMLineIndex = 0

    await on_icecandidate(candidate)

    node.signaling_handler.emit_message.assert_called_with(
        message_type='ice-candidate',
        data={
            'candidate': candidate.candidate,
            'sdpMid': candidate.sdpMid,
            'sdpMLineIndex': candidate.sdpMLineIndex,
        },
    )


async def test_on_icecandidate_skips_null_candidate(node, fresh_rtc):
    """A None candidate signals end-of-candidates — emit_message must not
    be called because there is no candidate data to send."""
    node.signaling_handler.emit_message = AsyncMock()
    await node._send_webrtc_offer()

    on_icecandidate = get_pc_handler(fresh_rtc, 'on_icecandidate')
    # _send_webrtc_offer already called emit_message once for the offer itself.
    # Reset before firing the handler so we only check the handler's behaviour.
    node.signaling_handler.emit_message.reset_mock()

    await on_icecandidate(None)

    node.signaling_handler.emit_message.assert_not_called()
