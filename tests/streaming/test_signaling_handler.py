"""
Unit tests for streaming.signaling_handler.SignalingHandler
============================================================

What we are testing
-------------------
SignalingHandler owns the Socket.IO connection to the signaling server
and is responsible for three things:

  1. Registering event handlers that react to server-sent events
     (connect, disconnect, peer_joined, signal, error).

  2. Routing incoming WebRTC signaling messages (answer, ice-candidate)
     to the correct methods on the parent StreamingNode.

  3. Retrying the connection with exponential backoff when the server
     is unreachable.

Test approach
-------------
- socketio.AsyncClient is mocked in conftest.py. The fresh_sio autouse
  fixture makes each test get its own independent sio mock instance so
  call history never bleeds between tests.
- Event handler functions are closures defined inside
  _register_socketio_handlers(). We recover them after registration by
  scanning sio.event.call_args_list and matching on __name__.
- node methods that are awaited (_send_webrtc_offer, _handle_answer,
  _handle_ice_candidate) are replaced with AsyncMock per-test so they
  are actually awaitable.
- The backoff test verifies the formula directly using the constants
  from the object — no async loop needed.
"""
import sys
import pytest
from unittest.mock import MagicMock, AsyncMock, patch

from streaming.signaling_handler import SignalingHandler


# ---------------------------------------------------------------------------
# Fixtures and helpers
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fresh_sio():
    """Give each test its own sio MagicMock so call history never bleeds."""
    sys.modules['socketio'].AsyncClient.side_effect = lambda **kwargs: MagicMock()
    yield
    sys.modules['socketio'].AsyncClient.side_effect = None


@pytest.fixture
def handler():
    return SignalingHandler(
        signaling_url='ws://test.local',
        logger=MagicMock(),
        node=MagicMock(),
    )


@pytest.fixture
def registered(handler):
    """Handler with all Socket.IO event handlers already registered."""
    handler._register_socketio_handlers()
    return handler


def get_event_handler(sio, name):
    """
    Recover a registered Socket.IO handler by its function name.

    _register_socketio_handlers uses @sio.event as a decorator, which
    calls sio.event(func) and records func in call_args_list. We match
    on func.__name__ to find the right one.
    """
    for call in sio.event.call_args_list:
        func = call.args[0]
        if func.__name__ == name:
            return func
    return None


# ---------------------------------------------------------------------------
# Init
# ---------------------------------------------------------------------------

def test_connected_starts_false(handler):
    """Node must not report itself as connected before the socket handshake."""
    assert handler._connected is False


def test_peer_id_starts_none(handler):
    """There is no peer until the signaling server sends peer_joined."""
    assert handler.peer_id is None


def test_connected_property_reflects_internal_state(handler):
    """The public connected property must mirror _connected exactly."""
    assert handler.connected is False
    handler._connected = True
    assert handler.connected is True


# ---------------------------------------------------------------------------
# Event handlers
# ---------------------------------------------------------------------------

async def test_connect_handler_sets_connected_true(registered):
    """Successful socket connection must flip _connected to True."""
    connect = get_event_handler(registered.sio, 'connect')
    await connect()
    assert registered._connected is True


async def test_disconnect_handler_sets_connected_false(registered):
    """Socket disconnection must flip _connected back to False."""
    registered._connected = True
    disconnect = get_event_handler(registered.sio, 'disconnect')
    await disconnect()
    assert registered._connected is False


async def test_peer_joined_stores_peer_id(registered):
    """The peer's ID must be saved so emit_message can address it correctly."""
    registered.node._send_webrtc_offer = AsyncMock()
    peer_joined = get_event_handler(registered.sio, 'peer_joined')
    await peer_joined({'peer_id': 'abc-123'})
    assert registered.peer_id == 'abc-123'


async def test_peer_joined_triggers_webrtc_offer(registered):
    """As soon as a peer joins, we must initiate the WebRTC handshake."""
    registered.node._send_webrtc_offer = AsyncMock()
    peer_joined = get_event_handler(registered.sio, 'peer_joined')
    await peer_joined({'peer_id': 'abc-123'})
    registered.node._send_webrtc_offer.assert_called_once()


async def test_signal_answer_routes_to_handle_answer(registered):
    """An 'answer' signal must be forwarded to _handle_answer with its data payload."""
    registered.node._handle_answer = AsyncMock()
    signal = get_event_handler(registered.sio, 'signal')
    answer_data = {'sdp': 'v=0...', 'type': 'answer'}
    await signal({'type': 'answer', 'data': answer_data})
    registered.node._handle_answer.assert_called_once_with(answer_data)


async def test_signal_ice_candidate_routes_to_handle_ice(registered):
    """An 'ice-candidate' signal must be forwarded to _handle_ice_candidate."""
    registered.node._handle_ice_candidate = AsyncMock()
    signal = get_event_handler(registered.sio, 'signal')
    ice_data = {'candidate': 'candidate:1 udp 1234'}
    await signal({'type': 'ice-candidate', 'data': ice_data})
    registered.node._handle_ice_candidate.assert_called_once_with(ice_data)


async def test_error_handler_logs_error(registered):
    """Server-side errors must surface in the node logs."""
    error = get_event_handler(registered.sio, 'error')
    await error('something went wrong')
    registered.logger.error.assert_called_once()


# ---------------------------------------------------------------------------
# emit_message
# ---------------------------------------------------------------------------

async def test_emit_message_sends_correct_payload(handler):
    """emit_message must wrap type and data into the expected signal envelope."""
    handler.sio.emit = AsyncMock()
    handler.peer_id = 'peer-42'
    await handler.emit_message('offer', {'sdp': 'v=0...'})
    handler.sio.emit.assert_called_once_with(
        'signal',
        {'to': 'peer-42', 'type': 'offer', 'data': {'sdp': 'v=0...'}}
    )


# ---------------------------------------------------------------------------
# Backoff math
# ---------------------------------------------------------------------------

def test_backoff_delay_doubles_each_retry_up_to_max(handler):
    """
    Verify the exponential backoff formula used in connect_to_signaling_server
    without running the full async retry loop.
    Each failed attempt should double the wait time, capped at max_retry_delay.
    """
    delay = handler.initial_retry_delay
    expected_delays = [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]
    for expected in expected_delays:
        assert delay == expected
        delay = min(delay * handler.retry_backoff_factor, handler.max_retry_delay)


# ---------------------------------------------------------------------------
# connect_to_signaling_server
# ---------------------------------------------------------------------------

async def test_connect_to_signaling_server_connects_and_waits(handler):
    """On a successful connection attempt the handler must call sio.connect
    with the correct URL and then sio.wait to hold the session open.
    rclpy.ok() is given one True then False so the loop runs exactly once."""
    handler.sio.connect = AsyncMock()
    handler.sio.wait = AsyncMock()
    sys.modules['rclpy'].ok.side_effect = [True, False]
    try:
        await handler.connect_to_signaling_server()
    finally:
        sys.modules['rclpy'].ok.side_effect = None

    handler.sio.connect.assert_called_once_with(
        handler.signaling_url, transports=['websocket']
    )
    handler.sio.wait.assert_called_once()


async def test_connect_to_signaling_server_logs_error_and_sleeps_on_failure(handler):
    """When sio.connect raises, the handler must log the error and sleep
    for the retry delay before attempting again. Without the sleep the
    retry loop would spin at full CPU on a permanently unavailable server."""
    handler.sio.connect = AsyncMock(side_effect=Exception("connection refused"))
    sys.modules['rclpy'].ok.side_effect = [True, False]
    try:
        with patch('asyncio.sleep', AsyncMock()) as mock_sleep:
            await handler.connect_to_signaling_server()
    finally:
        sys.modules['rclpy'].ok.side_effect = None

    handler.logger.error.assert_called()
    mock_sleep.assert_called_once_with(handler.initial_retry_delay)
