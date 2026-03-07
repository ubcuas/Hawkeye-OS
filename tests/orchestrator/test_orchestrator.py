"""
Unit tests for orchestrator.orchestrator.Orchestrator
======================================================

What we are testing
-------------------
Orchestrator is the central coordination node. It:

  1. Connects to GCOM via MQTT and subscribes to commands on success.
  2. Parses incoming MQTT messages and schedules payloads onto its
     asyncio queue when a valid action is present.
  3. Publishes a periodic status heartbeat to GCOM over MQTT.
  4. Receives ROS Image messages and forwards them to its asyncio queue.
  5. Routes requests from either ROS or GCOM to the right handlers.
  6. Publishes command acknowledgements back to GCOM over MQTT.

Test approach
-------------
- paho.mqtt.client is mocked globally in conftest.py. The fresh_mqtt_client
  autouse fixture makes each test get its own independent mqtt_client mock
  so call history never bleeds between tests (same pattern as test_signaling_handler.py).
- asyncio.run_coroutine_threadsafe is patched in tests that exercise
  image_callback and on_mqtt_message, because the orchestrator's `loop`
  is a MagicMock and cannot actually schedule coroutines.
- handle_request is async, so those tests are async def. asyncio_mode = auto
  in pytest.ini runs them without any extra decorator.
"""
import sys
import json
import pytest
from unittest.mock import MagicMock, patch

from orchestrator.orchestrator import (
    Orchestrator,
    MQTT_TOPIC_CMD,
    MQTT_TOPIC_STATUS,
    MQTT_TOPIC_CMD_ACK,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fresh_mqtt_client():
    """
    Give each test its own independent MQTT client mock.

    paho.mqtt.client is a single MagicMock set in conftest.py.
    mqtt.Client(...) always returns the same .return_value child mock,
    so call history from one test would leak into the next.
    Setting side_effect to a factory lambda means every call to
    mqtt.Client(...) returns a brand-new MagicMock instead.
    """
    sys.modules['paho.mqtt.client'].Client.side_effect = lambda *a, **kw: MagicMock()
    yield
    sys.modules['paho.mqtt.client'].Client.side_effect = None


@pytest.fixture
def orchestrator():
    """
    Construct an Orchestrator with topic names passed directly so the node
    does not depend on environment variables being set during tests.
    The loop is a MagicMock — we never actually schedule coroutines in these
    unit tests; we only assert the scheduling call was made.
    """
    return Orchestrator(
        loop=MagicMock(),
        object_detection_topic='object_detection/image',
        image_request_topic='image_request',
    )


# ---------------------------------------------------------------------------
# on_mqtt_connect
# ---------------------------------------------------------------------------

def test_on_mqtt_connect_success_subscribes_to_cmd_topic(orchestrator):
    """rc=0 means the broker accepted the connection; the client must
    subscribe to the command topic so GCOM commands are received."""
    client = MagicMock()
    orchestrator.on_mqtt_connect(client, None, None, 0, None)
    client.subscribe.assert_called_once_with(MQTT_TOPIC_CMD)


def test_on_mqtt_connect_failure_does_not_subscribe(orchestrator):
    """A non-zero rc means connection was rejected; no subscription
    should be created — there is nothing to subscribe on."""
    client = MagicMock()
    orchestrator.on_mqtt_connect(client, None, None, 1, None)
    client.subscribe.assert_not_called()


def test_on_mqtt_connect_failure_logs_error(orchestrator):
    """A failed connection must surface through the logger so operators
    can see that the GCOM link is down."""
    client = MagicMock()
    orchestrator.on_mqtt_connect(client, None, None, 5, None)
    orchestrator.get_logger().error.assert_called()


# ---------------------------------------------------------------------------
# on_mqtt_message
# ---------------------------------------------------------------------------

def test_on_mqtt_message_with_action_enqueues_payload(orchestrator):
    """A message that contains an 'action' key must be scheduled onto
    the asyncio image_queue for the main loop to process."""
    msg = MagicMock()
    msg.payload = json.dumps({'action': 'TAKE_PHOTO'}).encode()
    with patch('asyncio.run_coroutine_threadsafe') as mock_rcts:
        orchestrator.on_mqtt_message(None, None, msg)
        mock_rcts.assert_called_once()


def test_on_mqtt_message_uses_node_event_loop(orchestrator):
    """The coroutine must be submitted to the orchestrator's own event loop,
    not any other loop that might be running in the process."""
    msg = MagicMock()
    msg.payload = json.dumps({'action': 'TAKE_PHOTO'}).encode()
    with patch('asyncio.run_coroutine_threadsafe') as mock_rcts:
        orchestrator.on_mqtt_message(None, None, msg)
        submitted_loop = mock_rcts.call_args[0][1]
        assert submitted_loop is orchestrator.loop


def test_on_mqtt_message_without_action_does_not_enqueue(orchestrator):
    """A payload that has no 'action' key is not a recognised command
    and must be silently ignored — nothing should go on the queue."""
    msg = MagicMock()
    msg.payload = json.dumps({'info': 'telemetry only'}).encode()
    with patch('asyncio.run_coroutine_threadsafe') as mock_rcts:
        orchestrator.on_mqtt_message(None, None, msg)
        mock_rcts.assert_not_called()


def test_on_mqtt_message_invalid_json_logs_error(orchestrator):
    """Malformed JSON from the broker must be caught and logged as an
    error rather than crashing the node."""
    msg = MagicMock()
    msg.payload = b'not valid json {{{'
    orchestrator.on_mqtt_message(None, None, msg)
    orchestrator.get_logger().error.assert_called()


# ---------------------------------------------------------------------------
# publish_status_to_ground
# ---------------------------------------------------------------------------

def test_publish_status_sends_to_status_topic(orchestrator):
    """The heartbeat must be published to the MQTT status topic
    so GCOM can monitor drone health."""
    orchestrator.publish_status_to_ground()
    topic = orchestrator.mqtt_client.publish.call_args[0][0]
    assert topic == MQTT_TOPIC_STATUS


def test_publish_status_payload_is_valid_json(orchestrator):
    """GCOM parses the payload as JSON; it must always be decodable."""
    orchestrator.publish_status_to_ground()
    payload = orchestrator.mqtt_client.publish.call_args[0][1]
    data = json.loads(payload)  # raises if not valid JSON
    assert isinstance(data, dict)


def test_publish_status_payload_reports_online(orchestrator):
    """The 'status' field must be 'ONLINE' so GCOM knows the drone
    is alive and responsive."""
    orchestrator.publish_status_to_ground()
    payload = orchestrator.mqtt_client.publish.call_args[0][1]
    data = json.loads(payload)
    assert data['status'] == 'ONLINE'


def test_publish_status_payload_includes_queue_size(orchestrator):
    """GCOM uses queue_size to see how many images are waiting to be
    processed — it must be present in every heartbeat."""
    orchestrator.publish_status_to_ground()
    payload = orchestrator.mqtt_client.publish.call_args[0][1]
    data = json.loads(payload)
    assert 'queue_size' in data


# ---------------------------------------------------------------------------
# image_callback
# ---------------------------------------------------------------------------

def test_image_callback_schedules_enqueue_on_loop(orchestrator):
    """image_callback is called from the ROS executor thread. It must
    use run_coroutine_threadsafe to hand the message safely to the
    asyncio loop running on the main thread."""
    msg = MagicMock()
    with patch('asyncio.run_coroutine_threadsafe') as mock_rcts:
        orchestrator.image_callback(msg)
        mock_rcts.assert_called_once()
        assert mock_rcts.call_args[0][1] is orchestrator.loop


# ---------------------------------------------------------------------------
# handle_request
# ---------------------------------------------------------------------------

async def test_handle_request_ros_message_puts_response_on_outgoing_queue(orchestrator):
    """A ROS message (has a .data attribute) should produce a response
    that is queued for the send_messages loop to publish."""
    msg = MagicMock()
    msg.data = 'target_42'
    await orchestrator.handle_request(msg)
    assert orchestrator.outgoing_queue.qsize() == 1


async def test_handle_request_gcom_take_photo_calls_publish_ack(orchestrator):
    """A GCOM dict command with action='TAKE_PHOTO' must trigger an
    acknowledgement back to GCOM confirming the command was received."""
    with patch.object(orchestrator, 'publish_ack') as mock_ack:
        await orchestrator.handle_request({'action': 'TAKE_PHOTO'})
        mock_ack.assert_called_once()


# ---------------------------------------------------------------------------
# publish_ack
# ---------------------------------------------------------------------------

def test_publish_ack_sends_to_ack_topic(orchestrator):
    """ACK messages must go to the command acknowledgement topic
    so GCOM knows which commands have been acted on."""
    orchestrator.publish_ack('TAKE_PHOTO', 'COMMAND_RECEIVED')
    topic = orchestrator.mqtt_client.publish.call_args[0][0]
    assert topic == MQTT_TOPIC_CMD_ACK


def test_publish_ack_payload_contains_action_and_status(orchestrator):
    """The ACK payload must echo back the action and status so GCOM
    can correlate it with the original command."""
    orchestrator.publish_ack('TAKE_PHOTO', 'COMMAND_RECEIVED')
    payload = orchestrator.mqtt_client.publish.call_args[0][1]
    data = json.loads(payload)
    assert data['action'] == 'TAKE_PHOTO'
    assert data['status'] == 'COMMAND_RECEIVED'


def test_publish_ack_payload_includes_optional_detail(orchestrator):
    """When a detail string is provided it must appear in the payload
    so GCOM can display a human-readable reason."""
    orchestrator.publish_ack('TAKE_PHOTO', 'COMMAND_RECEIVED', detail='all clear')
    payload = orchestrator.mqtt_client.publish.call_args[0][1]
    data = json.loads(payload)
    assert data['detail'] == 'all clear'


# ---------------------------------------------------------------------------
# send_messages
# ---------------------------------------------------------------------------

async def test_send_messages_publishes_message_from_outgoing_queue(orchestrator):
    """send_messages must dequeue each outgoing message and publish it via
    image_request_pub. rclpy.ok() is given one True then False so the loop
    runs exactly once without spinning forever."""
    msg = MagicMock()
    await orchestrator.outgoing_queue.put(msg)
    sys.modules['rclpy'].ok.side_effect = [True, False]
    try:
        await orchestrator.send_messages()
    finally:
        sys.modules['rclpy'].ok.side_effect = None
    orchestrator.image_request_pub.publish.assert_called_once_with(msg)
