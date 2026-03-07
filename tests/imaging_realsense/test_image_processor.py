"""
Unit tests for imaging_realsense.image_processor.ImageProcessor
================================================================

What we are testing
-------------------
ImageProcessor sits at the entry point of the imaging pipeline. It receives
raw frames from the RealSense camera and GPS/IMU data from their respective
topics, then re-publishes the converted frame for downstream consumers
(Orchestrator and StreamingNode). It has four responsibilities:

  1. Initial state — sensible defaults before any data arrives.
  2. image_callback — convert the raw ROS Image to OpenCV format via
     CvBridge, write a debug snapshot to disk, convert back to a ROS
     message, copy the original header, and publish. Catch and log any
     conversion failure without crashing.
  3. imu_callback — cache the latest IMU message.
  4. gps_callback — cache the latest GPS fix and log the coordinates.

Test approach
-------------
- fresh_bridge (autouse): cv_bridge is mocked globally in conftest.py as a
  single MagicMock, so CvBridge() always returns the same return_value.
  Every ImageProcessor instance across all tests would share one bridge mock
  and call history would bleed. fresh_bridge sets side_effect to a factory
  lambda so each ImageProcessor gets its own independent CvBridge mock —
  the same pattern used for mqtt.Client in test_orchestrator.py and
  socketio.AsyncClient in test_signaling_handler.py.

- gps_callback uses :.6f and :.2f format specs on msg.latitude/longitude/
  altitude. Formatting a MagicMock with a numeric format spec can behave
  unpredictably, so the GPS test gives the message real float values.

- Pub/sub wiring (which topics the node subscribes and publishes to) is
  already covered exhaustively in test_node_contracts.py and is not
  repeated here.
"""
import sys
import pytest
from unittest.mock import MagicMock

from imaging_realsense.image_processor import ImageProcessor


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fresh_bridge():
    """
    Give each test its own independent CvBridge mock.

    cv_bridge is a single global MagicMock. Without this fixture, every
    ImageProcessor() call returns the same bridge mock and call counts from
    one test bleed into the next.
    """
    sys.modules['cv_bridge'].CvBridge.side_effect = lambda: MagicMock()
    yield
    sys.modules['cv_bridge'].CvBridge.side_effect = None


@pytest.fixture
def node():
    return ImageProcessor()


# ---------------------------------------------------------------------------
# __init__
# ---------------------------------------------------------------------------

def test_latest_gps_starts_none(node):
    """No GPS fix is cached before the first gps_callback fires."""
    assert node.latest_gps is None


def test_latest_imu_starts_none(node):
    """No IMU reading is cached before the first imu_callback fires."""
    assert node.latest_imu is None


def test_gps_lock_starts_none(node):
    """The threading lock placeholder is None until explicitly initialized."""
    assert node.gps_lock is None


# ---------------------------------------------------------------------------
# image_callback
# ---------------------------------------------------------------------------

def test_image_callback_converts_with_bgr8_encoding(node):
    """imgmsg_to_cv2 must be called with 'bgr8' encoding so the resulting
    OpenCV array has the correct channel ordering for downstream processing."""
    msg = MagicMock()
    node.image_callback(msg)
    node.bridge.imgmsg_to_cv2.assert_called_once_with(msg, "bgr8")


def test_image_callback_converts_cv_image_back_to_ros(node):
    """The OpenCV image produced by imgmsg_to_cv2 must be passed to
    cv2_to_imgmsg with 'bgr8' encoding to create the output ROS message."""
    msg = MagicMock()
    node.image_callback(msg)
    expected_cv_image = node.bridge.imgmsg_to_cv2.return_value
    node.bridge.cv2_to_imgmsg.assert_called_once_with(expected_cv_image, "bgr8")


def test_image_callback_copies_header_to_output(node):
    """The output message must carry the original header so downstream
    nodes can use the original timestamp and frame_id for synchronisation."""
    msg = MagicMock()
    node.image_callback(msg)
    output_msg = node.bridge.cv2_to_imgmsg.return_value
    assert output_msg.header == msg.header


def test_image_callback_publishes_output_message(node):
    """After conversion the output message must be published so Orchestrator
    and StreamingNode receive it."""
    msg = MagicMock()
    node.image_callback(msg)
    expected_output = node.bridge.cv2_to_imgmsg.return_value
    node.image_publisher.publish.assert_called_once_with(expected_output)


def test_image_callback_logs_error_on_conversion_failure(node):
    """If CvBridge raises during conversion the exception must be caught
    and logged — the node must not crash or propagate the exception."""
    node.bridge.imgmsg_to_cv2.side_effect = Exception("bad encoding")
    node.image_callback(MagicMock())  # must not raise
    node.get_logger().error.assert_called()


def test_image_callback_does_not_publish_on_conversion_failure(node):
    """When conversion fails nothing should be published — sending a
    corrupt or partially built message downstream would break the pipeline."""
    node.bridge.imgmsg_to_cv2.side_effect = Exception("bad encoding")
    node.image_callback(MagicMock())
    node.image_publisher.publish.assert_not_called()


# ---------------------------------------------------------------------------
# imu_callback
# ---------------------------------------------------------------------------

def test_imu_callback_caches_message(node):
    """The latest IMU reading must be stored so image_callback can use it
    for geotag synchronisation."""
    msg = MagicMock()
    node.imu_callback(msg)
    assert node.latest_imu is msg


def test_imu_callback_overwrites_previous_reading(node):
    """Each new IMU message must replace the old one — only the most
    recent reading is relevant for synchronisation."""
    first = MagicMock()
    second = MagicMock()
    node.imu_callback(first)
    node.imu_callback(second)
    assert node.latest_imu is second


# ---------------------------------------------------------------------------
# gps_callback
# ---------------------------------------------------------------------------

def test_gps_callback_caches_message(node):
    """The latest GPS fix must be stored so image_callback can geotag
    frames with the correct position."""
    msg = MagicMock()
    msg.latitude = 49.2827
    msg.longitude = -123.1207
    msg.altitude = 100.0
    node.gps_callback(msg)
    assert node.latest_gps is msg


def test_gps_callback_overwrites_previous_fix(node):
    """Each new GPS fix must replace the old one — only the most recent
    position is relevant."""
    first = MagicMock()
    first.latitude = 0.0
    first.longitude = 0.0
    first.altitude = 0.0
    second = MagicMock()
    second.latitude = 49.2827
    second.longitude = -123.1207
    second.altitude = 100.0
    node.gps_callback(first)
    node.gps_callback(second)
    assert node.latest_gps is second


def test_gps_callback_logs_coordinates(node):
    """GPS updates must be logged at debug level so developers can verify
    the fix is being received without flooding the console at runtime."""
    msg = MagicMock()
    msg.latitude = 49.2827
    msg.longitude = -123.1207
    msg.altitude = 100.0
    node.gps_callback(msg)
    node.get_logger().debug.assert_called()
