"""
Node communication contract tests
==================================

Verifies that every ROS2 node in the system registers the correct
publishers, subscribers, and timers during initialisation.

These are not integration tests — no messages actually flow between nodes.
Instead, _StubNode (defined in conftest.py) records every
create_subscription / create_publisher / create_timer call made during
__init__, and the tests assert on that recorded state.

The most important test here is test_object_detection_pipeline_contract,
which checks that the one topic connecting ImageProcessor → Orchestrator
and ImageProcessor → StreamingNode uses the same name and message type
on both ends. A typo in any of those three nodes would break the live
video pipeline silently at runtime; this test makes it fail loudly.
"""
import sys
import pytest
from unittest.mock import MagicMock

from sensor_msgs.msg import Image
from std_msgs.msg import String

from orchestrator.orchestrator import Orchestrator
from streaming.streaming import StreamingNode
from imaging_realsense.image_processor import ImageProcessor


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def orchestrator():
    # Pass topics directly so the node doesn't need env vars.
    return Orchestrator(
        loop=MagicMock(),
        object_detection_topic='object_detection/image',
        image_request_topic='image_request',
    )


@pytest.fixture
def streaming_node():
    return StreamingNode('ws://test.local')


@pytest.fixture
def image_processor():
    return ImageProcessor()


# ---------------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------------

def test_orchestrator_subscribes_to_object_detection(orchestrator):
    """Orchestrator must listen for processed images on the correct topic."""
    sub = orchestrator.subscription('object_detection/image')
    assert sub is not None
    assert sub['msg_type'] == Image
    assert sub['callback'] == orchestrator.image_callback


def test_orchestrator_publishes_image_request(orchestrator):
    """Orchestrator must be able to send image capture commands downstream."""
    pub = orchestrator.publisher('image_request')
    assert pub is not None
    assert pub['msg_type'] == String


def test_orchestrator_registers_status_heartbeat_timer(orchestrator):
    """A 1-second heartbeat timer must be registered to keep GCOM informed."""
    assert len(orchestrator._timers) > 0
    timer = orchestrator._timers[0]
    assert timer['period'] == 1.0
    assert timer['callback'] == orchestrator.publish_status_to_ground


# ---------------------------------------------------------------------------
# StreamingNode
# ---------------------------------------------------------------------------

def test_streaming_subscribes_to_object_detection(streaming_node):
    """StreamingNode must receive frames from the object detection pipeline."""
    sub = streaming_node.subscription('object_detection/image')
    assert sub is not None
    assert sub['msg_type'] == Image
    assert sub['callback'] == streaming_node._route_image_to_track


# ---------------------------------------------------------------------------
# ImageProcessor
# ---------------------------------------------------------------------------

def test_image_processor_subscribes_to_camera(image_processor):
    """ImageProcessor must read raw frames from the RealSense camera topic."""
    sub = image_processor.subscription('/camera/camera/color/image_raw')
    assert sub is not None
    assert sub['msg_type'] == Image


def test_image_processor_subscribes_to_imu(image_processor):
    """ImageProcessor must cache IMU data for geotag synchronisation."""
    from sensor_msgs.msg import Imu
    sub = image_processor.subscription('/camera/camera/imu')
    assert sub is not None
    assert sub['msg_type'] == Imu


def test_image_processor_subscribes_to_gps(image_processor):
    """ImageProcessor must cache GPS fixes for geotag synchronisation."""
    from sensor_msgs.msg import NavSatFix
    sub = image_processor.subscription('/gps/fix')
    assert sub is not None
    assert sub['msg_type'] == NavSatFix


def test_image_processor_publishes_to_object_detection(image_processor):
    """ImageProcessor must forward frames to the object detection topic."""
    pub = image_processor.publisher('object_detection/image')
    assert pub is not None
    assert pub['msg_type'] == Image


def test_image_processor_publishes_detections(image_processor):
    """ImageProcessor must publish detection results for downstream consumers."""
    pub = image_processor.publisher('/detections')
    assert pub is not None
    assert pub['msg_type'] == String


# ---------------------------------------------------------------------------
# Cross-node pipeline contract
# ---------------------------------------------------------------------------

def test_object_detection_pipeline_contract(image_processor, orchestrator, streaming_node):
    """
    The object_detection/image topic must be wired consistently across all
    three nodes that touch it.  A mismatch in topic name or message type
    between the producer (ImageProcessor) and either consumer (Orchestrator,
    StreamingNode) would cause a silent failure in the live video pipeline.
    """
    produced = image_processor.publisher('object_detection/image')
    consumed_by_orchestrator = orchestrator.subscription('object_detection/image')
    consumed_by_streaming = streaming_node.subscription('object_detection/image')

    assert produced is not None, "ImageProcessor does not publish object_detection/image"
    assert consumed_by_orchestrator is not None, "Orchestrator does not subscribe to object_detection/image"
    assert consumed_by_streaming is not None, "StreamingNode does not subscribe to object_detection/image"

    assert produced['msg_type'] == consumed_by_orchestrator['msg_type'] == consumed_by_streaming['msg_type'], \
        "Message type mismatch on object_detection/image across nodes"
