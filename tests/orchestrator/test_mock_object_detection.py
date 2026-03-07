"""
Unit tests for orchestrator.mock_object_detection.MockObjectDetection
======================================================================

What we are testing
-------------------
MockObjectDetection is a ROS node that simulates a camera feed by reading
frames from a video file and publishing them at 30 FPS on the
object_detection/image topic. It has three responsibilities:

  1. Open a video file at startup, setting self.cap = None and logging a
     warning if the file is not found.

  2. publish_frame — read the next video frame, convert BGR→RGB, and
     publish it. Loop the video when it ends. Fall back to a black frame
     if both reads fail or an exception is raised.

  3. publish_black_frame — always publish a hardcoded 480x640 black frame
     as a fallback when no real video is available.

Test approach
-------------
- cv2 is mocked globally in conftest.py. We set specific return values on
  cv2.VideoCapture and cv2.cvtColor per-test to simulate different states.

- Two fixtures cover the two init paths:
    node_no_video  — VideoCapture.isOpened() returns False → self.cap = None
    node_with_video — VideoCapture.isOpened() returns True → self.cap is set

- fake_frame (autouse) makes cv2.cvtColor return a real numpy array so that
  `height, width, channels = cv_image.shape` works in publish_frame.
  Without this, MagicMock.shape cannot be unpacked into 3 values and the
  source raises ValueError before we can test anything meaningful.

- reset_cap (autouse) clears cap.read.side_effect between tests so that
  side_effect lists set in one test do not bleed into the next.
"""
import sys
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

from orchestrator.mock_object_detection import MockObjectDetection


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def fake_frame():
    """
    Make cv2.cvtColor return a real numpy array every test.

    The source does `height, width, channels = cv_image.shape` after calling
    cvtColor. MagicMock.shape is itself a MagicMock, which cannot be unpacked
    into exactly 3 values — Python raises ValueError. A real numpy array has
    a concrete .shape tuple so the unpack works correctly.
    """
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    sys.modules['cv2'].cvtColor.return_value = frame
    return frame


@pytest.fixture(autouse=True)
def reset_cap():
    """
    Clear cap.read.side_effect before and after each test.

    Several tests set side_effect to a list to simulate read() returning
    different values on successive calls (e.g. first call fails, second
    succeeds). If not cleared, that side_effect persists to the next test
    and makes cap.read behave unexpectedly.
    """
    cap = sys.modules['cv2'].VideoCapture.return_value
    cap.read.side_effect = None
    yield
    cap.read.side_effect = None


@pytest.fixture
def node_no_video():
    """
    Node constructed when the video file cannot be opened.
    isOpened() returns False so __init__ sets self.cap = None.
    """
    sys.modules['cv2'].VideoCapture.return_value.isOpened.return_value = False
    return MockObjectDetection()


@pytest.fixture
def node_with_video():
    """
    Node constructed when the video file is available.
    isOpened() returns True so __init__ keeps self.cap set.
    cap.get() returns 30.0 to simulate a 30 FPS source video.
    """
    cap = sys.modules['cv2'].VideoCapture.return_value
    cap.isOpened.return_value = True
    cap.get.return_value = 30.0
    return MockObjectDetection()


# ---------------------------------------------------------------------------
# __init__
# ---------------------------------------------------------------------------

def test_frame_count_starts_at_zero(node_no_video):
    """frame_count must be zero before any frames are published."""
    assert node_no_video.frame_count == 0


def test_no_video_sets_cap_to_none(node_no_video):
    """When the video file cannot be opened, self.cap must be set to None
    so publish_frame knows to fall back to black frames."""
    assert node_no_video.cap is None


def test_no_video_logs_warning(node_no_video):
    """A missing video file must be surfaced as a warning so operators
    know the node is running in fallback mode."""
    node_no_video.get_logger().warn.assert_called()


def test_valid_video_keeps_cap(node_with_video):
    """When the video file opens successfully, self.cap must remain set
    so publish_frame can read from it."""
    assert node_with_video.cap is not None


# ---------------------------------------------------------------------------
# publish_frame
# ---------------------------------------------------------------------------

def test_publish_frame_falls_back_to_black_when_cap_is_none(node_no_video):
    """With no video available (cap is None), publish_frame must delegate
    to publish_black_frame rather than attempting to read from a None cap."""
    with patch.object(node_no_video, 'publish_black_frame') as mock_black:
        node_no_video.publish_frame()
        mock_black.assert_called_once()


def test_publish_frame_falls_back_to_black_when_cap_not_opened(node_with_video):
    """If the video capture closes mid-run, publish_frame must fall back to
    black frames rather than crashing. We override isOpened after construction
    to simulate a capture that was open but then became unavailable."""
    node_with_video.cap.isOpened.return_value = False
    with patch.object(node_with_video, 'publish_black_frame') as mock_black:
        node_with_video.publish_frame()
        mock_black.assert_called_once()


def test_publish_frame_publishes_image_on_success(node_with_video):
    """A successful frame read must result in exactly one publish call
    on the image publisher."""
    node_with_video.cap.read.return_value = (True, MagicMock())
    node_with_video.publish_frame()
    node_with_video.image_pub.publish.assert_called_once()


def test_publish_frame_increments_frame_count(node_with_video):
    """frame_count must increase by 1 after each successfully published frame."""
    node_with_video.cap.read.return_value = (True, MagicMock())
    node_with_video.publish_frame()
    assert node_with_video.frame_count == 1


def test_publish_frame_sets_rgb8_encoding(node_with_video):
    """Frames must be published with rgb8 encoding — the streaming node
    downstream expects this specific format."""
    node_with_video.cap.read.return_value = (True, MagicMock())
    node_with_video.publish_frame()
    msg = node_with_video.image_pub.publish.call_args[0][0]
    assert msg.encoding == 'rgb8'


def test_publish_frame_restarts_video_on_end(node_with_video):
    """When the video ends (first read returns False), publish_frame must
    seek back to frame 0 and attempt a second read before giving up.
    We use side_effect to return False on the first call and True on the
    second, then confirm cap.set was called with frame index 0."""
    node_with_video.cap.read.side_effect = [(False, None), (True, MagicMock())]
    node_with_video.publish_frame()
    node_with_video.cap.set.assert_called_once_with(
        sys.modules['cv2'].CAP_PROP_POS_FRAMES, 0
    )


def test_publish_frame_falls_back_to_black_on_persistent_read_failure(node_with_video):
    """If both the initial read and the restart read fail, publish_frame must
    log an error and fall back to a black frame rather than publishing nothing."""
    node_with_video.cap.read.return_value = (False, None)
    with patch.object(node_with_video, 'publish_black_frame') as mock_black:
        node_with_video.publish_frame()
        mock_black.assert_called_once()
    node_with_video.get_logger().error.assert_called()


def test_publish_frame_falls_back_to_black_on_exception(node_with_video):
    """Any unexpected exception during frame processing must be caught,
    logged as an error, and result in a black frame being published
    so the pipeline always receives something."""
    node_with_video.cap.read.side_effect = Exception("camera hardware failure")
    with patch.object(node_with_video, 'publish_black_frame') as mock_black:
        node_with_video.publish_frame()
        mock_black.assert_called_once()
    node_with_video.get_logger().error.assert_called()


# ---------------------------------------------------------------------------
# publish_black_frame
# ---------------------------------------------------------------------------

def test_publish_black_frame_calls_publish(node_no_video):
    """publish_black_frame must always result in exactly one publish call
    so the video pipeline is never starved of frames."""
    node_no_video.publish_black_frame()
    node_no_video.image_pub.publish.assert_called_once()


def test_publish_black_frame_increments_frame_count(node_no_video):
    """frame_count must advance even for black frames so the counter
    accurately reflects total frames dispatched, not just real ones."""
    node_no_video.publish_black_frame()
    assert node_no_video.frame_count == 1


def test_publish_black_frame_correct_dimensions(node_no_video):
    """Black frames must be 480x640 to match the resolution expected
    by the streaming pipeline downstream."""
    node_no_video.publish_black_frame()
    msg = node_no_video.image_pub.publish.call_args[0][0]
    assert msg.height == 480
    assert msg.width == 640


def test_publish_black_frame_sets_rgb8_encoding(node_no_video):
    """Black frames must use the same rgb8 encoding as real video frames
    so the streaming node never sees a format mismatch."""
    node_no_video.publish_black_frame()
    msg = node_no_video.image_pub.publish.call_args[0][0]
    assert msg.encoding == 'rgb8'
