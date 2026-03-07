import sys
import asyncio
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

from streaming.video_track import ROSVideoStreamTrack


FAKE_PTS = 42
FAKE_TIME_BASE = "1/90000"


@pytest.fixture(autouse=True)
def reset_video_frame_mock():
    """Reset VideoFrame call history between tests."""
    sys.modules['av'].VideoFrame.from_ndarray.reset_mock()
    yield


@pytest.fixture
def track():
    t = ROSVideoStreamTrack(MagicMock())

    async def fake_next_timestamp():
        return (FAKE_PTS, FAKE_TIME_BASE)

    t.next_timestamp = fake_next_timestamp
    return t


def make_image_msg(height=480, width=640, encoding="rgb8"):
    """Build a minimal fake ROS Image message."""
    channels = 3 if encoding == "rgb8" else 1
    msg = MagicMock()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.data = np.zeros((height, width, channels), dtype=np.uint8).tobytes()
    return msg


# --- init ---

def test_initial_counters_are_zero(track):
    """Both frame counters must start at zero before any data flows through."""
    assert track._sent_frame_count == 0
    assert track._received_frame_count == 0


def test_frame_queue_maxsize(track):
    """Queue capacity must be 30 — exceeding it triggers the silent drop logic in put_image()."""
    assert track.frame_queue.maxsize == 30


# --- put_image() ---

def test_put_image_rgb8_queues_3_channel_array(track):
    """An rgb8 message should produce a (H, W, 3) array in the queue."""
    track.put_image(make_image_msg(encoding="rgb8"))
    array = track.frame_queue.get_nowait()
    assert array.shape == (480, 640, 3)


def test_put_image_mono_queues_1_channel_array(track):
    """A non-rgb8 message should produce a (H, W, 1) array — one channel, not three."""
    track.put_image(make_image_msg(encoding="mono8"))
    array = track.frame_queue.get_nowait()
    assert array.shape == (480, 640, 1)


def test_put_image_increments_received_counter(track):
    """Every call to put_image() counts as one received frame regardless of queue state."""
    for _ in range(3):
        track.put_image(make_image_msg())
    assert track._received_frame_count == 3


def test_put_image_full_queue_drops_frame_silently(track):
    """When the queue is full, put_image() must drop the frame without raising an exception."""
    for _ in range(30):
        track.put_image(make_image_msg())
    track.put_image(make_image_msg())  # 31st frame — should be dropped
    assert track.frame_queue.qsize() == 30


def test_put_image_preserves_pixel_data(track):
    """Byte values in the ROS message must survive the frombuffer+reshape conversion unchanged."""
    channels = 3
    raw = np.random.randint(0, 255, (480, 640, channels), dtype=np.uint8)
    msg = MagicMock()
    msg.height = 480
    msg.width = 640
    msg.encoding = "rgb8"
    msg.data = raw.tobytes()
    track.put_image(msg)
    result = track.frame_queue.get_nowait()
    assert np.array_equal(result, raw)


def test_put_image_logs_error_on_bad_data(track):
    """If the data buffer does not match the declared dimensions, logger.error must be called."""
    msg = MagicMock()
    msg.height = 480
    msg.width = 640
    msg.encoding = "rgb8"
    msg.data = b"tooshort"  # will cause reshape to raise
    track.put_image(msg)
    track.logger.error.assert_called_once()


def test_put_image_logs_info_every_90_frames(track):
    """logger.info should fire on frame 90 but not on frame 89.
    The queue is drained after each put so QueueFull never silently
    short-circuits the logging branch before we reach frame 90."""
    for _ in range(89):
        track.put_image(make_image_msg())
        track.frame_queue.get_nowait()  # keep queue clear
    track.logger.info.assert_not_called()
    track.put_image(make_image_msg())  # 90th frame
    track.logger.info.assert_called_once()


# --- recv() ---

async def test_recv_increments_sent_counter(track):
    """Each successful frame delivery should increment _sent_frame_count."""
    await track.frame_queue.put(np.zeros((480, 640, 3), dtype=np.uint8))
    await track.recv()
    assert track._sent_frame_count == 1


async def test_recv_creates_video_frame_from_queued_data(track):
    """The exact numpy array put in the queue should be passed to VideoFrame.from_ndarray as rgb24."""
    VideoFrame = sys.modules['av'].VideoFrame
    frame_data = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_data[0, 0, 0] = 99  # make the array identifiable
    await track.frame_queue.put(frame_data)
    await track.recv()
    called_array, = VideoFrame.from_ndarray.call_args.args
    assert np.array_equal(called_array, frame_data)
    assert VideoFrame.from_ndarray.call_args.kwargs['format'] == "rgb24"


async def test_recv_timeout_sends_blank_frame(track):
    """When the queue is empty, recv() should fall back to a 480x640 black frame rather than stalling."""
    VideoFrame = sys.modules['av'].VideoFrame

    async def fake_wait_for(*args, **kwargs):
        raise asyncio.TimeoutError()

    with patch('asyncio.wait_for', fake_wait_for):
        await track.recv()

    called_array, = VideoFrame.from_ndarray.call_args.args
    assert called_array.shape == (480, 640, 3)
    assert np.all(called_array == 0)
    assert VideoFrame.from_ndarray.call_args.kwargs['format'] == "rgb24"


async def test_recv_timeout_does_not_increment_sent_counter(track):
    """A blank fallback frame is not a real delivery — the sent counter should not move."""
    async def fake_wait_for(*args, **kwargs):
        raise asyncio.TimeoutError()

    with patch('asyncio.wait_for', fake_wait_for):
        await track.recv()

    assert track._sent_frame_count == 0


async def test_recv_happy_path_sets_pts_and_time_base(track):
    """Returned VideoFrame must have pts and time_base set so aiortc can sequence it correctly."""
    await track.frame_queue.put(np.zeros((480, 640, 3), dtype=np.uint8))
    result = await track.recv()
    assert result.pts == FAKE_PTS
    assert result.time_base == FAKE_TIME_BASE


async def test_recv_timeout_sets_pts_and_time_base(track):
    """Blank fallback frames also need pts and time_base — aiortc requires every frame to be timestamped."""
    async def fake_wait_for(*args, **kwargs):
        raise asyncio.TimeoutError()

    with patch('asyncio.wait_for', fake_wait_for):
        result = await track.recv()

    assert result.pts == FAKE_PTS
    assert result.time_base == FAKE_TIME_BASE
