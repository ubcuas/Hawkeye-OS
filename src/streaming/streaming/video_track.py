
import asyncio
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
import cv2
from sensor_msgs.msg import CompressedImage

class ROSVideoStreamTrack(VideoStreamTrack):
    """
    Custom video track that reads frames from a ROS topic via asyncio queue.
    Converts ROS Image messages to VideoFrames for WebRTC transmission.
    """

    def __init__(self, logger: RcutilsLogger):
        super().__init__()
        self.frame_queue = asyncio.Queue(maxsize=30)
        self._sent_frame_count = 0 # frames sent over WebRTC
        self._received_frame_count = 0 # frames recieved by object detection
        self.logger = logger

    async def recv(self):
        """
        Receive the next video frame.
        Called by aiortc when it needs a frame to send.
        """
        try:
            # Get frame from queue (blocks if empty)
            frame_data = await asyncio.wait_for(self.frame_queue.get(), timeout=1.0)

            # Convert numpy array to VideoFrame
            video_frame = VideoFrame.from_ndarray(frame_data, format="rgb24")

            # Set presentation timestamp
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            self._sent_frame_count += 1

            return video_frame

        except asyncio.TimeoutError:
            # No frame available, generate a blank frame
            blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            video_frame = VideoFrame.from_ndarray(blank_frame, format="rgb24")

            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            return video_frame
    
    def put_image(self, msg: CompressedImage):
        """
        Decodes a ROS CompressedImage (JPEG/PNG) and queues it as an RGB numpy
        array for aiortc to send over WebRTC.
        """
        # self.logger.info(f"Streaming Received CompressedImage: {msg.format}, size={len(msg.data)} bytes")
        try:
            self._received_frame_count += 1

            # Decode JPEG/PNG bytes → BGR frame
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame_data = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame_data is None:
                self.logger.error("put_image: imdecode returned None — invalid JPEG?")
                return

            # aiortc VideoFrame.from_ndarray expects RGB24
            frame_data = cv2.cvtColor(frame_data, cv2.COLOR_BGR2RGB)
            height, width, _ = frame_data.shape

            queue_size_before = self.frame_queue.qsize()
            self.frame_queue.put_nowait(frame_data)
            queue_size_after = self.frame_queue.qsize()

            if self._received_frame_count % 90 == 0:
                self.logger.info(
                    f"Frame queued for WebRTC. Queue: {queue_size_before} -> {queue_size_after}. "
                    f"Frame size: {height}x{width}"
                )

        except asyncio.QueueFull:
            pass  # Drop frame to avoid blocking
        except Exception as e:
            self.logger.error(f"Error processing CompressedImage: {e}")
