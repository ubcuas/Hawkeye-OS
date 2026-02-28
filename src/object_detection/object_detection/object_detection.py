import asyncio
import os
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
from object_detection.yolo_detection import predict_images

"""
Object Detection node

Subscribes to depth and color image topics. Uses ApproximateTimeSynchronizer
to pair frames from both channels by timestamp before running YOLO detection.
"""

DEPTH_TOPIC = "/depth/image_rect_raw/compressed"
COLOR_TOPIC = "/color/image_raw/compressed"
PROCESSED_TOPIC = "/object_detection/image"

# ApproximateTimeSynchronizer settings
SYNC_QUEUE_SIZE = 10
SYNC_SLOP = 0.1  # Max time difference (seconds) between matched frames


class ObjectDetection(Node):
    def __init__(self):

        super().__init__("object_detection")

        # Async queue — synchronized pairs are enqueued here
        self.image_pair_queue = asyncio.Queue()

        # message_filters subscribers
        self.depth_sub = message_filters.Subscriber(self, CompressedImage, DEPTH_TOPIC)
        self.color_sub = message_filters.Subscriber(self, CompressedImage, COLOR_TOPIC)

        # publisher for combined image data
        self.process_pub =  self.create_publisher(
            Image,
            'object_detection/image',
            10
        )

        # Synchronize depth + color frames by approximate timestamp
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub],
            queue_size=SYNC_QUEUE_SIZE,
            slop=SYNC_SLOP,
        )
        self.sync.registerCallback(self.synchronized_callback)

        self.get_logger().info('object_detection node started')
        self.get_logger().info(f'Subscribed to depth: {DEPTH_TOPIC}')
        self.get_logger().info(f'Subscribed to color: {COLOR_TOPIC}')
        self.get_logger().info(f'Sync slop: {SYNC_SLOP}s')

    def synchronized_callback(self, depth_msg, color_msg):
        """Called when a matching depth+color pair arrives within slop window"""
        self.get_logger().info(
            f'SYNC: Matched pair — depth: {depth_msg.header.stamp}, '
            f'color: {color_msg.header.stamp}'
        )
        asyncio.run_coroutine_threadsafe(
            self.image_pair_queue.put((depth_msg, color_msg)),
            self.loop
        )

    async def process_images(self):
        """Dequeue synchronized pairs and run YOLO detection on both"""
        while rclpy.ok():
            self.get_logger().info('PROCESS: Waiting for synchronized image pair...')
            depth_msg, color_msg = await self.image_pair_queue.get()
            self.get_logger().info('PROCESS: Got pair — running prediction')
            try:
                bounding_boxes = predict_images(self, color_msg)
                self.get_logger().info(f'PROCESS: PREDICTED IMAGES {bounding_boxes}')



            except Exception as e:
                self.get_logger().error(f'PROCESS: Prediction error: {e}')


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    object_detection = ObjectDetection()
    object_detection.loop = asyncio.get_running_loop()

    executor = SingleThreadedExecutor()
    executor.add_node(object_detection)

    async def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            object_detection.process_images(),
        )
    except KeyboardInterrupt:
        pass
    finally:
        object_detection.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == '__main__':
    main()