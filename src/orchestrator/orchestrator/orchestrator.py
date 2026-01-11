import asyncio
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image

"""
Orchestrator node

Coordinates image capture requests and object detection results.
"""


class Orchestrator(Node):
    def __init__(self,
                 object_detection_topic=None,
                 image_request_topic=None):

        super().__init__("orchestrator")

        # Load from environment variables
        self.object_detection_topic = object_detection_topic or os.getenv('OBJECT_DETECTION_TOPIC')
        self.image_request_topic = image_request_topic or os.getenv('IMAGE_REQUEST_TOPIC')

        # Async queues
        self.image_queue = asyncio.Queue()  # Images from object detection (object detection -> orchestrator)
        self.request_queue = asyncio.Queue()  # Image requests (orchestrator -> image capture)

        # ROS publishers and subscribers
        self.image_request_pub = self.create_publisher(String, self.image_request_topic, 10)
        self.object_detection_sub = self.create_subscription(Image, self.object_detection_topic, self.image_callback, 10)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Subscribing to: {self.object_detection_topic}')
        self.get_logger().info(f'Publishing to: {self.image_request_topic}')

    def image_callback(self, msg):
        """Callback when receiving images from object detection"""
        asyncio.run_coroutine_threadsafe(
            self.image_queue.put(msg),
            self.loop
        )
        self.get_logger().info('IMAGE CALLBACK: Added to queue')

    async def process_images(self):
        """Process images from object detection"""
        while rclpy.ok():
            self.get_logger().info('PROCESS: Waiting for image from queue...')
            img_msg = await self.image_queue.get()
            self.get_logger().info('PROCESS: Got image')
            # TODO: Add image processing logic here

    async def send_requests(self):
        """Send image requests to object detection"""
        while rclpy.ok():
            msg = await self.request_queue.get()
            self.get_logger().info(f'Requesting images: {msg.data}')
            self.image_request_pub.publish(msg)


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    orchestrator = Orchestrator()
    orchestrator.loop = asyncio.get_running_loop()

    executor = SingleThreadedExecutor()
    executor.add_node(orchestrator)

    async def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            orchestrator.process_images(),
            orchestrator.send_requests()
        )
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == '__main__':
    main()