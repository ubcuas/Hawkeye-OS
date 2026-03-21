import asyncio
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point32
from hawkeye_msgs.msg import TaggedImage
from object_detection.yolo_detection import predict_images

import time

"""
Object Detection node

Subscribes to the TaggedImage topic published by image_processor.
Each message already contains synchronized color + depth data.
YOLO detection runs on the color frame and results are re-published
as an annotated TaggedImage.
"""

TAGGED_IMAGE_TOPIC = "/image_processor/tagged_image"
PROCESSED_TOPIC = "/object_detection/tagged_image"
PROCESS_EVERY_N = 5


class ObjectDetection(Node):
    def __init__(self):

        super().__init__("object_detection")

        # Async queue — incoming TaggedImage messages are enqueued here
        self.image_queue = asyncio.Queue()

        self.frame_count = 0

        # Subscribe to the combined TaggedImage from image_processor
        self.tagged_image_sub = self.create_subscription(
            TaggedImage,
            TAGGED_IMAGE_TOPIC,
            self.tagged_image_callback,
            10,
        )

        # Publisher for the annotated TaggedImage
        self.process_pub = self.create_publisher(
            TaggedImage,
            PROCESSED_TOPIC,
            10,
        )

        self.last_time = time.time()
        self.curr_time = self.last_time

        self.get_logger().info('object_detection node started')
        self.get_logger().info(f'Subscribed to TaggedImage: {TAGGED_IMAGE_TOPIC}')
        self.get_logger().info(f'Publishing annotated TaggedImage to: {PROCESSED_TOPIC}')

    def tagged_image_callback(self, msg: TaggedImage):
        """Enqueue incoming TaggedImage for async YOLO processing."""
        self.frame_count += 1
        if self.frame_count % PROCESS_EVERY_N != 0:
            return
        else:
            self.get_logger().info('RECV: Got TaggedImage — enqueueing for detection')
            asyncio.run_coroutine_threadsafe(
                self.image_queue.put(msg),
                self.loop,
            )

    async def process_images(self):
        """Dequeue TaggedImage messages and run YOLO detection on the color frame."""
        while rclpy.ok():

            self.curr_time = time.time()
            
            #self.get_logger().info('PROCESS: Waiting for TaggedImage...')
            tagged_msg: TaggedImage = await self.image_queue.get()

            color_msg = tagged_msg.image_data
            depth_msg = tagged_msg.depth_data

            # self.get_logger().info('PROCESS: Got TaggedImage — running prediction')
            try:
                prediction_result = predict_images(self, color_msg)
                if len(prediction_result) == 0:
                    continue
                
                # only take first prediction
                # for item in prediction_result[0]:
                # self.get_logger().info(prediction_result[0])
                bounding_boxes, conf, color_name = prediction_result[0] 

                self.get_logger().info(f'PROCESS: PREDICTED IMAGES {color_name}')

                # Convert list of ((x1,y1),(x2,y2)) tuples → flat Point32 list.
                # Each detection contributes two points: top-left then bottom-right.
                box_points = []
                x1, y1, x2, y2 = bounding_boxes
                box_points.append(Point32(x=float(x1), y=float(y1), z=0.0))
                box_points.append(Point32(x=float(x2), y=float(y2), z=0.0))

                color_map = {
                    "blue": (0, 0, 255),
                    "green": (0, 255, 0),
                    "red": (255, 0, 0),
                }
                color_rgb = color_map.get(color_name, (1,2,3))

                # Re-publish as annotated TaggedImage, preserving depth + metadata
                self.process_pub.publish(
                    TaggedImage(
                        image_data=color_msg,
                        depth_data=depth_msg,
                        imu_data=tagged_msg.imu_data,
                        gps_data=tagged_msg.gps_data,
                        color_r=color_rgb[0],
                        color_g=color_rgb[1],
                        color_b=color_rgb[2],
                        bounding_box=box_points,
                        confidence_level=conf,
                    )
                )

            except Exception as e:
                self.get_logger().error(f'PROCESS: Prediction error: {e}')

            time_taken = self.curr_time - self.last_time
            self.last_time = self.curr_time
            self.get_logger().info(f"processing time {time_taken}")


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