import asyncio
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point32
from hawkeye_msgs.msg import TaggedImage
from object_detection import yolo_detection
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

        self.get_logger().info('object_detection node started')
        self.get_logger().info(f'Subscribed to TaggedImage: {TAGGED_IMAGE_TOPIC}')
        self.get_logger().info(f'Publishing annotated TaggedImage to: {PROCESSED_TOPIC}')
        
        self.object_detector = yolo_detection.YoloDetector(self)
        self.frame_count = 0


    def tagged_image_callback(self, tagged_img_msg: TaggedImage):
        """Enqueue incoming TaggedImage for async YOLO processing."""
        # self.get_logger().info('RECV: Got TaggedImage')

        self.frame_count += 1
        if self.frame_count % PROCESS_EVERY_N != 0:
            return
        self.frame_count = 0 
        color_msg = tagged_img_msg.image_data
        depth_msg = tagged_img_msg.depth_data

        # self.get_logger().info('PROCESS: Got TaggedImage — running prediction')
        try:
            
            start_time = time.time()
            prediction_result = self.object_detector.predict(color_msg)
            if len(prediction_result) == 0:
                return
            
            # only take first prediction
            # for item in prediction_result[0]:
            # self.get_logger().info(prediction_result[0])
            bounding_boxes, conf, color_name = prediction_result[0] 

            # self.get_logger().info(f'PROCESS: PREDICTED IMAGES {color_name}')

            # Convert list of ((x1,y1),(x2,y2)) tuples → flat Point32 list.
            # Each detection contributes two points: top-left then bottom-right.
            box_points = []
            x1, y1, x2, y2 = bounding_boxes
            box_points.append(Point32(x=float(x1), y=float(y1), z=0.0))
            box_points.append(Point32(x=float(x2), y=float(y2), z=0.0))

            color_map = {
                "red":    (255,   0,   0),
                "yellow": (255, 255,   0),
                "green":  (  0, 255,   0),
                "blue":   (  0,   0, 255),
                "black":  (  0,   0,   0),
                "white":  (255, 255, 255),
            }
            color_rgb = color_map.get(color_name, (1, 2, 3))  # (1,2,3) = unknown sentinel

            # Re-publish as annotated TaggedImage, preserving depth + metadata
            self.process_pub.publish(
                TaggedImage(
                    image_data=color_msg,
                    depth_data=depth_msg,
                    imu_orientation=tagged_img_msg.imu_orientation,
                    yaw_deg=tagged_img_msg.yaw_deg,
                    color_r=color_rgb[0],
                    color_g=color_rgb[1],
                    color_b=color_rgb[2],
                    bounding_box=box_points,
                    confidence_level=conf,
                )
            )
            
            end_time = time.time()
            processing_time = end_time - start_time
            self.get_logger().info(f'PROCESS: Image processing took {processing_time:.4f} seconds')

        except Exception as e:
            self.get_logger().error(f'PROCESS: Prediction error: {e}')


def main(args=None):
    """Entry point wrapper"""
    rclpy.init(args=args)
    node = ObjectDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()