#!/usr/bin/env python3
# One-shot publisher for /object_detection/tagged_image (debug / manual bridge checks).
# ros2 run drone_control test_tagged_image_pub — pair with bridge_detection + CameraInfo if needed.

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from hawkeye_msgs.msg import TaggedImage
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

_LOG_RULE = "=================================================="


class TestTaggedImagePublisher(Node):
    def __init__(self):
        super().__init__("test_tagged_image")

        # ==========================
        # Tunable parameters (inline below in publish_once)
        # ==========================

        # Synthetic detection Publisher
        self.pub = self.create_publisher(TaggedImage, "/object_detection/tagged_image", 1)

        # Single publish after 1 s (lets subscribers connect)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

        self.get_logger().info(
            "test_tagged_image: started. Will publish one TaggedImage on /object_detection/tagged_image."
        )

    def publish_once(self):
        if self.published:
            return

        depth_m = 11.5
        img_cx, img_cy = 320, 240
        fx, fy = 600.0, 600.0
        v_offset = img_cy - int(10.0 * fy / depth_m)

        msg = TaggedImage()

        # Dummy color image
        color_img = np.zeros((480, 640, 3), dtype=np.uint8)
        color_img[:] = (0, 255, 0)
        _, encoded_color = cv2.imencode(".png", color_img)
        msg.image_data = CompressedImage()
        msg.image_data.format = "png"
        msg.image_data.data = encoded_color.tobytes()

        # Dummy depth image (uint16 mm)
        depth_img = np.ones((480, 640), dtype=np.uint16) * int(depth_m * 1000)
        _, encoded_depth = cv2.imencode(".png", depth_img)
        msg.depth_data = CompressedImage()
        msg.depth_data.format = "png"
        msg.depth_data.data = encoded_depth.tobytes()

        # Bbox center -> ~11.5 m forward, ~10 m up in body frame (with matching intrinsics)
        msg.bounding_box = [
            Point32(x=float(img_cx - 5), y=float(v_offset - 5), z=0.0),
            Point32(x=float(img_cx + 5), y=float(v_offset + 5), z=0.0),
        ]

        msg.confidence_level = 0.9

        self.pub.publish(msg)
        self.published = True

        self.get_logger().info(_LOG_RULE)
        self.get_logger().info("TEST PUBLISH (tagged_image helper)")
        self.get_logger().info("Name: single synthetic TaggedImage")
        self.get_logger().info(
            f"Approx body target from depth+bbox: forward ~{depth_m} m, up ~10 m (needs matching CameraInfo)"
        )
        self.get_logger().info(_LOG_RULE)


def main(args=None):
    rclpy.init(args=args)
    node = TestTaggedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down test_tagged_image...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
