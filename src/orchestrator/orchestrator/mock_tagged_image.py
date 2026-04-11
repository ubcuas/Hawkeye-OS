#!/usr/bin/env python3
"""
Mock Tagged Image Publisher

Publishes synthetic TaggedImage messages to object_detection/tagged_image
at 1 Hz to simulate infrequent object detection payloads.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import Point32, Quaternion
from sensor_msgs.msg import CompressedImage
from hawkeye_msgs.msg import TaggedImage


# Cycle through a few mock detections so the data is visually distinct
_MOCK_DETECTIONS = [
    {
        "color": (255, 0, 0),
        "bounding_box": [(100, 80), (300, 80), (300, 200), (100, 200)],
        "confidence": 0.92,
    },
    {
        "color": (0, 255, 0),
        "bounding_box": [(50, 50), (150, 200)],
        "confidence": 0.78,
    },
    {
        "color": (0, 0, 255),
        "bounding_box": [(200, 100), (280, 100), (280, 180), (200, 180), (240, 140)],
        "confidence": 0.85,
    },
]


class MockTaggedImagePublisher(Node):
    def __init__(self):
        super().__init__("mock_tagged_image")

        self.pub = self.create_publisher(TaggedImage, "object_detection/tagged_image", 10)
        self.timer = self.create_timer(1.0, self.publish_tagged_image)

        self._cycle_index = 0

        self.get_logger().info("Mock Tagged Image publisher started")
        self.get_logger().info("Publishing at 1 Hz on: object_detection/tagged_image")

    def publish_tagged_image(self):
        detection = _MOCK_DETECTIONS[self._cycle_index % len(_MOCK_DETECTIONS)]
        self._cycle_index += 1

        r, g, b = detection["color"]

        # Generate a small solid-colour image so the payload is non-trivial
        height, width = 240, 320
        now = self.get_clock().now().to_msg()

        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :] = [b, g, r]

        img_msg = CompressedImage()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "object_detection"
        img_msg.format = "jpeg"
        _, jpeg_data = cv2.imencode(".jpg", frame)
        img_msg.data = jpeg_data.tobytes()

        # Depth image: 16UC1 PNG gradient from 1m (top) to 5m (bottom)
        depth_frame = np.linspace(1000, 5000, height, dtype=np.uint16)
        depth_frame = np.tile(depth_frame.reshape(-1, 1), (1, width))

        depth_msg = CompressedImage()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "object_detection"
        depth_msg.format = "16UC1; png"
        _, depth_png = cv2.imencode(".png", depth_frame)
        depth_msg.data = depth_png.tobytes()

        msg = TaggedImage()
        msg.image_data = img_msg
        msg.depth_data = depth_msg
        msg.imu_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.yaw_deg = float((self._cycle_index * 45) % 360)  # fake yaw (deg)
        msg.color_r = r
        msg.color_g = g
        msg.color_b = b
        msg.confidence_level = float(detection["confidence"])
        msg.bounding_box = [
            Point32(x=float(x), y=float(y), z=0.0)
            for x, y in detection["bounding_box"]
        ]

        self.pub.publish(msg)
        # self.get_logger().info(
        #     f"Published TaggedImage — color=({r},{g},{b}) "
        #     f"vertices={len(msg.bounding_box)} confidence={msg.confidence_level}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = MockTaggedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
