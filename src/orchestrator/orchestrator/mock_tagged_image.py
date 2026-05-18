#!/usr/bin/env python3
"""
Mock Tagged Image Publisher

Mirrors ImageProcessor behavior for the mock pipeline:
- Publishes synthetic TaggedImage to /image_processor/tagged_image at 1 Hz
  so the streaming node's capture cache is always populated.
- Also responds immediately to TAKE_PHOTO on /orchestrator/image_request,
  matching the real image_processor trigger flow.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import Point32, Quaternion
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from hawkeye_msgs.msg import TaggedImage


IMAGE_REQUEST_TOPIC = "/orchestrator/image_request"
OUTPUT_TOPIC = "/image_processor/tagged_image"

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

        self.pub = self.create_publisher(TaggedImage, OUTPUT_TOPIC, 10)
        self.timer = self.create_timer(1.0, self._update_and_publish)

        self.image_request_sub = self.create_subscription(
            String,
            IMAGE_REQUEST_TOPIC,
            self._on_image_request,
            10,
        )

        self._cycle_index = 0
        self._latest_msg: TaggedImage | None = None

        self.get_logger().info("Mock Tagged Image publisher started")
        self.get_logger().info(f"Publishing at 1 Hz on: {OUTPUT_TOPIC}")
        self.get_logger().info(f"Listening for TAKE_PHOTO on: {IMAGE_REQUEST_TOPIC}")

    def _build_tagged_image(self) -> TaggedImage:
        detection = _MOCK_DETECTIONS[self._cycle_index % len(_MOCK_DETECTIONS)]
        self._cycle_index += 1

        r, g, b = detection["color"]
        height, width = 1280, 720  # matches RealSense D435 default resolution
        now = self.get_clock().now().to_msg()

        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :] = [b, g, r]

        img_msg = CompressedImage()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "image_processor"
        img_msg.format = "jpeg"
        _, jpeg_data = cv2.imencode(".jpg", frame)
        img_msg.data = jpeg_data.tobytes()

        depth_frame = np.linspace(1000, 5000, height, dtype=np.uint16)
        depth_frame = np.tile(depth_frame.reshape(-1, 1), (1, width))

        depth_msg = CompressedImage()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "image_processor"
        depth_msg.format = "16UC1; png"
        _, depth_png = cv2.imencode(".png", depth_frame)
        depth_msg.data = depth_png.tobytes()

        msg = TaggedImage()
        msg.image_data = img_msg
        msg.depth_data = depth_msg
        msg.imu_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.yaw_deg = float((self._cycle_index * 45) % 360)
        msg.color_r = r
        msg.color_g = g
        msg.color_b = b
        msg.confidence_level = float(detection["confidence"])
        msg.bounding_box = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in detection["bounding_box"]
        ]
        return msg

    def _update_and_publish(self):
        self._latest_msg = self._build_tagged_image()
        self.pub.publish(self._latest_msg)

    def _on_image_request(self, msg: String):
        if msg.data != "TAKE_PHOTO":
            return
        self.get_logger().info("TAKE_PHOTO received — publishing tagged image")
        if self._latest_msg is None:
            self._latest_msg = self._build_tagged_image()
        self.pub.publish(self._latest_msg)


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
