#!/usr/bin/env python3
"""
Mock Tagged Image Publisher

Publishes synthetic TaggedImage messages to object_detection/tagged_image
at 1 Hz to simulate infrequent object detection payloads.
"""

import colorsys

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from hawkeye_msgs.msg import TaggedImage


# Bounding box shapes to cycle through so the data is visually distinct
_MOCK_BOUNDING_BOXES = [
    {"vertices": [(100, 80), (300, 80), (300, 200), (100, 200)], "confidence": 92},
    {"vertices": [(50, 50), (150, 200)], "confidence": 78},
    {"vertices": [(200, 100), (280, 100), (280, 180), (200, 180), (240, 140)], "confidence": 85},
]

# Hue increment per tick (0.0-1.0 scale). 0.02 = 50 steps for a full rainbow.
_HUE_STEP = 0.02


class MockTaggedImagePublisher(Node):
    def __init__(self):
        super().__init__("mock_tagged_image")

        self.pub = self.create_publisher(TaggedImage, "object_detection/tagged_image", 10)
        self.timer = self.create_timer(1.0, self.publish_tagged_image)

        self._cycle_index = 0
        self._hue = 0.0

        self.get_logger().info("Mock Tagged Image publisher started")
        self.get_logger().info("Publishing at 1 Hz on: object_detection/tagged_image")

    def publish_tagged_image(self):
        bbox = _MOCK_BOUNDING_BOXES[self._cycle_index % len(_MOCK_BOUNDING_BOXES)]
        self._cycle_index += 1

        # Convert current hue to RGB and advance
        r_f, g_f, b_f = colorsys.hsv_to_rgb(self._hue, 1.0, 1.0)
        r, g, b = int(r_f * 255), int(g_f * 255), int(b_f * 255)
        self._hue = (self._hue + _HUE_STEP) % 1.0

        # Generate a small solid-colour image so the payload is non-trivial
        height, width = 240, 320
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :] = [r, g, b]

        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "object_detection"
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = width * 3
        img_msg.data = frame.tobytes()

        msg = TaggedImage()
        msg.image_data = img_msg
        msg.color_r = r
        msg.color_g = g
        msg.color_b = b
        msg.confidence_level = bbox["confidence"]
        msg.bounding_box = [
            Point32(x=float(x), y=float(y), z=0.0)
            for x, y in bbox["vertices"]
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
