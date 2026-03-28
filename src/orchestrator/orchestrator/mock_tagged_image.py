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
import cv2
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from hawkeye_msgs.msg import TaggedImage


# Bounding box shapes to cycle through so the data is visually distinct
_MOCK_BOUNDING_BOXES = [
    {"vertices": [(100, 80), (300, 80), (300, 200), (100, 200)], "confidence": 0.92},
    {"vertices": [(50, 50), (150, 200)], "confidence": 0.78},
    {"vertices": [(200, 100), (280, 100), (280, 180), (200, 180), (240, 140)], "confidence": 0.85},
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
        now = self.get_clock().now().to_msg()

        # Color image (JPEG-compressed)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :] = [r, g, b]

        img_msg = CompressedImage()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "mock_camera"
        img_msg.format = "jpeg"
        _, jpeg_data = cv2.imencode(".jpg", frame)
        img_msg.data = jpeg_data.tobytes()

        # Depth image (16UC1 PNG-compressed, values in millimeters)
        # Gradient from 1m (1000mm) at top to 5m (5000mm) at bottom
        depth_frame = np.linspace(1000, 5000, height, dtype=np.uint16)
        depth_frame = np.tile(depth_frame.reshape(-1, 1), (1, width))

        depth_msg = CompressedImage()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "mock_camera"
        depth_msg.format = "16UC1; png"
        _, depth_png = cv2.imencode(".png", depth_frame)
        depth_msg.data = depth_png.tobytes()

        # IMU data (simulated level orientation with minor angular velocity)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "mock_imu"
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.z = 0.01
        imu_msg.linear_acceleration.z = 9.81

        # GPS data (UBC campus)
        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = "mock_gps"
        gps_msg.latitude = 49.2606
        gps_msg.longitude = -123.2460
        gps_msg.altitude = 76.0

        msg = TaggedImage()
        msg.image_data = img_msg
        msg.depth_data = depth_msg
        msg.imu_data = imu_msg
        msg.gps_data = gps_msg
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
