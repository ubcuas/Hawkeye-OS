#!/usr/bin/env python3

import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage, Imu
from sensor_msgs.msg import NavSatFix
from hawkeye_msgs.msg import TaggedImage

"""
ImageProcessor node

Subscribes to compressed color and depth image topics published by the
RealSense camera, synchronizes them by timestamp using
ApproximateTimeSynchronizer, and re-publishes them together as a single
TaggedImage message for downstream consumers (e.g. object_detection).
"""

COLOR_TOPIC = "/camera/camera/color/image_raw/compressed"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw/compressed"
IMU_TOPIC   = "/camera/camera/imu"
GPS_TOPIC   = "/gps/fix"
OUTPUT_TOPIC = "/image_processor/tagged_image"

# ApproximateTimeSynchronizer settings
SYNC_QUEUE_SIZE = 10
SYNC_SLOP = 1000  # Max time difference (seconds) between matched frames

# Publish every Nth synchronized frame (1 = publish all)
PUBLISH_EVERY_N = 5


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # Cache for optional metadata
        self.latest_gps = None
        self.latest_imu = None

        # Frame counter for throttling
        self._frame_count = 0

        # message_filters subscribers for camera streams
        self.color_sub = message_filters.Subscriber(
            self, CompressedImage, COLOR_TOPIC, qos_profile=sensor_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, CompressedImage, DEPTH_TOPIC, qos_profile=sensor_qos
        )

        # Optional metadata subscriptions (regular, non-synchronized)
        self.imu_subscription = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            qos_profile=sensor_qos,
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            GPS_TOPIC,
            self.gps_callback,
            10,
        )

        # Synchronize color + depth frames by approximate timestamp
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=SYNC_QUEUE_SIZE,
            slop=SYNC_SLOP,
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for the combined TaggedImage
        self.tagged_image_pub = self.create_publisher(
            TaggedImage,
            OUTPUT_TOPIC,
            10,
        )

        self.get_logger().info('ImageProcessor node started')
        self.get_logger().info(f'Subscribed to color: {COLOR_TOPIC}')
        self.get_logger().info(f'Subscribed to depth: {DEPTH_TOPIC}')
        self.get_logger().info(f'Publishing TaggedImage to: {OUTPUT_TOPIC}')
        self.get_logger().info(f'Sync slop: {SYNC_SLOP}s')

    def synchronized_callback(self, color_msg: CompressedImage, depth_msg: CompressedImage):
        """Called when a matching color+depth pair arrives within the slop window."""
        self.get_logger().info("Got image")
        self._frame_count += 1

        if self._frame_count % PUBLISH_EVERY_N != 0:
            return

        self.get_logger().info(
            f'SYNC: Matched pair — color: {color_msg.header.stamp}, '
            f'depth: {depth_msg.header.stamp}'
        )

        tagged = TaggedImage()
        tagged.image_data = color_msg
        tagged.depth_data = depth_msg

        if self.latest_imu is not None:
            tagged.imu_data = self.latest_imu

        if self.latest_gps is not None:
            tagged.gps_data = self.latest_gps

        self.tagged_image_pub.publish(tagged)
        self.get_logger().debug('Published TaggedImage')

    def imu_callback(self, msg: Imu):
        """Cache latest IMU data."""
        self.latest_imu = msg

    def gps_callback(self, msg: NavSatFix):
        """Cache latest GPS data."""
        self.latest_gps = msg
        self.get_logger().debug(
            f'GPS updated: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}m)',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()