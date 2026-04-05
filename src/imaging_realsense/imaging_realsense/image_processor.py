#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import CompressedImage, Image, Imu
from sensor_msgs.msg import NavSatFix
from hawkeye_msgs.msg import TaggedImage

"""
ImageProcessor node

Subscribes to compressed color and depth image topics published by the
RealSense camera, synchronizes them by timestamp using
ApproximateTimeSynchronizer, and re-publishes them together as a single
TaggedImage message for downstream consumers (e.g. object_detection).
"""
# topic for connected camera
COLOR_TOPIC = "/camera/camera/color/image_raw/compressed"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"

# topics for ros2 bag
# COLOR_TOPIC = "color/image_raw/compressed"
# DEPTH_TOPIC = "/depth/image_rect_raw/compressed"

IMU_TOPIC   = "/camera/camera/imu"
GPS_TOPIC   = "/gps/fix"

OUTPUT_TOPIC = "/image_processor/tagged_image"

# ApproximateTimeSynchronizer settings
SYNC_QUEUE_SIZE = 10
SYNC_SLOP = 1000  # Max time difference (seconds) between matched frames

# Publish every Nth synchronized frame (1 = publish all)
PUBLISH_EVERY_N = 1

MAVROS_IMU_TOPIC = "/mavros/imu/data"


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # Cache for optional metadata
        self.latest_gps = None
        self.latest_imu = None
        self.latest_yaw_deg = None  # None → publish yaw_deg as NaN on TaggedImage
        self.latest_yaw_stamp_sec = None  # from Imu; for future synchronization
        self.latest_imu_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Frame counter for throttling
        self._frame_count = 0

        # message_filters subscribers for camera streams
        self.color_sub = message_filters.Subscriber(
            self, CompressedImage, COLOR_TOPIC, qos_profile=sensor_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, DEPTH_TOPIC, qos_profile=sensor_qos
        )

        # Optional metadata subscriptions (regular, non-synchronized)

        self.mavros_imu_subscription = self.create_subscription(
            Imu,
            MAVROS_IMU_TOPIC,
            self.mavros_imu_callback,
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
        self.get_logger().info(f'Subscribed to IMU: {IMU_TOPIC}')
        self.get_logger().info(f'Subscribed to MAVROS IMU: {MAVROS_IMU_TOPIC}')

    def synchronized_callback(self, color_msg: CompressedImage, depth_msg: Image):
        """Called when a matching color+depth pair arrives within the slop window."""
        self._frame_count += 1

        if self._frame_count % PUBLISH_EVERY_N != 0:
            return

        # self.get_logger().info(
        #     f'SYNC: Matched pair — color: {color_msg.header.stamp}, '
        #     f'depth: {depth_msg.header.stamp}'
        # )

        # Convert raw 16UC1 depth Image → PNG-encoded CompressedImage
        # COMMENT OUT FOR ROSBAG
        # (JPEG can't encode 16-bit; PNG supports it natively and losslessly)
        depth_compressed = CompressedImage()
        depth_compressed.header = depth_msg.header
        depth_compressed.format = 'png'
        depth_frame = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            (depth_msg.height, depth_msg.width)
        )
        _, png_bytes = cv2.imencode('.png', depth_frame)
        depth_compressed.data = png_bytes.tobytes()

        tagged = TaggedImage()
        tagged.image_data = color_msg
        tagged.depth_data = depth_compressed

        if self.latest_imu is not None:
            tagged.imu_data = self.latest_imu

        if self.latest_gps is not None:
            tagged.gps_data = self.latest_gps

        tagged.imu_orientation = self.latest_imu_orientation
        if self.latest_yaw_deg is not None:
            tagged.yaw_deg = float(self.latest_yaw_deg)
        else:
            tagged.yaw_deg = float("nan")

        self.tagged_image_pub.publish(tagged)
        self.get_logger().debug('Published TaggedImage')

    def gps_callback(self, msg: NavSatFix):
        """Cache latest GPS data."""
        self.latest_gps = msg
        self.get_logger().debug(
            f'GPS updated: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}m)',
            throttle_duration_sec=1.0,
        )

    @staticmethod
    def imu_calculation(msg: Imu):
        """Yaw from Imu.orientation quaternion to degrees [0, 360).

        Returns:
            (yaw_deg, stamp_sec, orientation)
        """
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        if msg.orientation_covariance[0] < 0.0:
            return None, stamp_sec, identity

        q = msg.orientation  # geometry_msgs/Quaternion
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-8:
            return None, stamp_sec, identity
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        q_out = Quaternion(x=x, y=y, z=z, w=w)

        # Yaw about vertical axis from Hamilton quaternion (x, y, z, w) — radians
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        yaw_deg = math.degrees(yaw_rad) % 360.0
        return yaw_deg, stamp_sec, q_out


    def mavros_imu_callback(self, msg: Imu):
        """Cache latest MAVROS IMU and derive yaw + quaternion."""
        yaw_deg, stamp_sec, orientation = self.imu_calculation(msg)
        self.latest_yaw_stamp_sec = stamp_sec
        self.latest_yaw_deg = yaw_deg
        self.latest_imu_orientation = orientation
        self.latest_imu = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()