#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__('image_processor')
        
        # Subscribe to official RealSense compressed image topic
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Subscribe to official RealSense IMU topic
        self.imu_subscription = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            10
        )
        
        # Subscribe to external GPS topic (if available)
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Publish detections
        self.detection_publisher = self.create_publisher(
            String,
            '/detections',
            10
        )
    
    def image_callback(self, msg):
        pass
    
    def imu_callback(self, msg):
        pass
    
    def gps_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
