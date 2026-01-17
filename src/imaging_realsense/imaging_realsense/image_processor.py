#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__('image_processor')
        
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.telemetry_subscription = self.create_subscription(
            String,
            '/camera/telemetry',
            self.telemetry_callback,
            10
        )
        
        self.detection_publisher = self.create_publisher(
            String,
            '/detections',
            10
        )
    
    def image_callback(self, msg):
        pass
    
    def telemetry_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
