#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class RealSenseCameraPublisher(Node):
    
    def __init__(self):
        super().__init__('realsense_camera_publisher')
        
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            10
        )
        
        self.telemetry_publisher = self.create_publisher(
            String,
            '/camera/telemetry',
            10
        )


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
