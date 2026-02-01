#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage, Imu, Image
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # Subscribe to RealSense raw image with SensorData QoS
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        self.image_publisher = self.create_publisher(
            Image,
            'object_detection/image',
            10
        )

        self.detection_publisher = self.create_publisher(
            String,
            '/detections',
            10
        )
    
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"Received image of size: {cv_image.shape}")
            cv2.imwrite('debug_photo.jpg', cv_image)

            output_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            output_msg.header = msg.header
            self.image_publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'Conversion failed: {e}')

    # ...existing imu_callback, gps_callback, main...
    
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