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
        
        # Cache for latest GPS and IMU data
        self.latest_gps = None
        self.latest_imu = None
        self.gps_lock = None  # Will store threading.Lock() if needed
        
        # Subscribe to official RealSense compressed image topic
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
        
        self.get_logger().info('ImageProcessor initialized - waiting for GPS and camera data...')
    

    def image_callback(self, msg):
        """Process image and attach latest GPS/IMU data."""
        # Check if we have GPS data yet
        if self.latest_gps is None:
            self.get_logger().warn('No GPS data available yet, skipping image', throttle_duration_sec=5.0)
            return
        
        # Get timestamps for synchronization check
        image_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gps_time = self.latest_gps.header.stamp.sec + self.latest_gps.header.stamp.nanosec * 1e-9
        time_diff = abs(image_time - gps_time)
        
        # Warn if GPS data is too old (more than 1 second)
        if time_diff > 1.0:
            self.get_logger().warn(f'GPS data is {time_diff:.2f}s old - may be stale')
        
        # Log synchronized data
        self.get_logger().info(
            f'Image captured at ({self.latest_gps.latitude:.6f}, {self.latest_gps.longitude:.6f}, {self.latest_gps.altitude:.2f}m) '
            f'time_diff={time_diff*1000:.1f}ms'
        )

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"Received image of size: {cv_image.shape}")
            cv2.imwrite('debug_photo.jpg', cv_image)

            output_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            output_msg.header = msg.header
            self.image_publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'Conversion failed: {e}')
        


    
    def imu_callback(self, msg):
        """Cache latest IMU data."""
        self.latest_imu = msg
    
    def gps_callback(self, msg):
        """Cache latest GPS data."""
        self.latest_gps = msg
        self.get_logger().debug(
            f'GPS updated: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}m)',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()