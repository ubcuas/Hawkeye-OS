#!/usr/bin/env python3
"""
Advanced image processor with timestamp-based GPS/image synchronization.
Uses message_filters for exact time matching.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import cv2


class ImageProcessorSynced(Node):
    
    def __init__(self):
        super().__init__('image_processor_synced')
        
        # Declare parameters for synchronization
        self.declare_parameter('sync_slop', 0.1)  # 100ms tolerance
        self.declare_parameter('queue_size', 10)
        
        slop = self.get_parameter('sync_slop').value
        queue_size = self.get_parameter('queue_size').value
        
        # Create message filter subscribers
        self.image_sub = Subscriber(
            self,
            CompressedImage,
            '/camera/camera/color/image_raw/compressed'
        )
        
        self.gps_sub = Subscriber(
            self,
            NavSatFix,
            '/gps/fix'
        )
        
        self.imu_sub = Subscriber(
            self,
            Imu,
            '/camera/camera/imu'
        )
        
        # Synchronize image + GPS (approximate time sync)
        self.sync_image_gps = ApproximateTimeSynchronizer(
            [self.image_sub, self.gps_sub],
            queue_size=queue_size,
            slop=slop
        )
        self.sync_image_gps.registerCallback(self.synced_callback)
        
        # Optional: Synchronize all three (image + GPS + IMU)
        # self.sync_all = ApproximateTimeSynchronizer(
        #     [self.image_sub, self.gps_sub, self.imu_sub],
        #     queue_size=queue_size,
        #     slop=slop
        # )
        # self.sync_all.registerCallback(self.synced_callback_all)
        
        # Publisher for detections
        self.detection_publisher = self.create_publisher(
            String,
            '/detections',
            10
        )
        
        self.get_logger().info(
            f'ImageProcessorSynced initialized with {slop*1000}ms sync tolerance'
        )
        self.image_count = 0
    
    def synced_callback(self, image_msg, gps_msg):
        """
        Callback triggered when image and GPS are synchronized.
        
        This is called ONLY when both messages have similar timestamps.
        """
        self.image_count += 1
        
        # Calculate actual time difference
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        gps_time = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9
        time_diff = abs(image_time - gps_time)
        
        self.get_logger().info(
            f'[{self.image_count}] Synced image captured at '
            f'({gps_msg.latitude:.6f}, {gps_msg.longitude:.6f}, {gps_msg.altitude:.2f}m) '
            f'time_diff={time_diff*1000:.1f}ms'
        )
        
        # Decode compressed image
        try:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error('Failed to decode image')
                return
            
            self.get_logger().info(f'Image shape: {cv_image.shape}')
            
            # TODO: Run object detection on cv_image
            # detections = self.run_detection(cv_image)
            
            # TODO: Publish detection results with GPS coordinates
            # self.publish_detection(detections, gps_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')
    
    def synced_callback_all(self, image_msg, gps_msg, imu_msg):
        """
        Optional: Callback for synchronizing all three streams.
        Use this if you need IMU data at the exact moment of image capture.
        """
        self.get_logger().info(
            f'Synced all: Image + GPS + IMU at '
            f'({gps_msg.latitude:.6f}, {gps_msg.longitude:.6f})'
        )
        # Process all three synchronized messages
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorSynced()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
