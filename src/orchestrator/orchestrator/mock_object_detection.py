#!/usr/bin/env python3
"""
Mock Object Detection - Video Stream Feed

Reads video from test_images/ folder and publishes frames continuously
to simulate a live camera feed for WebRTC streaming.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class MockObjectDetection(Node):
    def __init__(self):
        super().__init__('mock_object_detection')   
        
        # Publisher for continuous video feed
        self.image_pub = self.create_publisher(CompressedImage, 'color/image_raw/compressed', 10)
        
        # Timer for publishing at 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)
        
        self.frame_count = 0
        
        # Video capture
        self.video_path = 'test_images/test_video.mp4' # VIDEO PATH
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().warn(f'Could not open video: {self.video_path}')
            self.get_logger().warn('Falling back to black frames')
            self.cap = None
        else:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.get_logger().info(f'Loaded video: {self.video_path}')
            self.get_logger().info(f'Video FPS: {fps}, Total frames: {frame_count}')
        
        self.get_logger().info('Mock Object Detection started')
        self.get_logger().info('Publishing continuous feed at 30 FPS on: color/image_raw/compressed')

    def publish_frame(self):
        """Read and publish raw video frame"""
        if self.cap is None or not self.cap.isOpened():
            self.publish_black_frame()
            return
        
        try:
            # Read frame from video
            ret, cv_image = self.cap.read()
            
            # Loop video when it ends
            if not ret:
                self.get_logger().info('Video ended, restarting...')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, cv_image = self.cap.read()
                
            if not ret:
                self.get_logger().error('Failed to read video frame')
                self.publish_black_frame()
                return
            
            # Encode frame as JPEG (CompressedImage carries JPEG/PNG bytes)
            _, jpeg_data = cv2.imencode('.jpg', cv_image)

            # Create ROS CompressedImage message
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'object_detection'
            img_msg.format = 'jpeg'
            img_msg.data = jpeg_data.tobytes()

            # Publish image
            self.image_pub.publish(img_msg)
            
            self.frame_count += 1
            
            # Log every 30 frames (once per second)
            # if self.frame_count % 30 == 0:
            #     self.get_logger().info(
            #         f'Published frame {self.frame_count} ({width}x{height})'
            #     )
            
        except Exception as e:
            self.get_logger().error(f'Error processing video frame: {e}')
            self.publish_black_frame()

    def publish_black_frame(self):
        """Publish a black frame when no video available"""
        black_image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, jpeg_data = cv2.imencode('.jpg', black_image)

        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'object_detection'
        img_msg.format = 'jpeg'
        img_msg.data = jpeg_data.tobytes()

        self.image_pub.publish(img_msg)
        
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()