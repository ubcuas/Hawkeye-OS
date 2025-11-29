#!/usr/bin/env python3
"""
Mock Object Detection - Continuous Feed

Reads images from test_images/ folder and publishes them continuously
to simulate a live object detection feed for WebRTC streaming.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from pathlib import Path


class MockObjectDetection(Node):
    def __init__(self):
        super().__init__('mock_object_detection')
        
        # Publisher for continuous video feed
        self.image_pub = self.create_publisher(Image, 'object_detection/image', 10)
        
        # Timer for publishing at 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)
        
        self.current_image_index = 0
        self.frame_count = 0
        
        # Load images from directory
        self.image_dir = Path('test_images')
        self.image_files = self.load_image_files()
        
        if not self.image_files:
            self.get_logger().warn(f'No images found in {self.image_dir}/')
            self.get_logger().warn('Please add .jpg, .png, or .jpeg files to test_images/')
            self.get_logger().warn('Creating test_images/ directory...')
            self.image_dir.mkdir(parents=True, exist_ok=True)
        else:
            self.get_logger().info(f'Loaded {len(self.image_files)} images from {self.image_dir}/')
        
        self.get_logger().info('Mock Object Detection started')
        self.get_logger().info('Publishing continuous feed at 30 FPS on: object_detection/image')

    def load_image_files(self):
        """Load all image files from test_images directory"""
        if not self.image_dir.exists():
            return []
        
        # Supported image formats
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        image_files = []
        
        for ext in extensions:
            image_files.extend(sorted(self.image_dir.glob(ext)))
            image_files.extend(sorted(self.image_dir.glob(ext.upper())))
        
        return image_files

    def publish_frame(self):
        """Load and publish an image with simulated detection overlay"""
        if not self.image_files:
            # No images available, publish black frame
            self.publish_black_frame()
            return
        
        # Get current image file (cycle through available images)
        image_file = self.image_files[self.current_image_index]
        self.current_image_index = (self.current_image_index + 1) % len(self.image_files)
        
        try:
            # Read image with OpenCV
            cv_image = cv2.imread(str(image_file))
            
            if cv_image is None:
                self.get_logger().error(f'Failed to load image: {image_file}')
                self.publish_black_frame()
                return
            
            # Convert BGR to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Add simulated object detection overlay
            cv_image = self.add_detection_overlay(cv_image)
            
            # Get image dimensions
            height, width, channels = cv_image.shape
            
            # Create ROS Image message
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'object_detection'
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = 'rgb8'
            img_msg.is_bigendian = False
            img_msg.step = width * channels
            img_msg.data = cv_image.tobytes()
            
            # Publish image
            self.image_pub.publish(img_msg)
            
            self.frame_count += 1
            
            # Log every 30 frames (once per second)
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published frame {self.frame_count}: {image_file.name} ({width}x{height})'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing image {image_file}: {e}')
            self.publish_black_frame()

    def add_detection_overlay(self, image):
        """Add simulated bounding box to image"""
        height, width = image.shape[:2]
        
        # Moving bounding box based on frame count
        box_x = 100 + (self.frame_count * 5) % (width - 200) if width > 200 else 50
        box_y = 100 + (self.frame_count * 3) % (height - 200) if height > 200 else 50
        box_w = min(150, width - box_x - 10)
        box_h = min(150, height - box_y - 10)
        
        # Draw red bounding box
        thickness = 3
        color = (255, 0, 0)  # Red in RGB
        
        # Draw rectangle
        image[box_y:box_y+thickness, box_x:box_x+box_w] = color  # Top
        image[box_y+box_h-thickness:box_y+box_h, box_x:box_x+box_w] = color  # Bottom
        image[box_y:box_y+box_h, box_x:box_x+thickness] = color  # Left
        image[box_y:box_y+box_h, box_x+box_w-thickness:box_x+box_w] = color  # Right
        
        return image

    def publish_black_frame(self):
        """Publish a black frame when no images available"""
        # Create black image
        black_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Create ROS Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'object_detection'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = 640 * 3
        img_msg.data = black_image.tobytes()
        
        # Publish
        self.image_pub.publish(img_msg)
        
        self.frame_count += 1
        
        # Warn user every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().warn(
                f'No images in test_images/ - publishing black frames'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MockObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()