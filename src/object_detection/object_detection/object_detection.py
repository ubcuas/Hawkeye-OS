import asyncio
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from hawkeye_msgs.msg import TaggedImage
from sensor_msgs.msg import CameraInfo
from object_detection import yolo_detection
import time
import numpy as np
import math
import cv2
from collections import deque
import pyrealsense2 as rs

import csv
import os

"""
Object Detection node

Subscribes to the TaggedImage topic published by image_processor.
Each message already contains synchronized color + depth data.
YOLO detection runs on the color frame and results are re-published
as an annotated TaggedImage.
"""

TAGGED_IMAGE_TOPIC = "/image_processor/tagged_image"
PROCESSED_TOPIC = "/object_detection/tagged_image"
PROCESS_EVERY_N = 5

CAMERA_INFO_TOPIC = "/camera/camera/aligned_depth_to_color/camera_info" 
ODOM_TOPIC = "/visual_slam/tracking/odometry"

COORD_LOGGING_PATH = "/ros2_ws/datalog/target_world_coords.csv"
# COORD_LOGGING_PATH = os.path.expanduser("~/Hawkeye-OS/datalog/target_world_coords.csv")

class ObjectDetection(Node):
    def __init__(self):

        super().__init__("object_detection")

        # Subscribe to the combined TaggedImage from image_processor
        self.tagged_image_sub = self.create_subscription(
            TaggedImage,
            TAGGED_IMAGE_TOPIC,
            self.tagged_image_callback,
            10,
        )

        # Publisher for the annotated TaggedImage
        self.process_pub = self.create_publisher(
            TaggedImage,
            PROCESSED_TOPIC,
            10,
        )
        
        # Subscribe to CameraInfo for 3D Projection
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10
        )
        
        # Subscribe to Odometry from Isaac ROS VSLAM
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )
        
        self.latest_odom_pose = None
        self.world_position_q = deque(maxlen=10) # Store recent WORLD 3D positions
        
        self.intrinsics = None # Will hold {fx, fy, cx, cy}
        self.position_q = deque(maxlen=10) # Store recent 3D positions

        self.get_logger().info('object_detection node started')
        self.get_logger().info(f'Subscribed to TaggedImage: {TAGGED_IMAGE_TOPIC}')
        self.get_logger().info(f'Publishing annotated TaggedImage to: {PROCESSED_TOPIC}')
        
        self.object_detector = yolo_detection.YoloDetector(self)
        self.frame_count = 0
        
        # record data
        # Setup CSV Logging
        self.csv_path = COORD_LOGGING_PATH
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'X', 'Y', 'Z'])
            
        self.get_logger().info(f'Logging 3D target coordinates to {self.csv_path}')
        
    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera matrix parameters into RealSense Intrinsics object."""
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]  # cx
            self.intrinsics.ppy = msg.k[5]  # cy
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            
            # aligned_depth_to_color is usually rectified (no distortion)
            self.intrinsics.model = rs.distortion.none 
            self.intrinsics.coeffs = [0, 0, 0, 0, 0] 
            
            self.get_logger().info(f"RealSense Intrinsics Loaded: {self.intrinsics.width}x{self.intrinsics.height} cx:{self.intrinsics.ppx} cy:{self.intrinsics.ppy}")
            
    def odom_callback(self, msg: Odometry):
        """Always keep the most recent VSLAM pose."""
        self.latest_odom_pose = msg.pose.pose

    @staticmethod
    def transform_to_world(point_optical, pose):
        """Converts local camera coordinates to world coordinates using Odometry pose."""
        
        # 1. Convert from Camera Optical Frame to Camera Link (Body) Frame
        # Optical: Z=forward, X=right, Y=down
        # Body:    X=forward, Y=left,  Z=up
        x_opt, y_opt, z_opt = point_optical
        point_link = np.array([z_opt, -x_opt, -y_opt])
        
        # 2. Odometry Translation
        tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
        
        # 3. Odometry Quaternion
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        
        # 4. Convert quaternion to Rotation matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)]
        ])
        
        # 5. Apply transformation: P_world = R * P_link + T
        point_world = R.dot(point_link) + np.array([tx, ty, tz])
        
        return point_world

    def tagged_image_callback(self, tagged_img_msg: TaggedImage):
        """Enqueue incoming TaggedImage for async YOLO processing."""
        # self.get_logger().info('RECV: Got TaggedImage')

        self.frame_count += 1
        if self.frame_count % PROCESS_EVERY_N != 0:
            return
        self.frame_count = 0 
        color_msg = tagged_img_msg.image_data
        depth_msg = tagged_img_msg.depth_data

        # self.get_logger().info('PROCESS: Got TaggedImage — running prediction')
        try:
            
            start_time = time.time()
            prediction_result = self.object_detector.predict(color_msg)
            if len(prediction_result) == 0:
                return
            
            # only take first prediction
            # for item in prediction_result[0]:
            # self.get_logger().info(prediction_result[0])
            bounding_boxes, conf, color_name = prediction_result[0] 

            # self.get_logger().info(f'PROCESS: PREDICTED IMAGES {color_name}')

            # Convert list of ((x1,y1),(x2,y2)) tuples → flat Point32 list.
            # Each detection contributes two points: top-left then bottom-right.
            box_points = []
            x1, y1, x2, y2 = bounding_boxes
            box_points.append(Point32(x=float(x1), y=float(y1), z=0.0))
            box_points.append(Point32(x=float(x2), y=float(y2), z=0.0))

            color_map = {
                "red":    (255,   0,   0),
                "yellow": (255, 255,   0),
                "green":  (  0, 255,   0),
                "blue":   (  0,   0, 255),
                "black":  (  0,   0,   0),
                "white":  (255, 255, 255),
            }
            color_rgb = color_map.get(color_name, (1, 2, 3))  # (1,2,3) = unknown sentinel

            # Adding Robust 3d coord calculation here
            if self.intrinsics is not None:
                # 1. Decode depth image safely via openCV IMREAD_UNCHANGED (keeps 16-bit format)
                np_arr = np.frombuffer(depth_msg.data, np.uint8)
                depth_img_raw = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                
                # Verify successful decode
                if depth_img_raw is not None:
                    h, w = depth_img_raw.shape[:2]
                    
                    # 2. Extract bounding box and enforce bounds
                    x1, y1, x2, y2 = [int(v) for v in bounding_boxes]
                    roi_x1, roi_x2 = max(0, x1), min(w, x2)
                    roi_y1, roi_y2 = max(0, y1), min(h, y2)
                    
                    depth_roi = depth_img_raw[roi_y1:roi_y2, roi_x1:roi_x2]
                    
                    # Mask out invalid pixels (usually 0 in RealSense)
                    valid_depths = depth_roi[depth_roi > 0]
                    
                    if len(valid_depths) > 0:
                        self.get_logger().info(f"Depth ROI has {len(valid_depths)} valid pixels. Calculating 3D position...")
                        # Median spatial filtering
                        z_mm = np.median(valid_depths)
                        z_m = z_mm / 1000.0
                        
                        # Calculate center 'uv'
                        u = float((roi_x1 + roi_x2) / 2.0)
                        v = float((roi_y1 + roi_y2) / 2.0)
                        
                        # 3. Apply RealSense Library function
                        # Output is [x, y, z] in camera coordinates
                        point_3d_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_m)
                        
                        # 4. Transform to World Frame and filter temporally
                        if self.latest_odom_pose is not None:
                            world_pt = self.transform_to_world(point_3d_cam, self.latest_odom_pose)
                            self.world_position_q.append(world_pt)
                            
                            recent_world_positions = np.array(self.world_position_q)
                            smooth_x = np.median(recent_world_positions[:, 0])
                            smooth_y = np.median(recent_world_positions[:, 1])
                            smooth_z = np.median(recent_world_positions[:, 2])
                            
                            # 5. Output stabilized WORLD point
                            self.get_logger().info(
                                f"TARGET WORLD POS | X: {smooth_x:+.3f}m, Y: {smooth_y:+.3f}m, Z: {smooth_z:+.3f}m"
                            )
                            
                            # Log to CSV
                            with open(self.csv_path, mode='a', newline='') as file:
                                writer = csv.writer(file)
                                # time.time() provides standard epoch time
                                writer.writerow([time.time(), smooth_x, smooth_y, smooth_z])
                        else:
                            self.get_logger().warn("Waiting for VSLAM Odometry to compute world coordinates...")
                            
                            
            # Re-publish as annotated TaggedImage, preserving depth + metadata
            self.process_pub.publish(
                TaggedImage(
                    image_data=color_msg,
                    depth_data=depth_msg,
                    imu_orientation=tagged_img_msg.imu_orientation,
                    yaw_deg=tagged_img_msg.yaw_deg,
                    color_r=color_rgb[0],
                    color_g=color_rgb[1],
                    color_b=color_rgb[2],
                    bounding_box=box_points,
                    confidence_level=conf,
                )
            )
            
            end_time = time.time()
            processing_time = end_time - start_time
            self.get_logger().info(f'PROCESS: Image processing took {processing_time:.4f} seconds')

        except Exception as e:
            self.get_logger().error(f'PROCESS: Prediction error: {e}')


def main(args=None):
    """Entry point wrapper"""
    rclpy.init(args=args)
    node = ObjectDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()