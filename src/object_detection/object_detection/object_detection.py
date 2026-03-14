import asyncio
import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Point32, PoseStamped
from sensor_msgs.msg import CameraInfo
from hawkeye_msgs.msg import TaggedImage
from object_detection.yolo_detection import predict_images

"""
Object Detection node

Subscribes to the TaggedImage topic published by image_processor.
Each message already contains synchronized color + depth data.
YOLO detection runs on the color frame and results are re-published
as an annotated TaggedImage.

When `enable_navigation` is True, bounding-box centres are deprojected
to 3D camera-frame coordinates using the depth map and camera intrinsics,
then published as PoseStamped to the navigation node.
"""

TAGGED_IMAGE_TOPIC = "/image_processor/tagged_image"
PROCESSED_TOPIC = "/object_detection/tagged_image"
NAV_CMD_TOPIC = "/drone/cmd_pose"
DEPTH_CAMERA_INFO_TOPIC = "/camera/camera/depth/camera_info"


class ObjectDetection(Node):
    def __init__(self):

        super().__init__("object_detection")

        self.declare_parameter('enable_navigation', False)
        self.enable_navigation = self.get_parameter('enable_navigation').value

        # Async queue — incoming TaggedImage messages are enqueued here
        self.image_queue = asyncio.Queue()

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

        # Navigation support: 3D deprojection → publish target pose
        self.depth_intrin = None
        self.nav_pub = None
        if self.enable_navigation:
            self.nav_pub = self.create_publisher(PoseStamped, NAV_CMD_TOPIC, 10)
            self.create_subscription(
                CameraInfo, DEPTH_CAMERA_INFO_TOPIC,
                self._on_camera_info, QoSPresetProfiles.SENSOR_DATA.value,
            )
            self.get_logger().info(f'Navigation ENABLED -> {NAV_CMD_TOPIC}')

        self.get_logger().info('object_detection node started')

    def _on_camera_info(self, msg: CameraInfo):
        """Store depth intrinsics as an rs.intrinsics object (first message only)."""
        if self.depth_intrin is not None:
            return
        self.depth_intrin = rs.intrinsics()
        self.depth_intrin.width = msg.width
        self.depth_intrin.height = msg.height
        self.depth_intrin.fx = msg.k[0]
        self.depth_intrin.fy = msg.k[4]
        self.depth_intrin.ppx = msg.k[2]
        self.depth_intrin.ppy = msg.k[5]
        self.depth_intrin.model = rs.distortion.none
        self.depth_intrin.coeffs = list(msg.d)[:5]
        self.get_logger().info(f'Got depth intrinsics: {msg.width}x{msg.height}')

    def tagged_image_callback(self, msg: TaggedImage):
        """Enqueue incoming TaggedImage for async YOLO processing."""
        asyncio.run_coroutine_threadsafe(
            self.image_queue.put(msg),
            self.loop,
        )

    async def process_images(self):
        """Dequeue TaggedImage messages and run YOLO detection on the color frame."""
        while rclpy.ok():
            tagged_msg: TaggedImage = await self.image_queue.get()

            color_msg = tagged_msg.image_data
            depth_msg = tagged_msg.depth_data

            self.get_logger().info('PROCESS: Got TaggedImage — running prediction')
            try:
                bounding_boxes = predict_images(self, color_msg)
                self.get_logger().info(f'PREDICTED {len(bounding_boxes)} boxes')

                if self.enable_navigation and bounding_boxes and self.depth_intrin is not None:
                    depth_arr = np.frombuffer(bytes(depth_msg.data), np.uint8)
                    depth_frame = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)

                    for (x1, y1), (x2, y2) in bounding_boxes:
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                        if not (0 <= cy < depth_frame.shape[0] and 0 <= cx < depth_frame.shape[1]):
                            continue
                        depth_mm = depth_frame[cy, cx]
                        if depth_mm == 0:
                            continue

                        point_3d = rs.rs2_deproject_pixel_to_point(
                            self.depth_intrin, [cx, cy], depth_mm / 1000.0
                        )

                        pose = PoseStamped()
                        pose.header.stamp = color_msg.header.stamp
                        pose.header.frame_id = 'camera_depth_optical_frame'
                        pose.pose.position.x = point_3d[0]
                        pose.pose.position.y = point_3d[1]
                        pose.pose.position.z = point_3d[2]
                        pose.pose.orientation.w = 1.0
                        self.nav_pub.publish(pose)

                        self.get_logger().info(
                            f'NAV: ({cx},{cy}) -> 3D ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})'
                        )

                # Convert list of ((x1,y1),(x2,y2)) tuples → flat Point32 list.
                # Each detection contributes two points: top-left then bottom-right.
                box_points = []
                for (x1, y1), (x2, y2) in bounding_boxes:
                    box_points.append(Point32(x=float(x1), y=float(y1), z=0.0))
                    box_points.append(Point32(x=float(x2), y=float(y2), z=0.0))

                # Re-publish as annotated TaggedImage, preserving depth + metadata
                self.process_pub.publish(
                    TaggedImage(
                        image_data=color_msg,
                        depth_data=depth_msg,
                        imu_data=tagged_msg.imu_data,
                        gps_data=tagged_msg.gps_data,
                        color_r=0,
                        color_g=0,
                        color_b=0,
                        bounding_box=box_points,
                        confidence_level=0,
                    )
                )

            except Exception as e:
                self.get_logger().error(f'PROCESS: Prediction error: {e}')


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    object_detection = ObjectDetection()
    object_detection.loop = asyncio.get_running_loop()

    executor = SingleThreadedExecutor()
    executor.add_node(object_detection)

    async def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            object_detection.process_images(),
        )
    except KeyboardInterrupt:
        pass
    finally:
        object_detection.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == '__main__':
    main()