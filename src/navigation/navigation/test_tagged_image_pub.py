import rclpy
from rclpy.node import Node
from hawkeye_msgs.msg import TaggedImage
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from geometry_msgs.msg import Point32
import numpy as np
import cv2

class TestTaggedImagePublisher(Node):
    def __init__(self):
        super().__init__('test_tagged_image')
        self.pub = self.create_publisher(TaggedImage, '/object_detection/tagged_image', 1)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return

        msg = TaggedImage()

        # --- Dummy color image ---
        color_img = np.zeros((480, 640, 3), dtype=np.uint8)
        color_img[:] = (0, 255, 0)  # green background
        _, encoded_color = cv2.imencode('.png', color_img)
        msg.image_data = CompressedImage()
        msg.image_data.format = 'png'
        msg.image_data.data = encoded_color.tobytes()

        # --- Dummy depth image ---
        depth_m = 11.5  # forward distance
        depth_img = np.ones((480, 640), dtype=np.uint16) * int(depth_m * 1000)  # mm
        _, encoded_depth = cv2.imencode('.png', depth_img)
        msg.depth_data = CompressedImage()
        msg.depth_data.format = 'png'
        msg.depth_data.data = encoded_depth.tobytes()

        # --- Minimal IMU/GPS ---
        msg.imu_data = Imu()
        msg.gps_data = NavSatFix()

        # --- Bounding box producing 11.5m forward, 10m up ---
        img_cx, img_cy = 320, 240
        fx, fy = 600.0, 600.0  # camera intrinsics

        # Compute pixel offset for the depth image to get 10 m up
        v_offset = img_cy - int(10.0 * fy / depth_m)  # up is negative in image
        u_offset = img_cx  # straight ahead, centered horizontally

        msg.bounding_box = [
            Point32(x=float(u_offset - 5), y=float(v_offset - 5), z=0.0),
            Point32(x=float(u_offset + 5), y=float(v_offset + 5), z=0.0)
        ]

        msg.confidence_level = 0.9

        self.pub.publish(msg)
        self.get_logger().info(f"Published one TaggedImage -> target: forward {depth_m} m, up 10 m")
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = TestTaggedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()