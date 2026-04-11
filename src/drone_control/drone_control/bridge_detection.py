import math
from collections import deque

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from hawkeye_msgs.msg import TaggedImage


# Fixed camera intrinsics (Brown-Conrady / plumb_bob model).
# Values from CameraInfo K/d for camera_color_optical_frame (1280x720).
# These may be subject to change (recalibration, resolution, or camera hardware).
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_INTRINSICS = {
    "fx": 644.648681640625,
    "fy": 643.7971801757812,
    "ppx": 638.689453125,
    "ppy": 383.8777160644531,
}
# Brown-Conrady distortion coefficients [k1, k2, p1, p2, k3]
DISTORTION_COEFFS = [
    -0.05599868297576904,
    0.06472615152597427,
    -8.93151736818254e-05,
    -7.332695531658828e-05,
    -0.020574895665049553,
]

# WRONG BUT KEPT FOR POSTERITY

# CAMERA_INTRINSICS = {
#     "fx": 643.2360229492188,
#     "fy": 642.1893920898438,
#     "ppx": 661.8545532226562,
#     "ppy": 365.9696044921875,
# }
# # Brown-Conrady distortion coefficients [k1, k2, p1, p2, k3]
# DISTORTION_COEFFS = [
#     -0.05651168152689934,
#     0.06660270690917969,
#     -0.00015544862253591418,
#     0.0008432056056335568,
#     -0.02149238809943199,
# ]
# ==========================
# Tunable parameters
# ==========================
MIN_DETECTION_CONF = 0.1
DEPTH_PATCH_RADIUS = 3
DEBUG_FLAG = True
CAMERA_X_OFFSET_M = 0.0
CAMERA_Y_OFFSET_M = 0.0
CAMERA_Z_OFFSET_M = 0.0
CONFIDENCE_QUEUE_SIZE = 5


class BridgeDetection(Node):
    def __init__(self):
        super().__init__("bridge_detection")

        self.mission_active = False
        self.depth_intrinsics = None
        self.have_depth_intrinsics = False

        # (np.ndarray shape (3,), weight) in platform_odom header frame
        self._target_world_queue: deque = deque(maxlen=CONFIDENCE_QUEUE_SIZE)
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        mission_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.det_sub = self.create_subscription(
            TaggedImage,
            "/object_detection/tagged_image",
            self.detection_cb,
            qos_reliable,
        )

        self.mission_state_sub = self.create_subscription(
            Bool,
            "/drone/mission_active",
            self.mission_state_cb,
            mission_state_qos,
        )

        self.cmd_pub = self.create_publisher(
            PoseStamped,
            "/drone/cmd_pose",
            qos_reliable,
        )

        self._init_depth_intrinsics_from_constants()

        self.get_logger().info("Detection bridge started")
        self.get_logger().info("Listening on /object_detection/tagged_image")
        self.get_logger().info("Publishing commands to /drone/cmd_pose")

    def mission_state_cb(self, msg: Bool):
        if msg.data and not self.mission_active:
            self._target_world_queue.clear()
        self.mission_active = msg.data

    @staticmethod
    def _quaternion_to_R_body_to_world(qx: float, qy: float, qz: float, qw: float):
        """Rotation matrix R: p_world = R @ p_body (Hamilton quaternion x,y,z,w)."""
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-10:
            return None
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )
        return R

    @staticmethod
    def _odom_valid(odom) -> bool:
        q = odom.pose.pose.orientation
        return (
            BridgeDetection._quaternion_to_R_body_to_world(q.x, q.y, q.z, q.w)
            is not None
        )

    @staticmethod
    def _weighted_world_mean(entries):
        """entries: iterable of (p_world (3,), weight float)."""
        w_sum = 0.0
        acc = np.zeros(3, dtype=np.float64)
        for p, w in entries:
            if w <= 0.0:
                continue
            acc += w * p
            w_sum += w
        if w_sum <= 0.0:
            return None
        return acc / w_sum

    def _init_depth_intrinsics_from_constants(self):
        intr = rs.intrinsics()
        intr.width = CAMERA_WIDTH
        intr.height = CAMERA_HEIGHT
        intr.ppx = float(CAMERA_INTRINSICS["ppx"])
        intr.ppy = float(CAMERA_INTRINSICS["ppy"])
        intr.fx = float(CAMERA_INTRINSICS["fx"])
        intr.fy = float(CAMERA_INTRINSICS["fy"])
        intr.model = rs.distortion.brown_conrady
        intr.coeffs = list(DISTORTION_COEFFS)[:5]

        self.depth_intrinsics = intr
        self.have_depth_intrinsics = True

    def detection_cb(self, msg: TaggedImage):
        self.get_logger().info("Received TaggedImage")

        if self.mission_active:
            self.get_logger().info("Mission active, ignoring detection")
            return
        
        if not self.have_depth_intrinsics or self.depth_intrinsics is None:
            self.get_logger().warn("No depth camera intrinsics received yet. Ignoring detection.")
            return

        conf = float(msg.confidence_level)
        if conf < MIN_DETECTION_CONF:
            if DEBUG_FLAG:
                self.get_logger().info(
                    f"Ignoring detection: conf={conf:.3f} < {MIN_DETECTION_CONF:.3f}"
                )
            return

        if len(msg.bounding_box) < 2:
            self.get_logger().warn(
                "Ignoring detection: bounding_box has fewer than 2 points"
            )
            return

        try:
            # Normalize bbox coordinates
            x1 = int(min(msg.bounding_box[0].x, msg.bounding_box[1].x))
            y1 = int(min(msg.bounding_box[0].y, msg.bounding_box[1].y))
            x2 = int(max(msg.bounding_box[0].x, msg.bounding_box[1].x))
            y2 = int(max(msg.bounding_box[0].y, msg.bounding_box[1].y))

            # Center pixel of detection
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            depth_img_m = self.decode_depth_image_to_meters(msg.depth_data)
            depth_m = self.sample_depth_median(depth_img_m, u, v, DEPTH_PATCH_RADIUS)

            if depth_m is None:
                self.get_logger().warn(
                    "Ignoring detection: no valid depth near bbox center"
                )
                return

            # RealSense deprojection:
            # returns camera-frame coordinates [x, y, z]
            # camera optical frame: x right, y down, z forward
            x_cam, y_cam, z_cam = rs.rs2_deproject_pixel_to_point(
                self.depth_intrinsics,
                [float(u), float(v)],
                float(depth_m),
            )

            # Convert optical frame -> body frame expected by drone_control.py
            # body x forward, y left, z up
            x_body = z_cam + CAMERA_X_OFFSET_M
            y_body = -x_cam + CAMERA_Y_OFFSET_M
            z_body = -y_cam + CAMERA_Z_OFFSET_M

            p_body = np.array([x_body, y_body, z_body], dtype=np.float64)

            odom = msg.platform_odom
            if not self._odom_valid(odom):
                self._target_world_queue.clear()
                x_cmd, y_cmd, z_cmd = float(x_body), float(y_body), float(z_body)
                fusion_note = "no valid platform_odom"
            else:
                q = odom.pose.pose.orientation
                R_i = self._quaternion_to_R_body_to_world(q.x, q.y, q.z, q.w)
                t_i = np.array(
                    [
                        odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z,
                    ],
                    dtype=np.float64,
                )
                #Matmult
                p_world_i = R_i @ p_body + t_i
                self._target_world_queue.append((p_world_i, conf))
                p_fused = self._weighted_world_mean(self._target_world_queue)
                
                if p_fused is None:
                    p_fused = p_world_i
                v_world = p_fused - t_i
                v_body = R_i.T @ v_world
                x_cmd, y_cmd, z_cmd = float(v_body[0]), float(v_body[1]), float(v_body[2])
                fusion_note = (
                    f"fused_world=({p_fused[0]:.3f},{p_fused[1]:.3f},{p_fused[2]:.3f}) "
                    f"queue={len(self._target_world_queue)}"
                )

            cmd = PoseStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.pose.position.x = x_cmd
            cmd.pose.position.y = y_cmd
            cmd.pose.position.z = z_cmd
            cmd.pose.orientation.w = 1.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"Published cmd_pose from detection | "
                f"conf={conf:.3f} | "
                f"bbox=({x1},{y1})-({x2},{y2}) | "
                f"depth={depth_m:.3f} m | "
                f"cam=({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) | "
                f"body_instant=({x_body:.3f}, {y_body:.3f}, {z_body:.3f}) | "
                f"cmd_body=({x_cmd:.3f}, {y_cmd:.3f}, {z_cmd:.3f}) | "
                f"{fusion_note}"
            )

        except Exception as e:
            self.get_logger().error(f"Detection bridge error: {e}")

    def decode_depth_image_to_meters(self, depth_msg):
        np_arr = np.frombuffer(depth_msg.data, np.uint8)
        depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        if depth_img is None:
            raise ValueError(
                f"Failed to decode depth image, format='{depth_msg.format}'"
            )

        if depth_img.dtype == np.uint16:
            return depth_img.astype(np.float32) / 1000.0

        if depth_img.dtype == np.float32:
            return depth_img

        raise ValueError(f"Unsupported depth dtype: {depth_img.dtype}")

    def sample_depth_median(self, depth_img_m, u, v, radius):
        h, w = depth_img_m.shape[:2]

        if w == 0 or h == 0:
            return None

        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        u0 = max(0, u - radius)
        u1 = min(w, u + radius + 1)
        v0 = max(0, v - radius)
        v1 = min(h, v + radius + 1)

        patch = depth_img_m[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = BridgeDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()