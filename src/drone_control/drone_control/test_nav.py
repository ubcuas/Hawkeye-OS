#!/usr/bin/env python3
# Integration test: publishes /drone/cmd_pose and checks stand-off vs /mavros/local_position/pose.
# Requires mavros + drone_control (GUIDED, armed). See test_bridge_pipeline for bridge path.

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

_LOG_RULE = "=================================================="


@dataclass
class Point3:
    x: float
    y: float
    z: float


class NavTestNode(Node):

    def __init__(self):
        super().__init__("nav_test_node")

        # ==========================
        # Tunable parameters
        # ==========================
        self.declare_parameter("target_world_dx", 10.0)
        self.declare_parameter("target_world_dy", 0.0)
        self.declare_parameter("target_world_dz", 0.0)
        self.declare_parameter("desired_standoff_m", 1.5)
        self.declare_parameter("endpoint_tol_m", 0.6)
        self.declare_parameter("altitude_tol_m", 0.20)
        self.declare_parameter("stable_time_s", 2.0)
        self.declare_parameter("timeout_s", 60.0)
        self.declare_parameter("status_period_s", 1.0)

        self.target_world_dx = float(self.get_parameter("target_world_dx").value)
        self.target_world_dy = float(self.get_parameter("target_world_dy").value)
        self.target_world_dz = float(self.get_parameter("target_world_dz").value)
        self.desired_standoff_m = float(self.get_parameter("desired_standoff_m").value)
        self.endpoint_tol_m = float(self.get_parameter("endpoint_tol_m").value)
        self.altitude_tol_m = float(self.get_parameter("altitude_tol_m").value)
        self.stable_time_s = float(self.get_parameter("stable_time_s").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.status_period_s = float(self.get_parameter("status_period_s").value)

        # QoS Profile for mavros communication protocols
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Position Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_cb,
            qos_profile_sensor_data,
        )

        # State Subscriber
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_cb,
            qos_reliable,
        )

        # Command Publisher
        self.cmd_pub = self.create_publisher(
            PoseStamped,
            "/drone/cmd_pose",
            qos_reliable,
        )

        # Timer for test state machine
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Current vehicle pose
        self.current_pose: Optional[PoseStamped] = None

        # Current FCU state
        self.current_state = State()

        # Test status flags
        self.command_sent = False
        self.test_done = False

        # Cached test setup data
        self.start_pose: Optional[PoseStamped] = None
        self.object_target_world: Optional[Point3] = None
        self.expected_endpoint_world: Optional[Point3] = None
        self.command_body: Optional[Point3] = None

        # Timing variables
        self.send_time = None
        self.last_status_time = 0.0
        self.in_tolerance_since = None

        self.get_logger().info(
            "nav_test_node: started. Waiting for pose + GUIDED/armed. Ensure drone_control is running."
        )

    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _yaw_from_pose(self, pose: PoseStamped) -> float:
        qx = float(pose.pose.orientation.x)
        qy = float(pose.pose.orientation.y)
        qz = float(pose.pose.orientation.z)
        qw = float(pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _pose_to_point(self, pose: PoseStamped) -> Point3:
        return Point3(
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            z=float(pose.pose.position.z),
        )

    def _dist_3d(self, a: Point3, b: Point3) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def _dist_xy(self, a: Point3, b: Point3) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def _build_body_frame_command(self, start_pose: PoseStamped, object_target_world: Point3) -> Point3:
        # Convert designated world-frame target into the body-frame relative command; expected by drone_control.py.
        start = self._pose_to_point(start_pose)
        yaw = self._yaw_from_pose(start_pose)

        dx_world = object_target_world.x - start.x
        dy_world = object_target_world.y - start.y
        dz_world = object_target_world.z - start.z

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        dx_body = cos_yaw * dx_world + sin_yaw * dy_world
        dy_body = -sin_yaw * dx_world + cos_yaw * dy_world

        return Point3(dx_body, dy_body, dz_world)

    def _compute_expected_endpoint(self, start_pose: PoseStamped, object_target_world: Point3) -> Point3:
        start = self._pose_to_point(start_pose)

        vx = object_target_world.x - start.x
        vy = object_target_world.y - start.y
        vz = object_target_world.z - start.z

        vmag = math.sqrt(vx * vx + vy * vy + vz * vz)
        if vmag < 1e-6:
            raise ValueError("Target too close to start pose; cannot form direction vector.")

        if vmag <= self.desired_standoff_m:
            raise ValueError(
                f"Target is only {vmag:.3f} m away, which is <= desired standoff "
                f"{self.desired_standoff_m:.3f} m."
            )

        # Unit direction toward target
        ux = vx / vmag
        uy = vy / vmag
        uz = vz / vmag

        # Travel distance stops short of object by desired standoff
        travel_dist = vmag - self.desired_standoff_m

        return Point3(
            x=start.x + ux * travel_dist,
            y=start.y + uy * travel_dist,
            z=start.z + uz * travel_dist,
        )

    def _send_command_once(self):
        assert self.current_pose is not None

        # Snapshot starting pose
        self.start_pose = self.current_pose
        start = self._pose_to_point(self.start_pose)

        # Build designated object target in world frame
        self.object_target_world = Point3(
            x=start.x + self.target_world_dx,
            y=start.y + self.target_world_dy,
            z=start.z + self.target_world_dz,
        )

        # Build expected command and expected final endpoint
        self.command_body = self._build_body_frame_command(self.start_pose, self.object_target_world)
        self.expected_endpoint_world = self._compute_expected_endpoint(self.start_pose, self.object_target_world)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.command_body.x
        msg.pose.position.y = self.command_body.y
        msg.pose.position.z = self.command_body.z
        msg.pose.orientation.w = 1.0

        self.cmd_pub.publish(msg)

        # Mark command as sent and initialize timers
        self.command_sent = True
        self.send_time = self.get_clock().now()
        self.in_tolerance_since = None

        yaw_deg = math.degrees(self._yaw_from_pose(self.start_pose))

        self.get_logger().info(_LOG_RULE)
        self.get_logger().info("TEST STARTED (nav / cmd_pose)")
        self.get_logger().info(
            f"Start pose: ({start.x:.3f}, {start.y:.3f}, {start.z:.3f}), yaw={yaw_deg:.1f} deg"
        )
        self.get_logger().info(
            f"Designated world target (object): "
            f"({self.object_target_world.x:.3f}, {self.object_target_world.y:.3f}, {self.object_target_world.z:.3f})"
        )
        self.get_logger().info(
            f"Published body-frame command: "
            f"({self.command_body.x:.3f}, {self.command_body.y:.3f}, {self.command_body.z:.3f})"
        )
        self.get_logger().info(
            f"Expected final stand-off point: "
            f"({self.expected_endpoint_world.x:.3f}, {self.expected_endpoint_world.y:.3f}, {self.expected_endpoint_world.z:.3f})"
        )
        self.get_logger().info(_LOG_RULE)

    def _finish(self, passed: bool, reason: str):
        if self.test_done:
            return

        self.test_done = True
        current = self._pose_to_point(self.current_pose) if self.current_pose is not None else None

        self.get_logger().info(_LOG_RULE)
        if passed:
            self.get_logger().info(f"TEST PASS: {reason}")
        else:
            self.get_logger().error(f"TEST FAIL: {reason}")

        if current is not None and self.expected_endpoint_world is not None and self.object_target_world is not None:
            endpoint_err_xy = self._dist_xy(current, self.expected_endpoint_world)
            altitude_err = abs(current.z - self.expected_endpoint_world.z)
            object_dist = self._dist_3d(current, self.object_target_world)

            self.get_logger().info(
                f"Final pose: ({current.x:.3f}, {current.y:.3f}, {current.z:.3f})"
            )
            self.get_logger().info(
                f"Expected endpoint error XY: {endpoint_err_xy:.3f} m"
            )
            self.get_logger().info(
                f"Altitude error: {altitude_err:.3f} m"
            )
            self.get_logger().info(
                f"Distance to object target: {object_dist:.3f} m "
                f"(expected about {self.desired_standoff_m:.3f} m)"
            )

        self.get_logger().info(_LOG_RULE)
        raise SystemExit(0 if passed else 1)

    def timer_cb(self):
        if self.test_done:
            return

        if self.current_pose is None:
            return

        # Wait until the vehicle is in a state where drone_control.py would accept the command
        if not self.command_sent:
            if not self.current_state.connected:
                return
            if not self.current_state.armed:
                return
            if self.current_state.mode != "GUIDED":
                return

            try:
                self._send_command_once()
            except ValueError as exc:
                self._finish(False, str(exc))
            return

        # After command is sent, monitor progress
        assert self.current_pose is not None
        assert self.expected_endpoint_world is not None
        assert self.object_target_world is not None
        assert self.send_time is not None

        now = self.get_clock().now()
        elapsed = (now - self.send_time).nanoseconds / 1e9

        current = self._pose_to_point(self.current_pose)
        endpoint_err_xy = self._dist_xy(current, self.expected_endpoint_world)
        altitude_err = abs(current.z - self.expected_endpoint_world.z)
        object_dist = self._dist_3d(current, self.object_target_world)

        # Periodic status logging
        if elapsed - self.last_status_time >= self.status_period_s:
            self.last_status_time = elapsed
            self.get_logger().info(
                f"Monitoring | elapsed={elapsed:.1f}s | "
                f"pose=({current.x:.3f}, {current.y:.3f}, {current.z:.3f}) | "
                f"endpoint_err_xy={endpoint_err_xy:.3f} m | "
                f"alt_err={altitude_err:.3f} m | "
                f"dist_to_object={object_dist:.3f} m"
            )

        # Check whether the drone is within endpoint and altitude tolerance
        in_tol = endpoint_err_xy <= self.endpoint_tol_m and altitude_err <= self.altitude_tol_m

        if in_tol:
            if self.in_tolerance_since is None:
                self.in_tolerance_since = now
            stable_elapsed = (now - self.in_tolerance_since).nanoseconds / 1e9
            if stable_elapsed >= self.stable_time_s:
                self._finish(
                    True,
                    f"Reached expected stand-off point within tolerance for {stable_elapsed:.1f}s",
                )
            return
        else:
            self.in_tolerance_since = None

        # Fail if the test times out
        if elapsed >= self.timeout_s:
            self._finish(
                False,
                f"Timed out after {elapsed:.1f}s without reaching the expected stand-off point",
            )


def main():
    rclpy.init()
    node = NavTestNode()

    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down nav_test_node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
