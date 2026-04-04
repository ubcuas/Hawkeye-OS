#!/usr/bin/env python3
# SITL edge tests for drone_control (no bridge). ros2 run drone_control test_drone_control_edges
# Requires mavros + drone_control like test_nav (GUIDED, armed, pose).
# Phase 1: body cmd closer than standoff -> mission abort, little motion.
# Phase 2: large negative body z -> /mavros/setpoint_position/local z never below MIN_Z_M.

from __future__ import annotations

import math
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
from std_msgs.msg import Bool

# Must match drone_control.py
MIN_Z_M = 0.5
DESIRED_DIST_M = 1.5

_LOG_RULE = "=================================================="


class DroneControlEdgeNode(Node):
    def __init__(self):
        super().__init__("drone_control_edge_node")

        self.declare_parameter("too_close_forward_m", 1.0)
        self.declare_parameter("clamp_body_forward_m", 10.0)
        self.declare_parameter("clamp_body_delta_z_m", -20.0)
        self.declare_parameter("phase_timeout_s", 45.0)
        self.declare_parameter("too_close_max_horiz_travel_m", 3.0)
        self.declare_parameter("setpoint_min_z_slack_m", 0.05)

        self.too_close_forward = float(self.get_parameter("too_close_forward_m").value)
        self.clamp_fwd = float(self.get_parameter("clamp_body_forward_m").value)
        self.clamp_dz = float(self.get_parameter("clamp_body_delta_z_m").value)
        self.phase_timeout_s = float(self.get_parameter("phase_timeout_s").value)
        self.too_close_max_horiz = float(self.get_parameter("too_close_max_horiz_travel_m").value)
        self.z_slack = float(self.get_parameter("setpoint_min_z_slack_m").value)

        # QoS Profile for mavros communication protocols
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        mission_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        # Mission-state Subscriber
        self.mission_sub = self.create_subscription(
            Bool,
            "/drone/mission_active",
            self.mission_cb,
            mission_qos,
        )
        # Setpoint Subscriber (drone_control output to mavros)
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            "/mavros/setpoint_position/local",
            self.setpoint_cb,
            qos_reliable,
        )
        # Command Publisher
        self.cmd_pub = self.create_publisher(PoseStamped, "/drone/cmd_pose", qos_reliable)

        # Timer for test state machine
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Current vehicle pose
        self.pose: Optional[PoseStamped] = None
        # Current FCU state
        self.state = State()
        self.mission_active = False

        # Phases: wait | too_close_run | too_close_done | clamp_run
        self.phase = "wait"
        self.start_xy = (0.0, 0.0)
        self.phase_start = None
        self.min_setpoint_z: Optional[float] = None
        self._saw_mission_too_close = False
        self._saw_mission_clamp = False
        self._done = False

        self.get_logger().info(
            "drone_control_edge_node: started. Phases=2. Waiting for GUIDED + armed + pose. "
            "Ensure drone_control + mavros."
        )

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def state_cb(self, msg: State):
        self.state = msg

    def mission_cb(self, msg: Bool):
        self.mission_active = msg.data

    def setpoint_cb(self, msg: PoseStamped):
        if self.phase != "clamp_run":
            return
        z = float(msg.pose.position.z)
        if self.min_setpoint_z is None or z < self.min_setpoint_z:
            self.min_setpoint_z = z

    def _fail(self, msg: str):
        if self._done:
            return
        self._done = True
        self.get_logger().info(_LOG_RULE)
        self.get_logger().error(f"TEST FAIL: {msg}")
        self.get_logger().info(_LOG_RULE)
        raise SystemExit(1)

    def _pass(self, msg: str):
        if self._done:
            return
        self._done = True
        self.get_logger().info(_LOG_RULE)
        self.get_logger().info(f"TEST PASS: {msg}")
        self.get_logger().info(_LOG_RULE)
        raise SystemExit(0)

    def _publish_cmd_body(self, fwd: float, left: float, up: float):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.pose.position.x = float(fwd)
        m.pose.position.y = float(left)
        m.pose.position.z = float(up)
        m.pose.orientation.w = 1.0
        self.cmd_pub.publish(m)
        self.get_logger().info(
            f"Published /drone/cmd_pose body-frame (x,y,z)=({fwd}, {left}, {up}) m"
        )

    def timer_cb(self):
        if self._done or self.pose is None:
            return

        now = self.get_clock().now()
        if self.phase == "wait":
            if not (self.state.connected and self.state.armed and self.state.mode == "GUIDED"):
                return
            if self.mission_active:
                return
            self.start_xy = (
                float(self.pose.pose.position.x),
                float(self.pose.pose.position.y),
            )
            self.phase = "too_close_run"
            self.phase_start = now
            self._saw_mission_too_close = False
            self.get_logger().info(_LOG_RULE)
            self.get_logger().info("TEST PHASE 1/2 STARTED (drone_control edge)")
            self.get_logger().info("Name: standoff-too-close abort")
            self.get_logger().info(
                f"Expect: mission ends with horizontal travel < {self.too_close_max_horiz:.1f} m"
            )
            self.get_logger().info(_LOG_RULE)
            if self.too_close_forward >= DESIRED_DIST_M:
                self._fail(
                    f"too_close_forward_m must be < {DESIRED_DIST_M} (standoff); got {self.too_close_forward}"
                )
            self._publish_cmd_body(self.too_close_forward, 0.0, 0.0)
            return

        if self.phase == "too_close_run":
            assert self.phase_start is not None
            elapsed = (now - self.phase_start).nanoseconds / 1e9
            x = float(self.pose.pose.position.x)
            y = float(self.pose.pose.position.y)
            horiz = math.hypot(x - self.start_xy[0], y - self.start_xy[1])

            if elapsed > self.phase_timeout_s:
                self._fail("Timeout waiting for too-close mission to finish.")

            if self.mission_active:
                self._saw_mission_too_close = True

            if self._saw_mission_too_close and not self.mission_active:
                if horiz > self.too_close_max_horiz:
                    self._fail(
                        f"Too-close case moved horizontally {horiz:.2f} m "
                        f"(expected mission abort with little motion)."
                    )
                self.get_logger().info(
                    f"TEST PASS: phase 1/2 — mission ended, horizontal travel {horiz:.2f} m"
                )
                self.phase = "too_close_done"
                self.phase_start = now
                self.min_setpoint_z = None
                self._saw_mission_clamp = False
            return

        if self.phase == "too_close_done":
            assert self.phase_start is not None
            if self.mission_active:
                return
            if (now - self.phase_start).nanoseconds / 1e9 < 1.5:
                return
            self.phase = "clamp_run"
            self.phase_start = now
            self._saw_mission_clamp = False
            self.get_logger().info(_LOG_RULE)
            self.get_logger().info("TEST PHASE 2/2 STARTED (drone_control edge)")
            self.get_logger().info("Name: altitude clamp via setpoints")
            self.get_logger().info(
                f"Expect: min setpoint z >= {MIN_Z_M - self.z_slack:.2f} m "
                f"(MIN_Z_M={MIN_Z_M}, slack={self.z_slack})"
            )
            self.get_logger().info(_LOG_RULE)
            self._publish_cmd_body(self.clamp_fwd, 0.0, self.clamp_dz)
            return

        if self.phase == "clamp_run":
            assert self.phase_start is not None
            elapsed = (now - self.phase_start).nanoseconds / 1e9

            if elapsed > self.phase_timeout_s:
                self._fail("Timeout in clamp test phase.")

            if self.mission_active:
                self._saw_mission_clamp = True

            if self._saw_mission_clamp and not self.mission_active:
                if self.min_setpoint_z is None:
                    self._fail("No /mavros/setpoint_position/local samples during clamp mission.")
                if self.min_setpoint_z + self.z_slack < MIN_Z_M:
                    self._fail(
                        f"Saw setpoint z min {self.min_setpoint_z:.3f} below MIN_Z_M ({MIN_Z_M})."
                    )
                self._pass(
                    f"Phases 1–2 OK; min setpoint z={self.min_setpoint_z:.3f} m (>= MIN_Z_M)"
                )
            return


def main():
    rclpy.init()
    node = DroneControlEdgeNode()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down drone_control_edge_node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
