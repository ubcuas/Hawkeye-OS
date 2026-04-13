#!/usr/bin/env python3
# Integration test: bridge_detection + drone_control + mavros.
# Publishes /camera/depth/camera_info and /object_detection/tagged_image; checks pose like test_nav.
# Scenarios = body (fwd,left,up) m; pixels via pinhole matching rs2_deproject (_pixel_and_depth_for_body_vector).
# Typical SITL: near origin, ~5 m AGL. If drone_control clamps Z, analytic endpoint may differ.
# Bridge-only guards: test_bridge_edge_cases. Cmd-only edges: test_drone_control_edges.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Point32, PoseStamped
from hawkeye_msgs.msg import TaggedImage
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool

_LOG_RULE = "=================================================="


@dataclass
class Point3:
    x: float
    y: float
    z: float


@dataclass
class Scenario:
    """Object position relative to drone body at command time (x forward, y left, z up)."""

    forward_m: float
    left_m: float
    up_m: float


def _parse_scenarios_param(raw: str) -> List[Scenario]:
    out: List[Scenario] = []
    for part in raw.split("|"):
        part = part.strip()
        if not part:
            continue
        bits = [b.strip() for b in part.split(",")]
        if len(bits) != 3:
            raise ValueError(
                f"Bad scenario '{part}': expected 'forward,left,up' (three comma-separated numbers)"
            )
        out.append(Scenario(forward_m=float(bits[0]), left_m=float(bits[1]), up_m=float(bits[2])))
    if not out:
        raise ValueError("At least one scenario is required.")
    return out


def _pixel_and_depth_for_body_vector(
    scen: Scenario, fx: float, fy: float, cx: float, cy: float, width: int, height: int
) -> Tuple[int, int, float]:
    """
    Optical frame (RealSense): x right, y down, z forward.
    Body frame (bridge): x forward, y left, z up.
    """
    z_cam = scen.forward_m
    if z_cam <= 0.05:
        raise ValueError(f"forward_m must be > 0.05, got {z_cam}")

    x_cam = -scen.left_m
    y_cam = -scen.up_m

    u = cx + fx * x_cam / z_cam
    v = cy + fy * y_cam / z_cam

    margin = 8
    if not (margin <= u < width - margin and margin <= v < height - margin):
        raise ValueError(
            f"Scenario {scen} projects to pixel ({u:.1f}, {v:.1f}) outside image "
            f"({width}x{height}); choose smaller lateral/vertical offsets or larger forward_m."
        )

    return int(round(u)), int(round(v)), float(z_cam)


class BridgePipelineTestNode(Node):
    def __init__(self):
        super().__init__("bridge_pipeline_test_node")

        # ==========================
        # Tunable parameters
        # ==========================
        self.declare_parameter("scenarios", "10.0,0.0,0.0|9.0,2.0,0.0|10.0,0.0,-0.5")
        self.declare_parameter("desired_standoff_m", 1.5)
        self.declare_parameter("endpoint_tol_m", 0.6)
        self.declare_parameter("altitude_tol_m", 0.20)
        self.declare_parameter("stable_time_s", 2.0)
        self.declare_parameter("per_scenario_timeout_s", 120.0)
        self.declare_parameter("mission_start_timeout_s", 5.0)
        self.declare_parameter("inter_scenario_pause_s", 2.0)
        self.declare_parameter("status_period_s", 1.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("intrinsics_fx", 600.0)
        self.declare_parameter("intrinsics_fy", 600.0)
        self.declare_parameter("intrinsics_cx", 320.0)
        self.declare_parameter("intrinsics_cy", 240.0)

        scenarios_raw = str(self.get_parameter("scenarios").value)
        self.scenarios = _parse_scenarios_param(scenarios_raw)
        self.desired_standoff_m = float(self.get_parameter("desired_standoff_m").value)
        self.endpoint_tol_m = float(self.get_parameter("endpoint_tol_m").value)
        self.altitude_tol_m = float(self.get_parameter("altitude_tol_m").value)
        self.stable_time_s = float(self.get_parameter("stable_time_s").value)
        self.per_scenario_timeout_s = float(self.get_parameter("per_scenario_timeout_s").value)
        self.mission_start_timeout_s = float(self.get_parameter("mission_start_timeout_s").value)
        self.inter_scenario_pause_s = float(self.get_parameter("inter_scenario_pause_s").value)
        self.status_period_s = float(self.get_parameter("status_period_s").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.intrinsics_fx = float(self.get_parameter("intrinsics_fx").value)
        self.intrinsics_fy = float(self.get_parameter("intrinsics_fy").value)
        self.intrinsics_cx = float(self.get_parameter("intrinsics_cx").value)
        self.intrinsics_cy = float(self.get_parameter("intrinsics_cy").value)

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
            mission_state_qos,
        )

        # Depth intrinsics Publisher (for bridge_detection)
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "/camera/depth/camera_info",
            qos_reliable,
        )
        # Synthetic detection Publisher
        self.tagged_pub = self.create_publisher(
            TaggedImage,
            "/object_detection/tagged_image",
            qos_reliable,
        )

        # Timer for test state machine
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Current vehicle pose
        self.current_pose: Optional[PoseStamped] = None
        # Current FCU state
        self.current_state = State()
        self.mission_active = False

        # Test status
        self.test_done = False
        self.scenario_index = 0
        # Phases: wait_ready | pause | monitor_start | monitor_end | verify_stable
        self.phase = "wait_ready"

        # Cached scenario geometry
        self.command_pose: Optional[PoseStamped] = None
        self.object_target_world: Optional[Point3] = None
        self.expected_endpoint_world: Optional[Point3] = None

        # Timing
        self.scenario_start_time = None
        self.pause_until = None
        self.saw_mission_true = False
        self.last_status_elapsed = 0.0
        self.in_tolerance_since = None

        self.get_logger().info(
            f"bridge_pipeline_test_node: started. Scenarios={len(self.scenarios)}. "
            "Ensure bridge_detection + drone_control + mavros."
        )

    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def mission_cb(self, msg: Bool):
        self.mission_active = msg.data

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

    def _body_cmd_to_object_world(self, ref: PoseStamped, body: Point3) -> Point3:
        yaw = self._yaw_from_pose(ref)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx_world = cos_yaw * body.x - sin_yaw * body.y
        dy_world = sin_yaw * body.x + cos_yaw * body.y
        p = self._pose_to_point(ref)
        return Point3(
            x=p.x + dx_world,
            y=p.y + dy_world,
            z=p.z + body.z,
        )

    def _compute_expected_endpoint(self, ref: PoseStamped, object_world: Point3) -> Point3:
        start = self._pose_to_point(ref)

        vx = object_world.x - start.x
        vy = object_world.y - start.y
        vz = object_world.z - start.z

        vmag = math.sqrt(vx * vx + vy * vy + vz * vz)
        if vmag < 1e-6:
            raise ValueError("Target too close to reference pose; cannot form direction vector.")

        if vmag <= self.desired_standoff_m:
            raise ValueError(
                f"Target is only {vmag:.3f} m away, which is <= desired standoff "
                f"{self.desired_standoff_m:.3f} m."
            )

        ux = vx / vmag
        uy = vy / vmag
        uz = vz / vmag

        travel_dist = vmag - self.desired_standoff_m

        return Point3(
            x=start.x + ux * travel_dist,
            y=start.y + uy * travel_dist,
            z=start.z + uz * travel_dist,
        )

    def _publish_camera_info(self):
        fx, fy, cx, cy = self.intrinsics_fx, self.intrinsics_fy, self.intrinsics_cx, self.intrinsics_cy
        w, h = self.image_width, self.image_height
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height = h
        msg.width = w
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(msg)

    def _build_tagged_image(self, scen: Scenario) -> Tuple[TaggedImage, Point32, Point32]:
        u, v, depth_m = _pixel_and_depth_for_body_vector(
            scen,
            self.intrinsics_fx,
            self.intrinsics_fy,
            self.intrinsics_cx,
            self.intrinsics_cy,
            self.image_width,
            self.image_height,
        )

        depth_mm = int(round(depth_m * 1000.0))
        depth_img = np.full((self.image_height, self.image_width), depth_mm, dtype=np.uint16)
        ok, enc_depth = cv2.imencode(".png", depth_img)
        if not ok:
            raise RuntimeError("Failed to encode depth image")

        color_img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        ok_c, enc_color = cv2.imencode(".png", color_img)
        if not ok_c:
            raise RuntimeError("Failed to encode color image")

        half = 5
        p1 = Point32(x=float(u - half), y=float(v - half), z=0.0)
        p2 = Point32(x=float(u + half), y=float(v + half), z=0.0)

        msg = TaggedImage()
        msg.image_data = CompressedImage()
        msg.image_data.format = "png"
        msg.image_data.data = enc_color.tobytes()
        msg.depth_data = CompressedImage()
        msg.depth_data.format = "png"
        msg.depth_data.data = enc_depth.tobytes()
        msg.bounding_box = [p1, p2]
        msg.confidence_level = 0.95
        return msg, p1, p2

    def _publish_scenario(self, scen: Scenario):
        assert self.current_pose is not None
        ref = PoseStamped()
        ref.header = self.current_pose.header
        ref.pose = self.current_pose.pose

        body = Point3(x=scen.forward_m, y=scen.left_m, z=scen.up_m)
        self.command_pose = ref
        self.object_target_world = self._body_cmd_to_object_world(ref, body)
        self.expected_endpoint_world = self._compute_expected_endpoint(ref, self.object_target_world)

        tagged, p1, p2 = self._build_tagged_image(scen)
        self.tagged_pub.publish(tagged)

        self.scenario_start_time = self.get_clock().now()
        self.saw_mission_true = False
        self.in_tolerance_since = None
        self.last_status_elapsed = 0.0

        yaw_deg = math.degrees(self._yaw_from_pose(ref))
        sp = self._pose_to_point(ref)
        sn = self.scenario_index + 1
        sm = len(self.scenarios)
        self.get_logger().info(_LOG_RULE)
        self.get_logger().info(f"TEST SCENARIO {sn}/{sm} STARTED (bridge pipeline)")
        self.get_logger().info("Publishing TaggedImage -> expect bridge -> /drone/cmd_pose")
        self.get_logger().info(
            f"Reference pose: ({sp.x:.3f}, {sp.y:.3f}, {sp.z:.3f}), yaw={yaw_deg:.1f} deg"
        )
        self.get_logger().info(
            f"Simulated object body vector (fwd,left,up): "
            f"({scen.forward_m:.3f}, {scen.left_m:.3f}, {scen.up_m:.3f}) m"
        )
        self.get_logger().info(
            f"Expected object world: "
            f"({self.object_target_world.x:.3f}, {self.object_target_world.y:.3f}, "
            f"{self.object_target_world.z:.3f})"
        )
        self.get_logger().info(
            f"Expected stand-off endpoint: "
            f"({self.expected_endpoint_world.x:.3f}, {self.expected_endpoint_world.y:.3f}, "
            f"{self.expected_endpoint_world.z:.3f})"
        )
        self.get_logger().info(f"BBox corners: ({p1.x:.0f},{p1.y:.0f})-({p2.x:.0f},{p2.y:.0f})")
        self.get_logger().info(_LOG_RULE)

    def _finish_all(self, passed: bool, reason: str):
        if self.test_done:
            return
        self.test_done = True
        self.get_logger().info(_LOG_RULE)
        if passed:
            self.get_logger().info(f"TEST PASS: {reason}")
        else:
            self.get_logger().error(f"TEST FAIL: {reason}")
        self.get_logger().info(_LOG_RULE)
        raise SystemExit(0 if passed else 1)

    def _finish_scenario_fail(self, reason: str):
        self._finish_all(False, f"Scenario {self.scenario_index + 1}/{len(self.scenarios)}: {reason}")

    def _advance_scenario(self):
        self.scenario_index += 1
        self.phase = "pause"
        self.pause_until = self.get_clock().now() + Duration(seconds=self.inter_scenario_pause_s)
        self.get_logger().info(
            f"Scenario {self.scenario_index}/{len(self.scenarios)} complete. "
            f"Pausing {self.inter_scenario_pause_s:.1f}s before next."
        )

    def timer_cb(self):
        if self.test_done:
            return

        self._publish_camera_info()

        if self.current_pose is None:
            return

        now = self.get_clock().now()

        if self.phase == "wait_ready":
            if not self.current_state.connected or not self.current_state.armed:
                return
            if self.current_state.mode != "GUIDED":
                return
            if self.mission_active:
                return
            self.phase = "pause"
            self.pause_until = now
            self.get_logger().info(
                "Vehicle ready (GUIDED, armed, mission inactive). Starting scenario sequence."
            )
            return

        if self.phase == "pause":
            assert self.pause_until is not None
            if now < self.pause_until:
                return
            if self.scenario_index >= len(self.scenarios):
                self._finish_all(True, f"Completed {len(self.scenarios)} scenario(s).")
                return
            try:
                self._publish_scenario(self.scenarios[self.scenario_index])
            except (ValueError, RuntimeError) as exc:
                self._finish_all(False, str(exc))
                return
            self.phase = "monitor_start"
            return

        if self.phase == "monitor_start":
            assert self.scenario_start_time is not None
            elapsed = (now - self.scenario_start_time).nanoseconds / 1e9

            if self.mission_active:
                self.saw_mission_true = True
                self.phase = "monitor_end"
                self.get_logger().info("Mission became active (drone_control accepted cmd_pose).")
                return

            if elapsed >= self.mission_start_timeout_s:
                self._finish_scenario_fail(
                    f"No mission start within {self.mission_start_timeout_s:.1f}s "
                    "(check bridge_detection logs, intrinsics, confidence, bbox)."
                )
            return

        if self.phase == "monitor_end":
            assert self.scenario_start_time is not None
            assert self.expected_endpoint_world is not None
            assert self.object_target_world is not None
            elapsed = (now - self.scenario_start_time).nanoseconds / 1e9

            if elapsed - self.last_status_elapsed >= self.status_period_s:
                self.last_status_elapsed = elapsed
                cur = self._pose_to_point(self.current_pose)
                endpoint_err_xy = self._dist_xy(cur, self.expected_endpoint_world)
                alt_err = abs(cur.z - self.expected_endpoint_world.z)
                object_dist = self._dist_3d(cur, self.object_target_world)
                sn = self.scenario_index + 1
                sm = len(self.scenarios)
                self.get_logger().info(
                    f"Monitoring | scenario={sn}/{sm} | elapsed={elapsed:.1f}s | "
                    f"mission_active={self.mission_active} | "
                    f"pose=({cur.x:.3f}, {cur.y:.3f}, {cur.z:.3f}) | "
                    f"endpoint_err_xy={endpoint_err_xy:.3f} m | alt_err={alt_err:.3f} m | "
                    f"dist_to_object={object_dist:.3f} m"
                )

            if self.saw_mission_true and not self.mission_active:
                self.phase = "verify_stable"
                self.in_tolerance_since = None
                self.get_logger().info("Mission inactive — verifying stand-off pose.")
                return

            if elapsed >= self.per_scenario_timeout_s:
                self._finish_scenario_fail(
                    f"Timed out after {elapsed:.1f}s waiting for mission to complete."
                )
            return

        if self.phase == "verify_stable":
            assert self.expected_endpoint_world is not None
            assert self.object_target_world is not None
            assert self.scenario_start_time is not None

            elapsed = (now - self.scenario_start_time).nanoseconds / 1e9
            current = self._pose_to_point(self.current_pose)
            endpoint_err_xy = self._dist_xy(current, self.expected_endpoint_world)
            altitude_err = abs(current.z - self.expected_endpoint_world.z)

            if self.mission_active:
                self._finish_scenario_fail("mission_active became true during verification window.")

            in_tol = endpoint_err_xy <= self.endpoint_tol_m and altitude_err <= self.altitude_tol_m

            if in_tol:
                if self.in_tolerance_since is None:
                    self.in_tolerance_since = now
                stable_s = (now - self.in_tolerance_since).nanoseconds / 1e9
                if stable_s >= self.stable_time_s:
                    sn = self.scenario_index + 1
                    sm = len(self.scenarios)
                    self.get_logger().info(
                        f"TEST PASS: scenario {sn}/{sm} — within tolerance for {stable_s:.1f}s"
                    )
                    cur = current
                    self.get_logger().info(
                        f"Final pose: ({cur.x:.3f}, {cur.y:.3f}, {cur.z:.3f}) | "
                        f"endpoint_xy_err={endpoint_err_xy:.3f} m | alt_err={altitude_err:.3f} m | "
                        f"dist_to_object={self._dist_3d(cur, self.object_target_world):.3f} m"
                    )
                    self._advance_scenario()
                return

            self.in_tolerance_since = None

            if elapsed >= self.per_scenario_timeout_s:
                self._finish_scenario_fail(
                    f"After mission end, pose not within tolerance within {elapsed:.1f}s "
                    f"(endpoint_xy_err={endpoint_err_xy:.3f} m, alt_err={altitude_err:.3f} m)."
                )
            return


def main():
    rclpy.init()
    node = BridgePipelineTestNode()

    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bridge_pipeline_test_node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
