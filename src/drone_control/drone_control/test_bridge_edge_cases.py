#!/usr/bin/env python3
# Guard-rail tests for bridge_detection only (no mavros). ros2 run drone_control test_bridge_edge_cases
# Do not run with drone_control: step 1 publishes /drone/cmd_pose. Mission steps publish /drone/mission_active;
# if drone_control runs too, that topic may conflict — use run_mission_gating_tests:=false.

from __future__ import annotations

import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PoseStamped
from hawkeye_msgs.msg import TaggedImage
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, NavSatFix
from std_msgs.msg import Bool

_LOG_RULE = "=================================================="


def _camera_info(
    fx: float, fy: float, cx: float, cy: float, w: int, h: int, stamp
) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.height = h
    msg.width = w
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def _tagged_center(
    u: int,
    v: int,
    depth_m: float,
    confidence: float,
    bbox: List[Point32],
    depth_payload: Optional[bytes] = None,
    w: int = 640,
    h: int = 480,
) -> TaggedImage:
    msg = TaggedImage()
    color_img = np.zeros((h, w, 3), dtype=np.uint8)
    ok_c, enc_color = cv2.imencode(".png", color_img)
    if not ok_c:
        raise RuntimeError("color encode")

    if depth_payload is not None:
        enc_depth = depth_payload
    else:
        depth_mm = int(round(depth_m * 1000.0))
        depth_img = np.full((h, w), depth_mm, dtype=np.uint16)
        ok_d, enc = cv2.imencode(".png", depth_img)
        if not ok_d:
            raise RuntimeError("depth encode")
        enc_depth = enc.tobytes()

    msg.image_data = CompressedImage()
    msg.image_data.format = "png"
    msg.image_data.data = enc_color.tobytes()
    msg.depth_data = CompressedImage()
    msg.depth_data.format = "png"
    msg.depth_data.data = enc_depth
    msg.gps_data = NavSatFix()
    msg.bounding_box = bbox
    msg.confidence_level = float(confidence)
    return msg


class BridgeEdgeCaseNode(Node):
    def __init__(self):
        super().__init__("bridge_edge_case_node")

        # ==========================
        # Tunable parameters
        # ==========================
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("step_settle_s", 2.0)
        self.declare_parameter("run_mission_gating_tests", True)
        self.declare_parameter("publish_camera_info_hz", 5.0)

        self.w = int(self.get_parameter("image_width").value)
        self.h = int(self.get_parameter("image_height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.step_settle_s = float(self.get_parameter("step_settle_s").value)
        self.run_mission_gating = bool(self.get_parameter("run_mission_gating_tests").value)
        self.publish_camera_info_hz = float(self.get_parameter("publish_camera_info_hz").value)

        # QoS Profile for bridge topics
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

        # Count of /drone/cmd_pose messages received
        self.cmd_count = 0
        self._last_cmd_count_at_step_start = 0

        # Command Subscriber (bridge output)
        self.create_subscription(PoseStamped, "/drone/cmd_pose", self._cmd_cb, qos_reliable)

        # Depth intrinsics Publisher
        self.cam_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_reliable)
        # Synthetic detection Publisher
        self.tag_pub = self.create_publisher(TaggedImage, "/object_detection/tagged_image", qos_reliable)
        # Mission-state Publisher (optional steps)
        self.mission_pub = self.create_publisher(Bool, "/drone/mission_active", mission_qos)

        # Timer for scripted steps
        self.timer = self.create_timer(0.05, self._tick)

        self._test_done = False
        self._step = 0
        self._step_deadline: Optional[float] = None
        self._publish_intrinsics = False
        self._last_cam_pub_mono = 0.0
        self._current_step_title = ""

        self.get_logger().info(
            f"bridge_edge_case_node: started. Steps={self._num_steps()}. "
            f"mission_gating_tests={self.run_mission_gating}. Run bridge_detection only (no drone_control)."
        )

    def _cmd_cb(self, _msg: PoseStamped):
        self.cmd_count += 1

    def _num_steps(self) -> int:
        # Steps 0..6 always; 7..8 only when mission gating runs
        return 9 if self.run_mission_gating else 7

    def _publish_camera_info_now(self):
        # Publish one CameraInfo immediately (bypass rate limit)
        self._last_cam_pub_mono = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        self.cam_pub.publish(_camera_info(self.fx, self.fy, self.cx, self.cy, self.w, self.h, stamp))

    def _publish_cam_if_enabled(self):
        if not self._publish_intrinsics:
            return
        now = time.monotonic()
        period = 1.0 / max(self.publish_camera_info_hz, 0.1)
        if now - self._last_cam_pub_mono < period:
            return
        self._publish_camera_info_now()

    def _valid_bbox(self) -> List[Point32]:
        u, v = int(self.cx), int(self.cy)
        return [
            Point32(x=float(u - 5), y=float(v - 5), z=0.0),
            Point32(x=float(u + 5), y=float(v + 5), z=0.0),
        ]

    def _publish_valid_detection(self):
        msg = _tagged_center(
            int(self.cx), int(self.cy), 8.0, 0.95, self._valid_bbox(), w=self.w, h=self.h
        )
        self.tag_pub.publish(msg)

    def _fail(self, reason: str):
        if self._test_done:
            return
        self._test_done = True
        n = self._step + 1
        t = self._num_steps()
        self.get_logger().info(_LOG_RULE)
        self.get_logger().error(f"TEST FAIL: step {n}/{t} (bridge edge) — {reason}")
        self.get_logger().info(_LOG_RULE)
        raise SystemExit(1)

    def _pass_all(self):
        if self._test_done:
            return
        self._test_done = True
        self.get_logger().info(_LOG_RULE)
        self.get_logger().info(
            f"TEST PASS: completed all {self._num_steps()} steps (bridge edge-case suite)."
        )
        self.get_logger().info(_LOG_RULE)
        raise SystemExit(0)

    def _begin_step(self, title: str, expect: str):
        self._current_step_title = title
        n = self._step + 1
        t = self._num_steps()
        self.get_logger().info(_LOG_RULE)
        self.get_logger().info(f"TEST STEP {n}/{t} STARTED (bridge edge)")
        self.get_logger().info(f"Name: {title}")
        self.get_logger().info(f"Expect: {expect}")
        self.get_logger().info(_LOG_RULE)
        self._last_cmd_count_at_step_start = self.cmd_count
        self._step_deadline = time.monotonic() + self.step_settle_s

    def _pass_step(self):
        n = self._step + 1
        t = self._num_steps()
        self.get_logger().info(
            f"TEST PASS: step {n}/{t} — {self._current_step_title}"
        )

    def _delta_cmds(self) -> int:
        return self.cmd_count - self._last_cmd_count_at_step_start

    def _tick(self):
        if self._test_done:
            return

        self._publish_cam_if_enabled()

        # Step 0: no intrinsics yet — valid detection must NOT emit cmd
        if self._step == 0:
            if self._step_deadline is None:
                self._publish_intrinsics = False
                self._begin_step(
                    "No CameraInfo",
                    "no messages on /drone/cmd_pose after valid TaggedImage",
                )
                self._publish_valid_detection()
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("Received cmd_pose without CameraInfo (expected silence).")
                self._pass_step()
                self._step = 1
                self._step_deadline = None
            return

        # Step 1: enable intrinsics, valid detection -> expect >=1 cmd
        if self._step == 1:
            if self._step_deadline is None:
                # _publish_cam_if_enabled() already ran this tick while _publish_intrinsics was
                # still False (step 0). Publish CameraInfo before TaggedImage or bridge never
                # sees intrinsics for this detection.
                self._publish_intrinsics = True
                self._publish_camera_info_now()
                self._begin_step(
                    "CameraInfo + valid detection",
                    "at least one /drone/cmd_pose",
                )
                self._publish_valid_detection()
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() < 1:
                    self._fail("Expected at least one cmd_pose after intrinsics + valid detection.")
                self._pass_step()
                self._step = 2
                self._step_deadline = None
            return

        # Step 2: low confidence
        if self._step == 2:
            if self._step_deadline is None:
                self._begin_step(
                    "Low confidence",
                    "no new cmd_pose (conf < bridge MIN_DETECTION_CONF)",
                )
                msg = _tagged_center(
                    int(self.cx), int(self.cy), 8.0, 0.05, self._valid_bbox(), w=self.w, h=self.h
                )
                self.tag_pub.publish(msg)
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose published for low-confidence detection (should ignore).")
                self._pass_step()
                self._step = 3
                self._step_deadline = None
            return

        # Step 3: empty bbox
        if self._step == 3:
            if self._step_deadline is None:
                self._begin_step(
                    "Empty bounding box",
                    "no new cmd_pose",
                )
                msg = _tagged_center(int(self.cx), int(self.cy), 8.0, 0.95, [], w=self.w, h=self.h)
                self.tag_pub.publish(msg)
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose published with empty bbox.")
                self._pass_step()
                self._step = 4
                self._step_deadline = None
            return

        # Step 4: one-point bbox
        if self._step == 4:
            if self._step_deadline is None:
                self._begin_step(
                    "One-point bounding box",
                    "no new cmd_pose",
                )
                msg = _tagged_center(
                    int(self.cx),
                    int(self.cy),
                    8.0,
                    0.95,
                    [Point32(x=float(self.cx), y=float(self.cy), z=0.0)],
                    w=self.w,
                    h=self.h,
                )
                self.tag_pub.publish(msg)
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose published with degenerate bbox.")
                self._pass_step()
                self._step = 5
                self._step_deadline = None
            return

        # Step 5: invalid depth payload
        if self._step == 5:
            if self._step_deadline is None:
                self._begin_step(
                    "Corrupt depth PNG",
                    "no new cmd_pose (decode error)",
                )
                msg = _tagged_center(
                    int(self.cx),
                    int(self.cy),
                    8.0,
                    0.95,
                    self._valid_bbox(),
                    depth_payload=b"not-a-png",
                    w=self.w,
                    h=self.h,
                )
                self.tag_pub.publish(msg)
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose published after depth decode failure.")
                self._pass_step()
                self._step = 6
                self._step_deadline = None
            return

        # Step 6: all-zero depth (no valid median)
        if self._step == 6:
            if self._step_deadline is None:
                self._begin_step(
                    "All-zero depth image",
                    "no new cmd_pose (no valid depth at bbox center)",
                )
                z0 = np.zeros((self.h, self.w), dtype=np.uint16)
                ok_d, enc = cv2.imencode(".png", z0)
                if not ok_d:
                    self._fail("encode failed")
                msg = _tagged_center(
                    int(self.cx),
                    int(self.cy),
                    8.0,
                    0.95,
                    self._valid_bbox(),
                    depth_payload=enc.tobytes(),
                    w=self.w,
                    h=self.h,
                )
                self.tag_pub.publish(msg)
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose published with invalid depth patch.")
                self._pass_step()
                self._step = 7
                self._step_deadline = None
            return

        if not self.run_mission_gating:
            self._pass_all()
            return

        # Step 7: mission_active True -> ignore
        if self._step == 7:
            if self._step_deadline is None:
                self._begin_step(
                    "mission_active True",
                    "no new cmd_pose while /drone/mission_active is True",
                )
                self.mission_pub.publish(Bool(data=True))
                self._publish_valid_detection()
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() != 0:
                    self._fail("cmd_pose while mission_active=True (should ignore).")
                self._pass_step()
                self._step = 8
                self._step_deadline = None
            return

        # Step 8: mission_active False -> cmd again
        if self._step == 8:
            if self._step_deadline is None:
                self._begin_step(
                    "mission_active False",
                    "at least one new /drone/cmd_pose after clearing mission flag",
                )
                self.mission_pub.publish(Bool(data=False))
                self._publish_valid_detection()
            elif time.monotonic() >= self._step_deadline:
                if self._delta_cmds() < 1:
                    self._fail("Expected cmd_pose after clearing mission_active.")
                self._pass_step()
                self._pass_all()
            return


def main():
    rclpy.init()
    node = BridgeEdgeCaseNode()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
