import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Float32MultiArray
import threading
import time
import math
from dataclasses import dataclass


@dataclass
class ServoCommand:
    """Custom type for servo commands."""

    servo_num: float
    pwm_value: float
    count: float
    range_start: float
    range_end: float
    speed: float

    @classmethod
    def from_array(cls, msg: Float32MultiArray) -> "ServoCommand":
        return cls(servo_num=int(msg.data[0]), pwm_value=int(msg.data[1]))


# ==========================
# Tunable parameters
# ==========================
STEP_COUNT = 3
DWELL_SECONDS = 5.0
DESIRED_DIST_M = 1.5
DIST_TOL_M = 0.25
STEP_EPS = 0.30
SETPOINT_PERIOD_S = 0.05
STATUS_LOG_PERIOD_S = 1.0
DEBUG_FLAG = True


class ArduPilotNode(Node):
    def __init__(self):
        super().__init__("ardupilot_node")

        # Initialize target position
        self.target_pose = PoseStamped()

        # Initialize current position
        self.current_pose = PoseStamped()

        # Initialize current position
        self.reference_pose = PoseStamped()

        # Used so we don't try to access invalid reference/current pose
        self.have_pose = False

        # Variable for whether the drone is on a mission
        self.mission_active = False

        # Mission variables
        self.step_count = STEP_COUNT
        self.step_targets: list[PoseStamped] = []
        self.step_index = 0
        self.dwell_seconds: float = DWELL_SECONDS
        self.dwell_until = self.get_clock().now()
        self.in_dwell = False

        # QoS Profile for mavros communication protocols
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # State Subscriber
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_cb, qos_reliable
        )

        # Position Subscriber
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_cb,
            qos_profile_sensor_data,
        )

        # Command Subscriber
        self.cmd_sub = self.create_subscription(
            PoseStamped, "/drone/cmd_pose", self.command_cb, qos_reliable
        )

        # Location Publisher
        self.local_pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_reliable
        )

        # Water Servo Subcriber and Servo Client
        self.servo_client = self.create_client(CommandLong, "/mavros/cmd/command")

        self.cmd_set_servo_water = self.create_subscription(
            Float32MultiArray,
            "/drone/set_servo",
            self.command_set_servo_water_cb,
            qos_reliable,
        )
        self.cmd_set_servo_repeat = self.create_subscription(
            Float32MultiArray,
            "/drone/set_servo_repeat",
            self.command_set_servo_repeat_cb,
            qos_reliable,
        )

        # Timer for Setpoints
        self.timer = self.create_timer(SETPOINT_PERIOD_S, self.timer_cb)

        self.current_state = State()
        self.get_logger().info("Node initialized. Listening on /drone/cmd_pose")

    def cancel_mission(self):
        self.mission_active = False
        self.step_targets = []
        self.step_index = 0
        self.in_dwell = False
        self.dwell_until = self.get_clock().now()
        if self.have_pose:
            self.target_pose = self.copy_pose(self.current_pose)

    def state_cb(self, msg):
        self.current_state = msg

        if not self.mission_active:
            return

        if self.current_state.mode != "GUIDED":
            self.get_logger().warn("Left GUIDED -> canceling mission.")
            self.cancel_mission()
        elif (not self.current_state.armed) or (not self.current_state.connected):
            self.get_logger().warn("Disarmed/disconnected -> canceling mission.")
            self.cancel_mission()

    def copy_pose(self, src: PoseStamped) -> PoseStamped:
        dst = PoseStamped()
        dst.header.stamp = src.header.stamp
        dst.header.frame_id = src.header.frame_id
        dst.pose.position.x = src.pose.position.x
        dst.pose.position.y = src.pose.position.y
        dst.pose.position.z = src.pose.position.z
        dst.pose.orientation.x = src.pose.orientation.x
        dst.pose.orientation.y = src.pose.orientation.y
        dst.pose.orientation.z = src.pose.orientation.z
        dst.pose.orientation.w = src.pose.orientation.w
        return dst

    def local_pose_cb(self, msg):
        self.have_pose = True
        self.current_pose = msg

    def _yaw_from_pose(self, pose: PoseStamped) -> float:
        qx = float(pose.pose.orientation.x)
        qy = float(pose.pose.orientation.y)
        qz = float(pose.pose.orientation.z)
        qw = float(pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    # convert relative delta command from object detection into an absolute target
    def delta_to_target_pose(self, delta_msg: PoseStamped) -> PoseStamped:
        target = PoseStamped()

        # Relative position of the object in the drone's body frame
        dx_body = float(delta_msg.pose.position.x)
        dy_body = float(delta_msg.pose.position.y)

        yaw = self._yaw_from_pose(self.reference_pose)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        dx_world = cos_yaw * dx_body - sin_yaw * dy_body
        dy_world = sin_yaw * dx_body + cos_yaw * dy_body

        # Absolute "object" position = reference position + rotated horizontal delta
        target.pose.position.x = self.reference_pose.pose.position.x + dx_world
        target.pose.position.y = self.reference_pose.pose.position.y + dy_world
        target.pose.position.z = self.reference_pose.pose.position.z

        # Keep the reference orientation
        target.pose.orientation.x = self.reference_pose.pose.orientation.x
        target.pose.orientation.y = self.reference_pose.pose.orientation.y
        target.pose.orientation.z = self.reference_pose.pose.orientation.z
        target.pose.orientation.w = self.reference_pose.pose.orientation.w

        return target

    def set_target_yaw_facing_goal(self):
        cx = float(self.reference_pose.pose.position.x)
        cy = float(self.reference_pose.pose.position.y)
        fx = float(self.target_pose.pose.position.x)
        fy = float(self.target_pose.pose.position.y)

        dx = fx - cx
        dy = fy - cy
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return

        yaw = math.atan2(dy, dx)

        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = math.sin(yaw * 0.5)
        self.target_pose.pose.orientation.w = math.cos(yaw * 0.5)

    def command_cb(self, msg):
        if not self.have_pose:
            self.get_logger().warn("Command received but no pose yet. Ignoring.")
            return

        if self.mission_active:
            self.get_logger().warn("Mission already in progress. Skipping new command.")
            return

        if self.current_state.mode != "GUIDED" or not self.current_state.armed:
            self.get_logger().warn("Not armed or not in GUIDED. Ignoring command.")
            return

        self.reference_pose = self.copy_pose(self.current_pose)
        self.target_pose = self.delta_to_target_pose(msg)
        self.set_target_yaw_facing_goal()
        self.mission_active = True

        self.get_logger().info(
            f"Mission started -> final target: "
            f"({self.target_pose.pose.position.x:.3f}, "
            f"{self.target_pose.pose.position.y:.3f}, "
            f"{self.target_pose.pose.position.z:.3f})"
        )

    def timer_cb(self):
        if not (
            self.current_state.connected
            and self.current_state.armed
            and self.current_state.mode == "GUIDED"
            and self.have_pose
            and self.mission_active
        ):
            return

        now = self.get_clock().now()

        # Current Position
        cx = float(self.current_pose.pose.position.x)
        cy = float(self.current_pose.pose.position.y)
        cz = float(self.current_pose.pose.position.z)

        # Final Position
        fx = float(self.target_pose.pose.position.x)
        fy = float(self.target_pose.pose.position.y)
        fz = float(self.target_pose.pose.position.z)

        dist_to_obj = ((fx - cx) ** 2 + (fy - cy) ** 2 + (fz - cz) ** 2) ** 0.5

        if abs(dist_to_obj - DESIRED_DIST_M) <= DIST_TOL_M:
            self.get_logger().info(
                f"Mission terminating at "
                f"({cx:.2f}, {cy:.2f}, {cz:.2f}) "
                f"| distance={dist_to_obj:.3f}m at mission termination"
            )

            # Cancel mission
            self.cancel_mission()

            return

        if len(self.step_targets) == 0:
            sx, sy, sz = cx, cy, cz

            # Vector from start -> final
            vx = fx - sx
            vy = fy - sy
            vz = fz - sz
            vmag = (vx * vx + vy * vy + vz * vz) ** 0.5
            if vmag < 1e-6:
                # abort mission due to malformed v
                self.cancel_mission()
                return

            # Unit direction toward final
            ux = vx / vmag
            uy = vy / vmag
            uz = vz / vmag

            # Travel distance stops short of physical target by tolerance
            travel_dist = vmag - DESIRED_DIST_M

            if travel_dist <= 0.0:
                self.cancel_mission()
                return

            # Build N step targets along the line, equally spaced by travel_dist / step_count
            step_len = travel_dist / float(self.step_count)

            self.step_targets = []
            for i in range(self.step_count):
                step = PoseStamped()
                step.pose.position.x = sx + ux * step_len * float(i + 1)
                step.pose.position.y = sy + uy * step_len * float(i + 1)
                step.pose.position.z = sz + uz * step_len * float(i + 1)

                step.pose.orientation = self.target_pose.pose.orientation

                self.step_targets.append(step)

            self.step_index = 0
            self.in_dwell = False
            self.dwell_until = self.get_clock().now()

        # Clamp step_index
        if self.step_index < 0:
            self.step_index = 0
        if self.step_index >= len(self.step_targets):
            self.step_index = len(self.step_targets) - 1

        # Active step target
        active = self.step_targets[self.step_index]
        active.header.stamp = now.to_msg()
        active.header.frame_id = self.current_pose.header.frame_id

        # Publish setpoint
        self.local_pos_pub.publish(active)

        tx = float(active.pose.position.x)
        ty = float(active.pose.position.y)
        tz = float(active.pose.position.z)

        dx_s = tx - cx
        dy_s = ty - cy
        dz_s = tz - cz
        dist_to_step = (dx_s * dx_s + dy_s * dy_s + dz_s * dz_s) ** 0.5

        step_eps = STEP_EPS
        if dist_to_step > step_eps:
            return

        if not self.in_dwell:
            self.in_dwell = True
            self.dwell_until = now + Duration(seconds=self.dwell_seconds)
            self.get_logger().info(f"Entering dwell at step {self.step_index + 1}")
            return

        if now < self.dwell_until:
            return

        self.in_dwell = False

        if self.step_index < len(self.step_targets) - 1:
            self.step_index += 1
            self.get_logger().info(
                f"Advancing to step {self.step_index + 1}/{len(self.step_targets)}"
            )
            return

        return

    def command_set_servo_water_cb(self, msg: Float32MultiArray):
        """
        Callback to set a servo value via MAVROS.
        msg.data[0]: Servo number (e.g. 9 for AUX1)
        msg.data[1]: PWM value (e.g. 1000-2000)
        msg.data[2]: Count (ignore)
        msg.data[3]: range_start (ignore)
        msg.data[4]: range_end (ignore)
        msg.data[5]: Speed (ignore)
        """
        cmd = ServoCommand.from_array(msg)

        self.get_logger().info(
            f"RECEIVED SERVO COMMAND: servo={cmd.servo_num}, pwm={cmd.pwm_value}"
        )

        if not self.servo_client.service_is_ready():
            self.get_logger().warn(
                "MAVROS servo service not available - command logged only"
            )
            return

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183
        req.confirmation = 0
        req.param1 = float(cmd.servo_num)
        req.param2 = float(cmd.pwm_value)

        self.get_logger().info(
            f"Sending to MAVROS: Setting servo {cmd.servo_num} to PWM {cmd.pwm_value}"
        )

        future = self.servo_client.call_async(req)
        future.add_done_callback(self.servo_response_cb)

    def command_set_servo_repeat_cb(self, msg: Float32MultiArray):
        """
        Callback to set a servo value via MAVROS.
        msg.data[0]: Servo number (e.g. 9 for AUX1)
        msg.data[1]: PWM value (ignore)
        msg.data[2]: Count (amount of times to flap ts)
        msg.data[3]: range_start (degrees,internally calculate pwm)
        msg.data[4]: range_end (degrees,internally calculate pwm)
        msg.data[5]: Speed (internally calculated flap period)
        """
        cmd = ServoCommand.from_array(msg)

        self.get_logger().info(
            f"RECEIVED SERVO COMMAND: servo={cmd.servo_num}, count={cmd.count}, range = {cmd.range_start}, {cmd.range_end}, speed={cmd.speed}"
        )

        if not self.servo_client.service_is_ready():
            self.get_logger().warn(
                "MAVROS servo service not available - command logged only"
            )
            return

        time_for_one = 1 / cmd.speed
        increments = (cmd.range_end - cmd.range_start) / (time_for_one / 0.05)
        start_range = self.degrees_to_pwm(cmd.range_start)
        end_range = self.degrees_to_pwm(cmd.range_end)
        for i in range(cmd.count):
            while start_range < end_range:
                start_range_msg = Float32MultiArray
                start_range_msg.data = [cmd.servo_num, start_range, 0, 0, 0]
                self.command_set_servo_water_cb(msg)
                start_range += increments

    def degrees_to_pwm(self, target_angle: float) -> float:
        """
        Converts degrees to a PWM value
        """
        min_angle = 0.0
        max_angle = 180.0
        min_pwm = 1000.0
        max_pwm = 2000.0

        safe_angle = max(min_angle, min(target_angle, max_angle))

        proportion = (safe_angle - min_angle) / (max_angle - min_angle)
        pwm = min_pwm + (proportion * (max_pwm - min_pwm))

        return pwm

    def servo_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Servo command successful")
            else:
                self.get_logger().error(
                    f"Servo command failed, result: {response.result}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main():
    rclpy.init()
    node = ArduPilotNode()

    # Spin background threads (where all the work happens)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok() and not node.current_state.connected:
            node.get_logger().info("Waiting for connection...")
            time.sleep(STATUS_LOG_PERIOD_S)

        while rclpy.ok():
            if DEBUG_FLAG:
                node.get_logger().info(
                    f"STATUS: {node.current_state.mode} | "
                    f"ARMED: {node.current_state.armed} | "
                    f"MISSION: {node.mission_active} | "
                    f"POS: ({node.current_pose.pose.position.x:.3f}, "
                    f"{node.current_pose.pose.position.y:.3f}, "
                    f"{node.current_pose.pose.position.z:.3f})"
                )

            time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
