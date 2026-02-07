import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import threading
import time

class ArduPilotNode(Node):
    def __init__(self):
        super().__init__('ardupilot_node')  

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

        # Used for manual overrides
        self.shutdown_requested = False

        # Mission variables
        self.step_count = 3
        self.step_targets: list[PoseStamped] = []
        self.step_index = 0
        self.dwell_seconds: float = 1.0
        self.dwell_until = rclpy.time.Time()
        self.in_dwell = False
        self.tolerance_m: float = 0.25

        # QoS Profile for mavros communication protocols
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # State Subscriber
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_cb, 
            qos_reliable
        )

        # Position Subscriber
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_cb,
            qos_profile_sensor_data
        )

        # Command Subscriber
        self.cmd_sub = self.create_subscription(
            PoseStamped, 
            '/drone/cmd_pose', 
            self.command_cb, 
            qos_reliable
        )

        # Location Publisher
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            qos_reliable
        )
        
        # Timer for Setpoints
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.current_state = State()
        self.get_logger().info("Node initialized. Listening on /drone/cmd_pose")

    def state_cb(self, msg):
        self.current_state = msg

        if (self.mission_active == True and self.current_state.mode != "GUIDED"):
            self.mission_active = False
            self.shutdown_requested = True
            self.mission_active = False
            self.step_targets = []
            self.step_index = 0
            self.in_dwell = False
            self.dwell_until = rclpy.time.Time()
            if self.have_pose:
                self.target_pose = self.copy_pose(self.current_pose)


    def copy_pose(self, src: PoseStamped) -> PoseStamped:
        dst = PoseStamped()
        dst.header = src.header
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

        if not self.mission_active:
            self.target_pose = self.copy_pose(self.current_pose)

    # convert relative delta command from SLAM into an absolute target
    def delta_to_target_pose(self, delta_msg: PoseStamped) -> PoseStamped:
        target = PoseStamped()

        dx = float(delta_msg.pose.position.x)
        dy = float(delta_msg.pose.position.y)
        dz = float(delta_msg.pose.position.z)

        # Absolute target = reference position + delta
        target.pose.position.x = self.reference_pose.pose.position.x + dx
        target.pose.position.y = self.reference_pose.pose.position.y + dy
        target.pose.position.z = self.reference_pose.pose.position.z + dz

        # Keep the reference orientation
        target.pose.orientation.x = self.reference_pose.pose.orientation.x
        target.pose.orientation.y = self.reference_pose.pose.orientation.y
        target.pose.orientation.z = self.reference_pose.pose.orientation.z
        target.pose.orientation.w = self.reference_pose.pose.orientation.w

        # Target received log
        self.get_logger().info(
            f"Delta received: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f} -> "
            f"Target: x={target.pose.position.x:.2f}, y={target.pose.position.y:.2f}, z={target.pose.position.z:.2f}"
        )

        return target


    def command_cb(self, msg):
        if (not self.have_pose or self.mission_active):
            return
        self.reference_pose = self.copy_pose(self.current_pose)
        self.target_pose = self.delta_to_target_pose(msg)
        self.mission_active = True

    def timer_cb(self):
        if not (self.current_state.connected and self.current_state.armed and self.current_state.mode == "GUIDED" and self.have_pose and self.mission_active):
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

        dx_f = fx - cx
        dy_f = fy - cy
        dz_f = fz - cz
        dist_to_final = (dx_f * dx_f + dy_f * dy_f + dz_f * dz_f) ** 0.5

        if dist_to_final <= self.tolerance_m:
            self.get_logger().info(f"Mission complete: within tolerance ({dist_to_final:.2f}m <= {self.tolerance_m:.2f}m).")

            # End mission + clear vars
            self.mission_active = False
            self.step_targets = []
            self.step_index = 0
            self.in_dwell = False
            self.dwell_until = rclpy.time.Time()

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
                self.mission_active = False
                return

            # Unit direction toward final
            ux = vx / vmag
            uy = vy / vmag
            uz = vz / vmag

            # Travel distance stops short of physical target by tolerance
            travel_dist = max(vmag - self.tolerance_m, 0.0)

            # Build N step targets along the line, equally spaced by travel_dist / step_count
            step_len = travel_dist / float(self.step_count)

            self.step_targets = []
            for i in range(self.step_count):
                step = PoseStamped()
                step.pose.position.x = sx + ux * step_len * float(i + 1)
                step.pose.position.y = sy + uy * step_len * float(i + 1)
                step.pose.position.z = sz + uz * step_len * float(i + 1)

                step.pose.orientation = self.current_pose.pose.orientation

                self.step_targets.append(step)

            self.step_index = 0
            self.in_dwell = False
            self.dwell_until = rclpy.time.Time()
        
        # Clamp step_index
        if self.step_index < 0:
            self.step_index = 0
        if self.step_index >= len(self.step_targets):
            self.step_index = len(self.step_targets) - 1

        # Active step target
        active = self.step_targets[self.step_index]
        active.header.stamp = now.to_msg()

        # Publish setpoint
        self.local_pos_pub.publish(active)

        tx = float(active.pose.position.x)
        ty = float(active.pose.position.y)
        tz = float(active.pose.position.z)

        dx_s = tx - cx
        dy_s = ty - cy
        dz_s = tz - cz
        dist_to_step = (dx_s * dx_s + dy_s * dy_s + dz_s * dz_s) ** 0.5

        step_eps = 0.10
        if dist_to_step > step_eps:
            return
        
        if not self.in_dwell:
            self.in_dwell = True
            self.dwell_until = now + Duration(seconds=float(self.dwell_seconds))
            return
        
        if now < self.dwell_until:
            return
        
        self.in_dwell = False

        if self.step_index < len(self.step_targets) - 1:
            self.step_index += 1
            return
        
        return

def main():
    rclpy.init()
    node = ArduPilotNode()

    # Spin background threads where all the work happens
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok() and not node.current_state.connected:
            node.get_logger().info("Waiting for connection...")
            time.sleep(1)

        while rclpy.ok():
            if node.shutdown_requested:
                node.get_logger().warn("Shutdown requested (manual override).")
                break

            # debugging only
            node.get_logger().info(
                f"STATUS: {node.current_state.mode} | "
                f"ARMED: {node.current_state.armed} | "
                f"MISSION: {node.mission_active} | "
                f"POS: ({node.current_pose.pose.position.x:.1f}, "
                f"{node.current_pose.pose.position.y:.1f}, "
                f"{node.current_pose.pose.position.z:.1f})"
            )

            time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()