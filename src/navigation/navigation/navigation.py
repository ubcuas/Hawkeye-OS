import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import threading
import time

class ArduPilotNode(Node):
    def __init__(self):
        super().__init__('ardupilot_node')

        # --- Variables to hold current target ---
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0 # Default hover height
        
        # 1. Position Publisher (Sends setpoints to MAVROS)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # 2. State Subscriber
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        # 3. NEW: Command Subscriber (Listens for your commands)
        # Publish to this topic to move the drone!
        self.cmd_sub = self.create_subscription(
            PoseStamped, 
            '/drone/cmd_pose', 
            self.command_cb, 
            10
        )
        
        # 4. Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # 5. Timer (Heartbeat)
        # MAVROS needs a steady stream of setpoints. 
        # This runs at 20Hz to publish the current target continuously.
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.current_state = State()
        self.get_logger().info("Node initialized. Listening on /drone/cmd_pose")

    def state_cb(self, msg):
        self.current_state = msg

    def command_cb(self, msg):
        """
        Callback when a new command is received on /drone/cmd_pose
        """
        self.get_logger().info(f"Received target: X:{msg.pose.position.x:.2f} Y:{msg.pose.position.y:.2f} Z:{msg.pose.position.z:.2f}")
        # Update the internal target variable
        self.target_pose = msg

    def timer_cb(self):
        """
        Loop that runs at 20Hz. 
        It republishes the last known target pose to keep the drone happy.
        """
        # Update timestamp to current time (required by MAVROS)
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = "map" # or "odom"
        
        # Publish to MAVROS
        self.local_pos_pub.publish(self.target_pose)

def main():
    rclpy.init()
    node = ArduPilotNode()

    # Spin in a background thread so callbacks (timer and subs) work
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # --- Startup Sequence ---
    
    # 1. Wait for connection
    while not node.current_state.connected:
        node.get_logger().info("Waiting for connection...")
        time.sleep(1)

    # 2. Set to GUIDED Mode
    node.get_logger().info("Setting GUIDED mode...")
    mode_cmd = SetMode.Request()
    mode_cmd.custom_mode = 'GUIDED'
    while not node.set_mode_client.call(mode_cmd).mode_sent:
        time.sleep(1)

    # 3. Arm the drone
    node.get_logger().info("Arming...")
    arm_cmd = CommandBool.Request()
    arm_cmd.value = True
    while not node.arming_client.call(arm_cmd).success:
        node.get_logger().warn("Arming failed (check GPS fix?), retrying...")
        time.sleep(2)

    # 4. Takeoff
    node.get_logger().info("Taking off...")
    takeoff_cmd = CommandTOL.Request()
    takeoff_cmd.altitude = 2.0 
    node.takeoff_client.call(takeoff_cmd)
    
    node.get_logger().info("Drone is airborne and listening for commands on /drone/cmd_pose...")

    # Keep the main thread alive. 
    # The actual movement is