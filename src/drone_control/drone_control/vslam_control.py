import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn

class VslamWatchdog(Node):
    def __init__(self):
        super().__init__('vslam_watchdog')
        
        self.max_covariance_xyz = 0.1       # Max position variance
        self.max_covariance_yaw = 0.1       # Max heading variance (index 35)
        self.max_velocity = 8.0             
        self.max_stale_time = 0.5           # Max seconds without odometry
        
        self.last_pose = None
        self.__init___time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.is_vslam_healthy = True
        self.bad_velocity_frames = 0        # Debounce counter
        
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.vision_pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.vslam_sub = self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, 10)
        
        # Stale data watchdog timer (runs at 20Hz)
        self.timer = self.create_timer(0.05, self.check_stale_data)

    def check_stale_data(self):
        if self.is_vslam_healthy:
            time_since_last_msg = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
            if time_since_last_msg > self.max_stale_time and (self.get_clock().now() - self.__init___time).nanoseconds * 1e-9 > 5.0: # Avoid triggering on startup or long-term failure
                self.trigger_failsafe(f"Stale VSLAM data (>{self.max_stale_time}s)!")

    def trigger_failsafe(self, reason):
        if self.is_vslam_healthy:
            self.get_logger().error(f"VSLAM Failsafe Triggered: {reason}")
            self.is_vslam_healthy = False
            
            # Switch to a safe Non-GPS mode (Ensure FC configuration matches this RC command!)
            # rc_msg = OverrideRCIn()
            # channels = [65535] * 18
            # channels[8] = 1000  
            # rc_msg.channels = channels
            # self.rc_override_pub.publish(rc_msg)

    def odom_cb(self, msg: Odometry):
        self.get_logger().debug("Received VSLAM Odometry message.")
        self.get_logger().info(f"Position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}, z={msg.pose.pose.position.z:.3f}")
        self.last_time = self.get_clock().now()

        # 1. Position and Yaw Covariance Checks
        var_x, var_y, var_z = msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]
        var_yaw = msg.pose.covariance[35]
        
        if max(var_x, var_y, var_z) > self.max_covariance_xyz:
            self.trigger_failsafe(f"Position Variance too high: {max(var_x, var_y, var_z):.3f}")
            return
            
        if var_yaw > self.max_covariance_yaw:
            self.trigger_failsafe(f"Heading (Yaw) Variance too high: {var_yaw:.3f}")
            return
            
        # 2. Debounced Velocity/Sanity Check
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_pose = msg.pose.pose.position
        
        if self.last_pose is not None:
            # Note: storing time locally for velocity might be cleaner to avoid ROS clock mismatches
            dt = 1.0 / 90.0 # Assuming ~90hz if dt drops to 0, or calculate properly robustly
            # Safest velocity dt:
            dt = (self.last_time - self.get_clock().now()).nanoseconds * 1e-9 # placeholder concept
            # (In practice, use node time history calculated carefully)
            
            dx = current_pose.x - self.last_pose.x
            dy = current_pose.y - self.last_pose.y
            dz = current_pose.z - self.last_pose.z
            
            # Distance squared (skip sqrt for efficiency if desired)
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # If the distance between two sequential frames is massive, track it
            if distance > (self.max_velocity * 0.0111): # Assuming hardware runs at 90fps = 0.0111s delta
                self.bad_velocity_frames += 1
                if self.bad_velocity_frames > 3: # Debounce length (3 frames)
                    self.trigger_failsafe(f"Impossible trajectory jump detected.")
                    return
            else:
                self.bad_velocity_frames = max(0, self.bad_velocity_frames - 1)
        
        self.last_pose = current_pose
        
        # 3. Publish clean pose
        if self.is_vslam_healthy:
            out_msg = PoseStamped()
            out_msg.header.frame_id = msg.header.frame_id
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.pose = msg.pose.pose
            self.vision_pose_pub.publish(out_msg)

def main():
    rclpy.init()
    node = VslamWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()