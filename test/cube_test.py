import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, Imu

class CubeTest(Node):
    def __init__(self):
        super().__init__('cube_test')
        
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, 10)

    def state_cb(self, msg):
        self.get_logger().info(f'Connected: {msg.connected} | Armed: {msg.armed} | Mode: {msg.mode}')

    def gps_cb(self, msg):
        self.get_logger().info(f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f}')

    def imu_cb(self, msg):
        self.get_logger().info(f'IMU orientation x: {msg.orientation.x:.3f}')

def main():
    rclpy.init()
    node = CubeTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()