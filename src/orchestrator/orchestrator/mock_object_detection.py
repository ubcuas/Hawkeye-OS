import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockObjectDetection(Node):
    def __init__(self):
        super().__init__('mock_object_detection')
        self.publisher_ = self.create_publisher(String, 'object_detection', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Mock Object Detection node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Detected object #{self.count}: person at (x:{self.count*10}, y:{self.count*20})'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()