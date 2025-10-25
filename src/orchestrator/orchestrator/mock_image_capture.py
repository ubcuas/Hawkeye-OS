import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockImageCapture(Node):
    def __init__(self):
        super().__init__('mock_image_capture')
        self.subscription = self.create_subscription(
            String,
            'image_capture',
            self.listener_callback,
            10)
        self.get_logger().info('Mock Image Capture node started')
        self.get_logger().info('Listening for image requests...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image request: "{msg.data}"')
        self.get_logger().info('Simulating image capture...')


def main(args=None):
    rclpy.init(args=args)
    node = MockImageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()