import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 
from std_msgs.msg import String

# TODO: maybe create a message class/enum for objectDetect/imageRequest format, for now it is String
"""
    GCOM sends request to take image --> orchestrator receives the request from a public API{?} --> orchestrator sends request to image request queue to tell it to start capturing image --> image goes to object detection --> object detection data --> orchestrator --> forwarded back to GCOM

    Check: best way to send images over LTE (to GCOM), GCOM needs bidirectional communication

    TODO NOW: implement the startProcess() and endProcess() function (later when GCOM wants the orchestrator to start actually sending them images) {through an API?} 

    Note: Use Websocket for Orchestrator to receive messages from GCOM, GCOM already has web endpoint 

"""

class Orchestrator(Node):
    def __init__(self, 
                 input_topic='object_detection',
                 output_topic='image_capture', 
                 input_msg_type=String,
                 output_msg_type=String): 
        
        # Create a Parent Node with the name "orchestrator"
        super().__init__("orchestrator") 

        self.incoming_queue = asyncio.Queue()
        self.outgoing_queue = asyncio.Queue()

        # Store message types for other functions
        self.input_msg_type = input_msg_type
        self.output_msg_type = output_msg_type

        # create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)
        # create_subscription(msg_type, topic, callback, qos_profile, *, callback_group=None, event_callbacks=None, raw=False)
        """
            Note: the publisher node is the node that sends messages out of the orchestrator (to the Image Request queue) and vice versa
        """
        self.image_request_pub = self.create_publisher(output_msg_type, output_topic, 10) 

        self.object_detect_sub = self.create_subscription(input_msg_type, input_topic, self.ros_callback, 10)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Listening on: {input_topic} (Object Detection Queue)')
        self.get_logger().info(f'Publishing on: {output_topic} (Image Request Queue)')


    def ros_callback(self, msg):
        # Callback: called by executor whenever a message is received
        self.get_logger().info(f'Received message: {msg.data if hasattr(msg, "data") else str(msg)}')

        asyncio.run_coroutine_threadsafe(
            self.incoming_queue.put(msg),
            self.loop
        )
 

    async def process_messages(self):
        """
            Process messages from queue
        """
        while rclpy.ok():
            msg = await self.incoming_queue.get()
            # TODO: implement request handler (depends on how object detection is set up )
            await self.handle_request(msg)


    async def handle_request(self, msg):
        """
            Process incoming object detection results and generate image requests
            Default: echo back for testing with String messages
        """
        self.get_logger().info(f'Handling object detection result')
        
        # TODO: replace with actual handling logic later
        if self.output_msg_type == String:
            response = String()
            response.data = f'Image request for: {msg.data}'
            await self.outgoing_queue.put(response)
      

    async def send_messages(self):
        """
            Send messages to the queue
        """
        while rclpy.ok():
            msg = await self.outgoing_queue.get()
            self.get_logger().info(f'Publishing image request')
            self.image_request_pub.publish(msg)


async def async_main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create orchestrator node 
    orchestrator = Orchestrator()
    orchestrator.loop = asyncio.get_running_loop()

    # Create ROS callback manager 
    executor = SingleThreadedExecutor()
    executor.add_node(orchestrator)

    # Spin to keep orchestrator node alive 
    async def spin():
        while rclpy.ok():
            # Process one ROS callback, then yield to other task 
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            orchestrator.process_messages(),
            orchestrator.send_messages()
        )
    except KeyboardInterrupt:
        pass

    # Cleanup and shutdown 
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))

if __name__ == '__main__':
    asyncio.run(main())
    