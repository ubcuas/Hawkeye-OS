import asyncio
import json
import os
import websockets
from websockets.exceptions import ConnectionClosed
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 
from std_msgs.msg import String

"""
TODO: Create enums for message datas later

Flow:
1. GCOM sends request via WebSocket --> orchestrator receives
2. Orchestrator publishes to image_capture topic
3. Image captured and processed by object detection
4. Object detection results received by orchestrator
5. Orchestrator sends results back to GCOM via WebSocket
"""

class Orchestrator(Node):
    def __init__(self, 
                 input_topic=None,
                 output_topic=None, 
                 input_msg_type=String,
                 output_msg_type=String,
                 gcom_websocket_url=None): 
        
        super().__init__("orchestrator")
        
        # Load env variables
        self.input_topic = input_topic or os.getenv('OBJECT_DETECTION_TOPIC', 'object_detection')
        self.output_topic = output_topic or os.getenv('IMAGE_CAPTURE_TOPIC', 'image_capture')
        self.gcom_websocket_url = gcom_websocket_url or os.getenv('GCOM_WEBSOCKET_URL', 'ws://localhost:8765') 

        # Async queues for message handling
        self.incoming_queue = asyncio.Queue()  # From object detection (ROS)
        self.outgoing_queue = asyncio.Queue()  # To image capture (ROS)
        self.gcom_outgoing_queue = asyncio.Queue()  # To GCOM (WebSocket)

        self.input_msg_type = input_msg_type
        self.output_msg_type = output_msg_type
        self.websocket = None
        self.send_to_gcom = False  # Flag to send data to GCOM or not 

        # ROS publishers and subscribers
        self.image_request_pub = self.create_publisher(output_msg_type, self.output_topic, 10) 
        self.object_detect_sub = self.create_subscription(input_msg_type, self.input_topic, self.ros_callback, 10)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Listening on: {self.input_topic} (Object Detection)')
        self.get_logger().info(f'Publishing on: {self.output_topic} (Image Capture)')
        self.get_logger().info(f'GCOM WebSocket URL: {self.gcom_websocket_url}')


    def ros_callback(self, msg):
        """Callback when receiving messages from object detection"""
        self.get_logger().info(f'Received from object detection: {msg.data if hasattr(msg, "data") else str(msg)}')
        
        asyncio.run_coroutine_threadsafe(
            self.incoming_queue.put(msg),
            self.loop
        )


    async def websocket_handler(self):
        """Manage WebSocket connection to GCOM with auto-reconnect"""
        while rclpy.ok():
            try:
                async with websockets.connect(self.gcom_websocket_url) as websocket:
                    self.websocket = websocket
                    self.get_logger().info('Connected to GCOM')
                    
                    # Run send and receive concurrently
                    await asyncio.gather(
                        self.websocket_receive(websocket),
                        self.websocket_send(websocket)
                    )
                    
            except ConnectionClosed:
                self.get_logger().warn('GCOM connection closed, reconnecting in 5s...')
                self.websocket = None
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}, reconnecting in 5s...')
                self.websocket = None
                await asyncio.sleep(5)

    async def websocket_receive(self, websocket):
        """Receive and process messages from GCOM in JSON"""
        async for message in websocket:
            try:
                data = json.loads(message)
                self.get_logger().info(f'Received from GCOM: {data}')
                await self.handle_gcom_request(data)
            except json.JSONDecodeError:
                self.get_logger().error(f'Invalid JSON from GCOM: {message}')

    async def websocket_send(self, websocket):
        """Send messages to GCOM"""
        while rclpy.ok():
            msg = await self.gcom_outgoing_queue.get()
            try:
                await websocket.send(json.dumps(msg))
                self.get_logger().info(f'Sent to GCOM: {msg}')
            except Exception as e:
                self.get_logger().error(f'Failed to send to GCOM: {e}')

    async def handle_gcom_request(self, data):
        """
        Process incoming requests from GCOM
        
        Expected format:
        {
            "command": "start_capture" | "stop_capture" | "capture_image",
            "parameters": {...}
        }
        """
        command = data.get('command')
        
        match(command): 
            case 'start_capture': 
                self.send_to_gcom = True 
                self.get_logger().info('Started sending images to GCOM')
                await self.gcom_outgoing_queue.put({
                    'status': 'success',
                    'message': 'Started sending images to GCOM'
                })
            
            case 'stop_capture':
                self.send_to_gcom = False
                self.get_logger().info('Stopped sending images to GCOM')
                await self.gcom_outgoing_queue.put({
                    'status': 'success',
                    'message': 'Stopped sending images to GCOM'
                })
            
            case 'capture_image':
                self.get_logger().info('Single image capture requested')
                # Send image capture request to ROS
                if self.output_msg_type == String:
                    msg = String()
                    msg.data = 'Single capture request from GCOM'
                    await self.outgoing_queue.put(msg)
                
        else:
            self.get_logger().warn(f'Unknown command from GCOM: {command}')
            await self.gcom_outgoing_queue.put({
                'status': 'error',
                'message': f'Unknown command: {command}'
            })

    async def process_messages(self):
        """Process messages from object detection"""
        while rclpy.ok():
            msg = await self.incoming_queue.get()
            await self.handle_object_detection(msg)

    async def handle_object_detection(self, msg):
        """
        Process object detection results and send to GCOM if enabled
        """
        self.get_logger().info('Processing object detection result')
        
        # TODO: edit once object detection's message types and fields known, now just extracts "data" field
        data_str = msg.data if hasattr(msg, 'data') else str(msg)
        
        # Flag enabled --> send to GCOM 
        if self.send_to_gcom:
            await self.gcom_outgoing_queue.put({
                'type': 'object_detection',
                'data': data_str,
                'timestamp': self.get_clock().now().to_msg().sec
            })
            self.get_logger().info(f'Sent to GCOM: {data_str}')
        else:
            self.get_logger().debug(f'GCOM sending disabled, not forwarding: {data_str}')

    async def send_messages(self):
        """Send messages to image capture topic"""
        while rclpy.ok():
            msg = await self.outgoing_queue.get()
            self.get_logger().info('Publishing image request')
            self.image_request_pub.publish(msg)


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    # Create orchestrator (all config from environment variables)
    orchestrator = Orchestrator()
    orchestrator.loop = asyncio.get_running_loop()

    # Create ROS executor
    executor = SingleThreadedExecutor()
    executor.add_node(orchestrator)

    # ROS spin function
    async def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            orchestrator.process_messages(),      # Process object detection
            orchestrator.send_messages(),         # Send to image capture
            orchestrator.websocket_handler()      # Handle WebSocket connection
        )
    except KeyboardInterrupt:
        pass

    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == '__main__':
    main()