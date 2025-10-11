import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 

class Orchestrator(Node):
    def __init__(self): 
        # Create a Parent Node with the name "orchestrator"
        super().__init__("orchestrator") 


        self.incoming_queue = asyncio.Queue()
        self.outgoing_queue = asyncio.Queue()

        # TODO: create sub and pub properly
        self.objectDetect_pub = self.create_publisher(None); 
        self.imageRequest_sub = self.create_subscription(None); 

    # Callback: called by executor whenever a message is received
    def ros_callback(self, msg):
        #
        asyncio.run_coroutine_threadsafe(
            self.incoming_queue.put(msg),
            self.loop
        )

    # Handler: process received messages 
    async def process_messages(self):
        while rclpy.ok():
            msg = await self.incoming_queue.get()
            # TODO: implement request handler (depends on how object detection is set up )
            await self.handle_request(msg)
    
    # Publisher
    async def send_messages(self):
        while rclpy.ok():
            msg = await self.outgoing_queue.get()
            # TODO: implement message publisher (depends on how Image Request is set up )
            self.objectDetect_pub.publish(msg)

async def main():
    # Initialize ROS2
    rclpy.init()

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


    await asyncio.gather(
        spin(),
        orchestrator.process_messages(),
        orchestrator.send_messages()
    )

if __name__ == '__main__':
    asyncio.run(main())



