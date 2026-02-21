import asyncio
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 
from std_msgs.msg import String
from sensor_msgs.msg import Image
import paho.mqtt.client as mqtt

# MQTT Configuration (GCOM Connection)
MQTT_BROKER = "broker.hivemq.com" # Replace with your VPS IP for production
MQTT_TOPIC_CMD = "ubc_uas/drone_01/commands"
MQTT_TOPIC_STATUS = "ubc_uas/drone_01/status"
MQTT_TOPIC_CMD_ACK = "ubc_uas/drone_01/command_ack"

"""
Orchestrator node

Coordinates image capture requests and object detection results.
"""


class Orchestrator(Node):
    def __init__(self,
                loop,
                object_detection_topic=None,
                image_request_topic=None):
        
        super().__init__("orchestrator") 

        # Load from environment variables
        self.object_detection_topic = object_detection_topic or os.getenv('OBJECT_DETECTION_TOPIC')
        self.image_request_topic = image_request_topic or os.getenv('IMAGE_REQUEST_TOPIC')

        self.loop = loop

        self.image_queue = asyncio.Queue()
        self.outgoing_queue = asyncio.Queue()

        # ROS publishers and subscribers
        self.image_request_pub = self.create_publisher(String, self.image_request_topic, 10)
        self.object_detection_sub = self.create_subscription(Image, self.object_detection_topic, self.image_callback, 10)

        # --- MQTT Communication (GCOM Link) ---
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "Drone_Orchestrator")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(MQTT_BROKER, 1883, 60)
            self.mqtt_client.loop_start() # Runs network in a background thread (non-blocking)
            self.get_logger().info(f'Connected to MQTT Broker: {MQTT_BROKER}')
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        # --- Status Heartbeat ---
        # Send status to GCOM every 1.0 second
        self.status_timer = self.create_timer(1.0, self.publish_status_to_ground)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Subscribing to: {self.object_detection_topic}')
        self.get_logger().info(f'Publishing to: {self.image_request_topic}')

    # --- MQTT Callbacks ---
    def on_mqtt_connect(self, client, userdata, flags, rc, properties):
        """Called when connected to the broker."""
        if rc == 0: # success:
            client.subscribe(MQTT_TOPIC_CMD)
            self.get_logger().info(f"Subscribed to GCOM commands: {MQTT_TOPIC_CMD}")
        else:
            self.get_logger().error(f"MQTT connect failed with rc : {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """
        Triggered when GCOM sends a command.
        Note: This runs in the MQTT thread, so we use `run_coroutine_threadsafe`
        to pass data safely into the main asyncio loop.
        """
        try:
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f"MQTT PAYLOAD: {payload}")
            
            command = payload.get('action')
            
            self.get_logger().info(f"Received GCOM Command: {command}")

            # Example: GCOM says "TAKE_PHOTO" -> We push it to the queue for processing
            if command:
                asyncio.run_coroutine_threadsafe(
                    self.image_queue.put(payload), 
                    self.loop
                )

        except Exception as e:
            self.get_logger().error(f"Failed to process MQTT message: {e}")

    def publish_status_to_ground(self):
        """
        Sends heartbeat/telemetry to GCOM.
        """

        status = {
            "status": "ONLINE",
            "queue_size": self.image_queue.qsize()
            # Add battery or GPS info here later
        }
        self.mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps(status))

    # --- Existing ROS/Async Logic ---
    def image_callback(self, msg):
        """Callback when receiving images from object detection"""
        asyncio.run_coroutine_threadsafe(
            self.image_queue.put(msg),
            self.loop
        )
        self.get_logger().info('IMAGE CALLBACK: Added to queue')


    def ros_callback(self, msg):
        self.get_logger().info(f'Received ROS message: {msg.data if hasattr(msg, "data") else str(msg)}')
        asyncio.run_coroutine_threadsafe(
            self.image_queue.put(msg),
            self.loop
        )
 
    async def process_images(self):
        """Process images from object detection"""
        while rclpy.ok():
            self.get_logger().info('PROCESS: Waiting for image from queue...')
            img_msg = await self.image_queue.get()
            self.get_logger().info('PROCESS: Got image')
            # TODO: Add image processing logic here

    async def handle_request(self, msg):
        """Logic Router"""
        self.get_logger().info(f'Handling request: {msg}')
        
        # CASE 1: Message from ROS (Object Detection)
        if hasattr(msg, 'data') or isinstance(msg, String):
             response = String()
             response.data = f'Image request for target: {msg.data}'
             await self.outgoing_queue.put(response)
        
        # CASE 2: Message from GCOM (MQTT Dictionary)
        elif isinstance(msg, dict):
            action = msg.get('action')
            if action == "TAKE_PHOTO":
                self.get_logger().info("Executing GCOM Photo Request...")
                self.publish_ack(action, status="COMMAND_RECEIVED")
                # Logic to trigger camera goes here
                pass

    def publish_ack(self, action, status, detail=None):
        ack = {
            "action": action,
            "status": status,
            "detail": detail
        }
        print("Publishing ACK to GCOM:", ack)
        self.mqtt_client.publish(MQTT_TOPIC_CMD_ACK, json.dumps(ack))

    async def send_messages(self):
        """Producer Loop"""
        while rclpy.ok():
            msg = await self.outgoing_queue.get()
            self.get_logger().info(f'Publishing image request')
            self.image_request_pub.publish(msg)

async def async_main(args=None):
    rclpy.init(args=args)

    loop = asyncio.get_running_loop()
    orchestrator = Orchestrator(loop=loop)

    executor = SingleThreadedExecutor()
    executor.add_node(orchestrator)

    async def spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        await asyncio.gather(
            spin(),
            orchestrator.process_images(),
            orchestrator.send_messages()
        )
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.mqtt_client.loop_stop() # Stop the background MQTT thread
        orchestrator.mqtt_client.disconnect()
        orchestrator.destroy_node()
        rclpy.shutdown()

def main(args=None):
    asyncio.run(async_main(args))

if __name__ == '__main__':
    main()
