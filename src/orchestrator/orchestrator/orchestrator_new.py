import json
import os
import queue
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import Image
from hawkeye_msgs.msg import TaggedImage
# import paho.mqtt.client as mqtt

# # MQTT Configuration (GCOM Connection)
# MQTT_BROKER = "broker.hivemq.com" # Replace with your VPS IP for production
# MQTT_TOPIC_CMD = "ubc_uas/drone_01/commands"
# MQTT_TOPIC_STATUS = "ubc_uas/drone_01/status"
# MQTT_TOPIC_CMD_ACK = "ubc_uas/drone_01/command_ack"

"""
Orchestrator node

Coordinates image capture requests and object detection results.
"""

class Orchestrator(Node):
    def __init__(self,
                object_detection_topic=None,
                image_request_topic=None):
        
        super().__init__("orchestrator") 

        # Load from environment variables
        self.object_detection_topic = object_detection_topic or os.getenv('OBJECT_DETECTION_TOPIC', '/object_detection/images')
        self.image_request_topic = image_request_topic or os.getenv('IMAGE_REQUEST_TOPIC', '/orchestrator/image_request')
        self.image_request_topic = "/orchestrator/image_request"
        
        # Thread-safe queues for communication between ROS, MQTT, and timers
        self.request_queue = queue.Queue()
        
        # Callback groups to allow concurrent execution
        self.cbg_sub = MutuallyExclusiveCallbackGroup()
        self.cbg_timer = MutuallyExclusiveCallbackGroup()

        # Take photo request
        self.image_request_pub = self.create_publisher(String, self.image_request_topic, 10)
        # Subscribe to object detection results
        self.object_detection_sub = self.create_subscription(
            Image, self.object_detection_topic, self.image_callback, 10, callback_group=self.cbg_sub)


        # --- MQTT Communication (GCOM Link) --- DISABLED: replaced by WebRTC data channel
        # self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "Drone_Orchestrator")
        # self.mqtt_client.on_connect = self.on_mqtt_connect
        # self.mqtt_client.on_message = self.on_mqtt_message
        # try:
        #     self.mqtt_client.connect(MQTT_BROKER, 1883, 60)
        #     self.mqtt_client.loop_start()
        #     self.get_logger().info(f'Connected to MQTT Broker: {MQTT_BROKER}')
        # except Exception as e:
        #     self.get_logger().error(f"MQTT Connection Failed: {e}")

        # --- Status Heartbeat --- DISABLED: replaced by WebRTC data channel
        # self.status_timer = self.create_timer(1.0, self.publish_status_to_ground, callback_group=self.cbg_timer)

        # --- Request Processing Timer ---
        # Fast timer to process incoming requests from the queue
        self.process_timer = self.create_timer(0.05, self.process_queue, callback_group=self.cbg_timer)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Subscribing to: {self.object_detection_topic}')
        self.get_logger().info(f'Publishing to: {self.image_request_topic}')

    # --- MQTT Callbacks --- DISABLED: replaced by WebRTC data channel
    # def on_mqtt_connect(self, client, userdata, flags, rc, properties):
    #     if rc == 0:
    #         client.subscribe(MQTT_TOPIC_CMD)
    #     else:
    #         self.get_logger().error(f"MQTT connect failed with rc : {rc}")

    # def on_mqtt_message(self, client, userdata, msg):
    #     try:
    #         payload = json.loads(msg.payload.decode())
    #         command = payload.get('action')
    #         if command:
    #             self.request_queue.put(payload)
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to process MQTT message: {e}")

    # def publish_status_to_ground(self):
    #     status = {"status": "ONLINE", "queue_size": self.request_queue.qsize()}
    #     self.mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps(status))

    # def publish_ack(self, action, status, detail=None):  # DISABLED: replaced by WebRTC data channel
    #     ack = {"action": action, "status": status, "detail": detail}
    #     self.mqtt_client.publish(MQTT_TOPIC_CMD_ACK, json.dumps(ack))

    # --- ROS Callbacks ---
    def image_callback(self, msg):
        """Callback when receiving images from object detection"""
        self.get_logger().info('IMAGE CALLBACK: Added to queue')
        self.request_queue.put(msg)

    # --- Processing Logic ---
    def process_queue(self):
        """Processes pending items from the queue (called via timer)."""
        try:
            # Only process one per tick to avoid blocking the timer, 
            # or use a short while loop if high throughput is needed
            while not self.request_queue.empty():
                msg = self.request_queue.get_nowait()
                self.handle_request(msg)
                self.request_queue.task_done()
        except queue.Empty:
            pass

    def handle_request(self, msg):
        """Logic Router"""
        # CASE 1: Message from ROS (Object Detection)
        if hasattr(msg, 'data') or isinstance(msg, String):
            response = String()
            response.data = f'Image request for target: {msg.data}'
            self.get_logger().info('Publishing image request from ROS message')
            self.image_request_pub.publish(response)
        
        # CASE 2: Message from GCOM (MQTT Dictionary) --- DISABLED: replaced by WebRTC data channel
        # elif isinstance(msg, dict):
        #     action = msg.get('action')
        #     if action == "TAKE_PHOTO":
        #         self.publish_ack(action, status="COMMAND_RECEIVED")
        #         trigger = String()
        #         trigger.data = "TAKE_PHOTO"
        #         self.image_request_pub.publish(trigger)
        #         self.publish_ack(action, status="COMMAND_EXECUTED")

def main(args=None):
    rclpy.init(args=args)

    orchestrator = Orchestrator()
    # MultiThreadedExecutor allows timer callbacks and subscriber callbacks to process concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(orchestrator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # orchestrator.mqtt_client.loop_stop()  # DISABLED: replaced by WebRTC data channel
        # orchestrator.mqtt_client.disconnect()
        orchestrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


