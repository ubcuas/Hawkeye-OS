import asyncio
import json
import os
import base64
import websockets
from websockets.exceptions import ConnectionClosed
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole
from av import VideoFrame
import numpy as np
import fractions

"""
Orchestrator with WebRTC streaming and WebSocket commands

Two channels:
1. WebSocket: Commands (start/stop stream, single capture) + single images
2. WebRTC: Continuous live video stream with object detection

Flows:
    Single Image Capture Command: 
        1. GCOM sends request via WebSocket --> orchestrator receives command 
        2. Orchestrator publishes to image_capture topic
        3. Image captured and processed by object detection
        4. Object detection results received by orchestrator
        5. Orchestrator sends results back to GCOM via WebSocket

    Streaming: 
        1. GCOM sends request via WebSocket --> orchestrator receives 
        2. Same flow 2-4 
        3. Orchestrator sends to GCOM via WebRTC stream s
"""

class ImageStreamTrack(VideoStreamTrack):
    """
    Custom video track that streams images from ROS
    """
    def __init__(self):
        super().__init__()
        self.frame_queue = asyncio.Queue(maxsize=5)  # Buffer for frames
        self.frame_count = 0
        
    async def recv(self):
        """Get next frame for WebRTC"""
        try:
            # Get ROS image from queue
            ros_image = await asyncio.wait_for(self.frame_queue.get(), timeout=1.0)
            
            # Convert ROS Image to numpy array
            if ros_image.encoding == 'rgb8':
                image_array = np.frombuffer(ros_image.data, dtype=np.uint8)
                image_array = image_array.reshape((ros_image.height, ros_image.width, 3))
            else:
                # TODO: add more encodings if needed
                raise ValueError(f"Unsupported encoding: {ros_image.encoding}")
            
            # Create VideoFrame for WebRTC
            frame = VideoFrame.from_ndarray(image_array, format='rgb24')
            frame.pts = self.frame_count
            frame.time_base = fractions.Fraction(1, 30)  # 30 FPS
            self.frame_count += 1
            
            return frame

            """
            After return: 
            1. aiortc library takes the frame
            2. Encodes it to H.264
            3. Packetizes into RTP packets
            4. Sends over UDP to GCOM
            """
            
        except asyncio.TimeoutError:
            # No frame available, send black frame
            black_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame = VideoFrame.from_ndarray(black_frame, format='rgb24')
            frame.pts = self.frame_count
            frame.time_base = fractions.Fraction(1, 30)
            self.frame_count += 1
            return frame


class Orchestrator(Node):
    def __init__(self, 
                 object_detection_topic=None,
                 image_request_topic=None, 
                 gcom_websocket_url=None,
                 webrtc_signaling_url=None): 
        
        super().__init__("orchestrator")
        
        # Load from environment variables 
        self.object_detection_topic = object_detection_topic or os.getenv('OBJECT_DETECTION_TOPIC')
        self.image_request_topic = image_request_topic or os.getenv('IMAGE_REQUEST_TOPIC')
        self.gcom_websocket_url = gcom_websocket_url or os.getenv('GCOM_WEBSOCKET_URL')
        self.webrtc_signaling_url = webrtc_signaling_url or os.getenv('WEBRTC_SIGNALING_URL')

        # Async queues
        self.image_queue = asyncio.Queue()  # Images from object detection (object detection -> orchestrator)
        self.request_queue = asyncio.Queue()  # Image requests (orchestrator -> image capture)
        self.gcom_outgoing_queue = asyncio.Queue()  # Messages to GCOM

        # WebRTC
        self.pc = None  # RTCPeerConnection
        self.video_track = None
        self.streaming = False  # WebRTC streaming active
        self.send_single_images = False  # WebSocket single image mode

        self.websocket = None
        self.signaling_ws = None

        # ROS publishers and subscribers
        self.image_request_pub = self.create_publisher(String, self.image_request_topic, 10)
        self.object_detection_sub = self.create_subscription(Image, self.object_detection_topic, self.image_callback, 10)

        self.get_logger().info('Orchestrator node started')
        self.get_logger().info(f'Subscribing to: {self.object_detection_topic}')
        self.get_logger().info(f'Publishing to: {self.image_request_topic}')
        self.get_logger().info(f'WebSocket: {self.gcom_websocket_url}')
        self.get_logger().info(f'WebRTC Signaling: {self.webrtc_signaling_url}')

    def image_callback(self, msg):
        """Callback when receiving images from object detection"""
        asyncio.run_coroutine_threadsafe(
            self.image_queue.put(msg),
            self.loop
        )

    async def websocket_handler(self):
        """Manage WebSocket connection for commands and single images"""
        while rclpy.ok():
            try:
                async with websockets.connect(self.gcom_websocket_url) as websocket:
                    self.websocket = websocket
                    self.get_logger().info('Connected to GCOM WebSocket')
                    
                    await asyncio.gather(
                        self.websocket_receive(websocket),
                        self.websocket_send(websocket)
                    )
                    
            except ConnectionClosed:
                self.get_logger().warn('GCOM WebSocket closed, reconnecting in 5s...')
                self.websocket = None
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}, reconnecting in 5s...')
                self.websocket = None
                await asyncio.sleep(5)

    async def websocket_receive(self, websocket):
        """Receive commands from GCOM, send to handle_gcom_command"""
        async for message in websocket:
            try:
                data = json.loads(message)
                self.get_logger().info(f'Received command: {data}')
                await self.handle_gcom_command(data)
            except json.JSONDecodeError:
                self.get_logger().error(f'Invalid JSON: {message}')

    async def websocket_send(self, websocket):
        """Send single images and status to GCOM"""
        while rclpy.ok():
            msg = await self.gcom_outgoing_queue.get()
            try:
                await websocket.send(json.dumps(msg))
                msg_type = msg.get('type', 'unknown')
                if msg_type == 'image':
                    self.get_logger().info('Sent single image via WebSocket')
                else:
                    self.get_logger().info(f'Sent: {msg_type}')
            except Exception as e:
                self.get_logger().error(f'Failed to send: {e}')

    async def webrtc_signaling_handler(self):
        """Handle WebRTC signaling for stream setup"""
        while rclpy.ok():
            try:
                async with websockets.connect(self.webrtc_signaling_url) as ws:
                    self.signaling_ws = ws
                    self.get_logger().info('Connected to WebRTC signaling server')
                    
                    async for message in ws:
                        try:
                            data = json.loads(message)
                            await self.handle_signaling(data)
                        except json.JSONDecodeError:
                            self.get_logger().error(f'Invalid signaling JSON: {message}')
                            
            except ConnectionClosed:
                self.get_logger().warn('Signaling connection closed, reconnecting in 5s...')
                self.signaling_ws = None
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f'Signaling error: {e}, reconnecting in 5s...')
                self.signaling_ws = None
                await asyncio.sleep(5)

    async def handle_signaling(self, data):
        """Handle WebRTC signaling messages (SDP offer/answer)"""
        msg_type = data.get('type') 
        
        if msg_type == 'offer':
            self.get_logger().info('Received WebRTC offer')
            
            # Create peer connection
            self.pc = RTCPeerConnection()
            
            # Create video track
            self.video_track = ImageStreamTrack()
            self.pc.addTrack(self.video_track)
            
            # Set remote description (offer)
            await self.pc.setRemoteDescription(
                RTCSessionDescription(sdp=data['sdp'], type=data['type'])
            )
            
            # Create answer
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            
            # Send answer back
            if self.signaling_ws:
                await self.signaling_ws.send(json.dumps({
                    'type': 'answer',
                    'sdp': self.pc.localDescription.sdp
                }))
                self.get_logger().info('Sent WebRTC answer')
                
        elif msg_type == 'ice':
            """ 
            TODO: handle ICE later when actually accessing endpoint (not localhost) 
            
            https://developer.mozilla.org/en-US/docs/Web/API/WebRTC_API/Protocols - ICE needed when p2p connection not immediately possible
            """
            pass

    async def handle_gcom_command(self, data):
        """
        Handle commands from GCOM
        
        Commands:
        - start_stream: Start WebRTC video streaming
        - stop_stream: Stop WebRTC streaming
        - capture_image: Capture single image (via WebSocket)
        """
        command = data.get('command')
        
        if command == 'start_stream':
            self.streaming = True
            self.get_logger().info('Started WebRTC streaming mode')
            
            await self.gcom_outgoing_queue.put({
                'type': 'status',
                'status': 'success',
                'message': 'WebRTC streaming started'
            }) 
            
            # Request continuous images
            msg = String()
            msg.data = 'start'
            await self.request_queue.put(msg)
            
        elif command == 'stop_stream':
            self.streaming = False
            self.get_logger().info('Stopped WebRTC streaming')
            
            await self.gcom_outgoing_queue.put({
                'type': 'status',
                'status': 'success',
                'message': 'Streaming stopped'
            })
            
        elif command == 'capture_image':
            self.send_single_images = True
            self.get_logger().info('Single image capture requested')
            
            # Request single image
            msg = String()
            msg.data = 'capture'
            await self.request_queue.put(msg)
                
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            await self.gcom_outgoing_queue.put({
                'type': 'status',
                'status': 'error',
                'message': f'Unknown command: {command}'
            })

    async def process_images(self):
        """Route images to WebRTC stream or WebSocket based on mode"""
        while rclpy.ok():
            img_msg = await self.image_queue.get()
            
            # Send to WebRTC stream
            if self.streaming and self.video_track:
                try:
                    # Add to video track queue (non-blocking)
                    if self.video_track.frame_queue.full():
                        # Drop oldest frame if queue full
                        try:
                            self.video_track.frame_queue.get_nowait()
                        except asyncio.QueueEmpty:
                            pass
                    await self.video_track.frame_queue.put(img_msg)
                    self.get_logger().debug(f'Sent frame to WebRTC stream')
                except Exception as e:
                    self.get_logger().error(f'Error sending to WebRTC: {e}')
            
            # Send single image via WebSocket
            if self.send_single_images:
                self.send_single_images = False  # One-shot
                image_base64 = base64.b64encode(bytes(img_msg.data)).decode('utf-8')
                
                await self.gcom_outgoing_queue.put({
                    'type': 'image',
                    'encoding': img_msg.encoding,
                    'width': img_msg.width,
                    'height': img_msg.height,
                    'data': image_base64,
                    'timestamp': self.get_clock().now().to_msg().sec
                })
                self.get_logger().info(f'Queued single image for WebSocket')

    async def send_requests(self):
        """Send image requests to object detection"""
        while rclpy.ok():
            msg = await self.request_queue.get()
            self.get_logger().info(f'Requesting images: {msg.data}')
            self.image_request_pub.publish(msg)


async def async_main(args=None):
    """Main async entry point"""
    rclpy.init(args=args)

    orchestrator = Orchestrator()
    orchestrator.loop = asyncio.get_running_loop()

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
            orchestrator.send_requests(),
            orchestrator.websocket_handler(),
            orchestrator.webrtc_signaling_handler()
        )
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup WebRTC
        if orchestrator.pc:
            await orchestrator.pc.close()
        orchestrator.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point wrapper"""
    asyncio.run(async_main(args))


if __name__ == '__main__':
    main()