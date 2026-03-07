"""
Global test configuration.

Mocks out ROS2 and all heavy external dependencies so source modules
can be imported and tested without a live ROS2 runtime.
"""
import os
import sys
from unittest.mock import MagicMock

# Ensure streaming.constants doesn't raise at import time in test runs.
os.environ.setdefault('WEBRTC_SIGNALING_URL', 'ws://test.local')

# --- ROS2 runtime mocks ---
rclpy_mock = MagicMock()
sys.modules['rclpy'] = rclpy_mock
sys.modules['rclpy.executors'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.impl'] = MagicMock()
sys.modules['rclpy.impl.rcutils_logger'] = MagicMock()

# rclpy.node.Node must be a real subclassable class.
# All ROS2 nodes inherit from it, and Python's metaclass machinery breaks
# when you subclass a raw MagicMock instance.
class _StubNode:
    """
    Minimal stand-in for rclpy.node.Node.

    Records every create_subscription, create_publisher, and create_timer
    call made during __init__ so tests can assert on the communication
    contract without a live ROS2 runtime.
    """
    def __init__(self, name, **kwargs):
        self._node_name = name
        self._subscriptions = []  # {topic, msg_type, callback, qos}
        self._publishers = []     # {topic, msg_type, qos}
        self._timers = []         # {period, callback}
        self._logger = MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile=10):
        self._subscriptions.append({
            'topic': topic,
            'msg_type': msg_type,
            'callback': callback,
            'qos': qos_profile,
        })
        return MagicMock()

    def create_publisher(self, msg_type, topic, qos_profile=10):
        self._publishers.append({
            'topic': topic,
            'msg_type': msg_type,
            'qos': qos_profile,
        })
        return MagicMock()

    def create_timer(self, period, callback):
        self._timers.append({'period': period, 'callback': callback})
        return MagicMock()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return MagicMock()

    def declare_parameter(self, name, default):
        return MagicMock()

    def get_parameter(self, name):
        return MagicMock()

    def destroy_node(self):
        pass

    # --- Query helpers ---

    def subscription(self, topic):
        """Return the first subscription registered on the given topic, or None."""
        return next((s for s in self._subscriptions if s['topic'] == topic), None)

    def publisher(self, topic):
        """Return the first publisher registered on the given topic, or None."""
        return next((p for p in self._publishers if p['topic'] == topic), None)


_node_module_mock = MagicMock()
_node_module_mock.Node = _StubNode
sys.modules['rclpy.node'] = _node_module_mock

# --- ROS2 message type mocks ---
# std_msgs.msg.String must be a real subclassable class so that
# isinstance(msg, String) works in source code under test.
class _StubString:
    def __init__(self):
        self.data = ''

_std_msgs_msg_mock = MagicMock()
_std_msgs_msg_mock.String = _StubString
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = _std_msgs_msg_mock

sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()

# --- ROS2 support library mocks ---
sys.modules['cv_bridge'] = MagicMock()
sys.modules['message_filters'] = MagicMock()

# --- External service / protocol mocks ---
sys.modules['paho'] = MagicMock()
sys.modules['paho.mqtt'] = MagicMock()
sys.modules['paho.mqtt.client'] = MagicMock()

sys.modules['socketio'] = MagicMock()

sys.modules['aiortc'] = MagicMock()
sys.modules['aiortc.sdp'] = MagicMock()

# VideoStreamTrack must be a real subclassable class, not a MagicMock.
# Python's metaclass machinery breaks when you try to subclass a MagicMock
# instance, causing constructors to return mocks instead of real objects.
class _StubVideoStreamTrack:
    def __init__(self):
        pass

_mediastreams_mock = MagicMock()
_mediastreams_mock.VideoStreamTrack = _StubVideoStreamTrack
sys.modules['aiortc.mediastreams'] = _mediastreams_mock

sys.modules['av'] = MagicMock()

sys.modules['cv2'] = MagicMock()
