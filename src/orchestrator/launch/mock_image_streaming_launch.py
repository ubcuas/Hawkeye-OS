from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch mock object detection and streaming nodes together."""

    mock_object_detection_node = Node(
        package='orchestrator',
        executable='mock_object_detection',
        name='mock_object_detection',
        output='screen',
        parameters=[
            {'object_detection_topic': 'object_detection/image'}
        ]
    )

    streaming_node = Node(
        package='streaming',
        executable='streaming',
        name='streaming',
        output='screen',
        parameters=[
            {'object_detection_topic': 'object_detection/image'}
        ]
    )

    return LaunchDescription([
        mock_object_detection_node,
        streaming_node,
    ])
