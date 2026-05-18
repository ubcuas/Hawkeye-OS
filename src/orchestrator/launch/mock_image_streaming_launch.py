from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch mock object detection and streaming nodes together."""

    mock_object_detection_node = Node(
        package="orchestrator",
        executable="mock_object_detection",
        name="mock_object_detection",
        output="screen",
        parameters=[{"object_detection_topic": "color/image_raw/compressed"}],
    )

    streaming_node = Node(
        package="streaming",
        executable="streaming",
        name="streaming",
        output="screen",
        parameters=[{"object_detection_topic": "color/image_raw/compressed"}],
    )

    mock_tagged_image_node = Node(
        package="orchestrator",
        executable="mock_tagged_image",
        name="mock_tagged_image",
        output="screen",
    )

    return LaunchDescription(
        [
            mock_object_detection_node,
            mock_tagged_image_node,
            streaming_node,
        ]
    )
