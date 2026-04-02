import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="tcp://host.docker.internal:14551",
        description="URL to the MAVLink stream",
    )


    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mavros"),
                "launch",
                "apm.launch",
            )
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "tgt_system": "1",
            "tgt_component": "1",
            "log_output": "screen",
        }.items(),
    )


    drone_control_node = Node(
        package="drone_control",
        executable="drone_control",
        name="drone_control",
        output="screen",
        emulate_tty=True,
    )


    bridge_detection_node = Node(
        package="drone_control",
        executable="bridge_detection",
        name="bridge_detection",
        output="screen",
        emulate_tty=True,
    )


    return LaunchDescription([
        fcu_url_arg,
        mavros_launch,
        drone_control_node,
        bridge_detection_node,
    ])
