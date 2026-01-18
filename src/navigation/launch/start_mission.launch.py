import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# We import AnyLaunchDescriptionSource because apm.launch is an XML file, not Python
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the MAVROS connection URL
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='tcp://127.0.0.1:5760@5760',
        description='URL to the MAVLink stream'
    )

    # 2. Include the Standard MAVROS Launch file (apm.launch)
    # Note: We use AnyLaunchDescriptionSource here, NOT PythonLaunchDescriptionSource
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'tgt_system': '1',
            'tgt_component': '1',
            'log_output': 'screen'
        }.items()
    )

    # 3. Start YOUR Custom Node
    my_node = Node(
        package='navigation',
        executable='navigation',
        name='mission_controller',
        output='screen'
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_launch,
        my_node
    ])