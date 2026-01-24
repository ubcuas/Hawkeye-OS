import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. CONFIGURATION ---
    # Define the MAVROS connection URL (default to SITL/Sim)
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='tcp://127.0.0.1:5760', # Adjusted to standard SITL port
        description='URL to the MAVLink stream'
    )

    # --- 2. START MAVROS ---
    # We include the standard 'apm.launch' file from the mavros package
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

    # --- 3. START NAVIGATION NODE ---
    navigation_node = Node(
        package='navigation',
        executable='navigation',      # Must match 'entry_points' in navigation/setup.py
        name='navigation_node',
        output='screen',
        emulate_tty=True
    )

    orchestrator_node = Node(
        package='orchestrator',
        executable='orchestrator', # Must match 'entry_points' in orchestrator/setup.py
        name='orchestrator_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_launch,
        navigation_node,
        orchestrator_node
    ])