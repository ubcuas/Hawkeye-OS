#!/usr/bin/env python3
"""
Launch file for RealSense camera with timestamp-synchronized GPS processing.
Uses message_filters for accurate GPS-to-image matching.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    sync_slop_arg = DeclareLaunchArgument(
        'sync_slop',
        default_value='0.1',
        description='Time tolerance for synchronization in seconds (default: 0.1 = 100ms)'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='Queue size for message synchronization buffer'
    )
    
    # Path to official RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py'
            )
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false',
            'camera_namespace': 'camera',
            'camera_name': 'camera',
        }.items()
    )
    
    # Synchronized image processor node
    image_processor_synced = Node(
        package='imaging_realsense',
        executable='image_processor_synced',
        name='image_processor_synced',
        output='screen',
        parameters=[{
            'sync_slop': LaunchConfiguration('sync_slop'),
            'queue_size': LaunchConfiguration('queue_size'),
        }]
    )
    
    return LaunchDescription([
        sync_slop_arg,
        queue_size_arg,
        realsense_launch,
        image_processor_synced,
    ])
