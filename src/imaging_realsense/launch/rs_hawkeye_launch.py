#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
    
    #Image processor node
    image_processor = Node(
        package='imaging_realsense',
        executable='image_processor',
        name='image_processor',
        output='screen',
    )


    orchestrator = Node(
        package='orchestrator',
        executable='orchestrator',
        name='orchestrator',
        output='screen',
        emulate_tty=True,
    )

    streaming = Node(
        package='streaming',
        executable='streaming',
        name='streaming',
        output='screen',
    )

    object_detection = Node(
        package='object_detection',
        executable='object_detection',
        name='object_detection',
        output='screen',
    )
    
    return LaunchDescription([
        #realsense_launch,
        orchestrator,
        image_processor,
        # object_detection, 
        streaming,
    ])
