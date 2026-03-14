#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Skip launching the RealSense camera driver (use when playing from a rosbag)',
    )

    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable 3D deprojection of detections and publish target poses to the navigation node',
    )

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
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_rosbag')),
    )
    
    #Image processor node
    image_processor = Node(
        package='imaging_realsense',
        executable='image_processor',
        name='image_processor',
        output='screen',
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
        parameters=[{
            'enable_navigation': LaunchConfiguration('enable_navigation'),
        }],
    )

    navigation = Node(
        package='navigation',
        executable='navigation',
        name='navigation',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_navigation')),
    )

    return LaunchDescription([
        use_rosbag_arg,
        enable_navigation_arg,
        realsense_launch,
        image_processor,
        object_detection,
        navigation,
        streaming,
    ])
