#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ref_img_arg = DeclareLaunchArgument(
        'reference_image_path',
        default_value='',
        description='Path to the reference image for localization'
    )
    
    cam_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ngps_ros2'),
            'config',
            'ngps_config.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    ngps_node = Node(
        package='ap_ngps_ros2',
        executable='ngps_localization_node.py',
        name='ngps_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'reference_image_path': LaunchConfiguration('reference_image_path'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            }
        ],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    return LaunchDescription([
        ref_img_arg,
        cam_topic_arg,
        config_arg,
        ngps_node,
    ])
