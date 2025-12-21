#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('ap_vo'),
        'config',
        'vo_config.yaml'
    ])
    
    vo_node = Node(
        package='ap_vo',
        executable='vo_node.py',
        name='vo_node',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([vo_node])
