#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    reference_image_arg = DeclareLaunchArgument(
        'reference_image_path',
        default_value='',
        description='Path to the reference image for NGPS localization'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    ngps_config_arg = DeclareLaunchArgument(
        'ngps_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ngps_ros2'),
            'config',
            'ngps_config.yaml'
        ]),
        description='Path to the NGPS configuration file'
    )
    
    vips_config_arg = DeclareLaunchArgument(
        'vips_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vins'),
            'config',
            'high_alt',
            'high_alt_mono_imu_config.yaml'
        ]),
        description='Path to the VIPS configuration file'
    )
    
    ukf_config_arg = DeclareLaunchArgument(
        'ukf_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ukf'),
            'params',
            'estimator_config.yaml'
        ]),
        description='Path to the UKF configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    ngps_node = Node(
        package='ap_ngps_ros2',
        executable='ngps_localization_node.py',
        name='ngps_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ngps_config_file'),
            {
                'reference_image_path': LaunchConfiguration('reference_image_path'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    vips_node = Node(
        package='vins',
        executable='vins_node',
        name='vips_node',
        output='screen',
        parameters=[
            LaunchConfiguration('vips_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        arguments=[LaunchConfiguration('vips_config_file')],
        remappings=[
            ('/vins_estimator/odometry', '/odometry/vio'),
            ('/vins_estimator/path', '/vips/path'),
            ('/vins_estimator/pose', '/vips/pose'),
        ]
    )
    
    ukf_node = Node(
        package='ap_ukf',
        executable='fusion_ros',
        name='fusion_ros',
        output='screen',
        parameters=[
            LaunchConfiguration('ukf_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/eca_a9/imu', '/imu/data'),
            ('/odometry/vio', '/odometry/vio'),
            ('/odometry/vps', '/odometry/vps'),
            ('/topic/sensor/odom_state', '/fused/odometry'),
        ]
    )
    
    return LaunchDescription([
        reference_image_arg,
        camera_topic_arg,
        ngps_config_arg,
        vips_config_arg,
        ukf_config_arg,
        use_sim_time_arg,
        
        ngps_node,
        vips_node,
        ukf_node,
    ])
