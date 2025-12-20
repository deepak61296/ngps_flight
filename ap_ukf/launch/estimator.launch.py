from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ap_ukf',
            executable='fusion_ros',
            name='fusion_ros',
            output='screen',
#            prefix=['xterm -e gdb -ex run --args'],
            parameters=[os.path.join(get_package_share_directory("ap_ukf"), 'params', 'estimator_config.yaml'), ],
           ),
])
