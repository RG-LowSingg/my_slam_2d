"""Launch camera and IMU as separate nodes."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')
    params_file = os.path.join(pkg_dir, 'config', 'gemini335_params.yaml')

    return LaunchDescription([
        Node(
            package='gemini335_perception',
            executable='camera_node',
            name='gemini335_camera',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='gemini335_perception',
            executable='imu_node',
            name='gemini335_imu',
            parameters=[params_file],
            output='screen',
        ),
    ])
