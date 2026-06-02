import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')
    params_file = os.path.join(pkg_dir, 'config', 'gemini335_params.yaml')

    perception_node = Node(
        package='gemini335_perception',
        executable='perception_node',
        name='gemini335_perception',
        parameters=[params_file],
        output='screen',
    )

    intrusion_guard_node = Node(
        package='gemini335_perception',
        executable='intrusion_guard',
        name='camera_intrusion_guard',
        parameters=[{
            'intrusion_distance': 1.0,
            'intrusion_angle_range': 60.0,
            'confirm_frames': 3,
            'clear_frames': 10,
        }],
        output='screen',
    )

    return LaunchDescription([
        perception_node,
        intrusion_guard_node,
    ])
