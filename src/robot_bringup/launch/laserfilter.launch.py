from launch_ros.actions import Node
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'laser_filter_params.yaml')

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters = [params_file],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )
    
    return LaunchDescription([
        laser_filter_node
    ])