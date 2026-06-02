import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    use_sime_time = LaunchConfiguration('use_sim_time', default='false')

    use_sime_time_arg = DeclareLaunchArgument (
        'use_sim_time',
        default_value='false',
    )
    
    remap_scan_topic = SetRemap(src='/scan', dst='/scan_raw')

    # static_tf_node removed, now managed by URDF

    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0', 
            'frame_id': 'laser_link',
            'use_sim_time': use_sime_time
        }.items()
    )

    return LaunchDescription([
        remap_scan_topic,
        lidar_driver_launch
    ])