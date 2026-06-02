import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_dir, 'config', 'sim_nav2_params.yaml'))

    # Include Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(nav2_bringup_cmd)

    return ld
