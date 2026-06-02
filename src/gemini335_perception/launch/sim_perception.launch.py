import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')
    params_file = os.path.join(pkg_dir, 'config', 'gemini335_params.yaml')
    

    # Simulation Perception Node
    sim_perception_node = Node(
        package='gemini335_perception',
        executable='sim_perception_node',
        name='sim_perception_node',
        parameters=[params_file, {'enable_sim_noise': True}],
        output='screen'
    )

    return LaunchDescription([
        sim_perception_node
    ])
