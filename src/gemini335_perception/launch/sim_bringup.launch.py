import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')

    # Arguments
    slam_arg = DeclareLaunchArgument('slam', default_value='False', description='Whether run SLAM')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='False', description='Whether run Nav2')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='False', description='Whether run RViz2')

    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    rviz = LaunchConfiguration('rviz')

    # Gazebo Sim
    gazebo_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gazebo_sim.launch.py'))
    )

    # Perception
    perception_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sim_perception.launch.py'))
    )

    # SLAM
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sim_slam.launch.py')),
        condition=IfCondition(slam)
    )

    # Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sim_nav2.launch.py')),
        launch_arguments={
            'map': os.path.join('/home/lowsing/gemini335_ros2_ws_nav', 'maps', 'sim_map.yaml'),
            'params_file': os.path.join(pkg_dir, 'config', 'sim_nav2_params.yaml')
        }.items(),
        condition=IfCondition(nav2)
    )

    # Delay Nav2 to allow Gazebo to start up completely and publish a stable /clock
    delayed_nav2_cmd = TimerAction(
        period=8.0,
        actions=[nav2_cmd]
    )

    # RViz2 (with lightweight preset config)
    from launch_ros.actions import Node
    rviz_config = os.path.join(pkg_dir, 'config', 'sim_nav_lite.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription()
    ld.add_action(slam_arg)
    ld.add_action(nav2_arg)
    ld.add_action(rviz_arg)
    
    ld.add_action(gazebo_sim_cmd)
    ld.add_action(perception_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(delayed_nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
