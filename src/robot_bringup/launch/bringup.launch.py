import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_serial  = get_package_share_directory('robot_serial')
    pkg_imu     = get_package_share_directory('robot_imu')
    pkg_lidar   = get_package_share_directory('robot_lidar')
    pkg_odom    = get_package_share_directory('robot_odometry')
    pkg_percep  = get_package_share_directory('gemini335_perception')
    pkg_bringup = get_package_share_directory('robot_bringup')

    # Launch 参数: 串口设备路径
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyTHS0',
        description='UART device path to STM32'
    )

    # 1. URDF → robot_state_publisher (统一管理所有静态 TF)
    urdf_file = os.path.join(pkg_percep, 'urdf', 'robot_cell.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        serial_port_arg,
        # 1. TF 树 (URDF 统一管理)
        robot_state_publisher,
        # 2. 底盘串口驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_serial, 'launch', 'serial.launch.py'))),
        # 3. 底盘 IMU 滤波 (Madgwick)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_imu, 'launch', 'imu.launch.py'))),
        # 4. 雷达驱动 (static_tf 已移除)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_lidar, 'launch', 'lidar.launch.py'))),
        # 5. 里程计
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_odom, 'launch', 'odometry.launch.py'))),
        # 6. 雷达滤波 (scan_raw → scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'laserfilter.launch.py'))),
        # 7. Gemini 335 相机感知 + 闯入守卫
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_percep, 'launch', 'perception.launch.py'))),
    ])