from setuptools import setup
import os
from glob import glob

package_name = 'gemini335_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf/modules'), glob('urdf/modules/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes/senior_omni_robot_meshes'), glob('meshes/senior_omni_robot_meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lowsing',
    maintainer_email='lowsing@todo.todo',
    description='Orbbec Gemini 335 perception nodes for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = gemini335_perception.camera_node:main',
            'imu_node = gemini335_perception.imu_node:main',
            'perception_node = gemini335_perception.perception_node:main',
            'sim_perception_node = gemini335_perception.sim_perception_node:main',
            'intrusion_guard = gemini335_perception.camera_intrusion_guard_node:main',
        ],
    },
)
