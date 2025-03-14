import os
import shutil
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_gazebo_simulation'

# Ensure models folder is copied completely
def package_files(directory):
    """Recursively collect all files under 'directory'"""
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Models - Copy entire directory contents
        (os.path.join('share', package_name, 'models/mobile_robot'), package_files('models/mobile_robot')),
        (os.path.join('share', package_name, 'models/test_data'), package_files('models/test_data')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yinsong Wang',
    maintainer_email='wangyinsong01@gmail.com',
    description='Package to launch Gazebo simulation by ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_robot_spawner = obj_gazebo_simulation.multi_robot_spawner:main',
            'gazebo_converter = obj_gazebo_simulation.gazebo_converter:main'
        ],
    },
)
