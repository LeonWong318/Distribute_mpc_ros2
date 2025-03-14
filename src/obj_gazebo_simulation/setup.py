import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_gazebo_simulation'

# Function to collect all files while keeping directory structure
def package_files(source_dir, target_dir):
    paths = []
    for root, _, files in os.walk(source_dir):
        for file in files:
            full_path = os.path.join(root, file)
            relative_path = os.path.relpath(full_path, source_dir)
            paths.append((os.path.join('share', package_name, target_dir, os.path.dirname(relative_path)), [full_path]))
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
        # Models - Preserve structure
    ] + package_files('models/mobile_robot', 'models/mobile_robot') 
      + package_files('models/test_data', 'models/test_data'),
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
