import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'pkg_gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Auto-detects Python packages
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # Models
        (os.path.join('share', package_name,'models/small_warehouse/'), glob('./models/small_warehouse/*')),
        (os.path.join('share', package_name, 'models/mobile_robot'), glob('./models/mobile_robot/*')),
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
            'demo = pkg_gazebo_simulation.demo:main'
        ],
    },
)
