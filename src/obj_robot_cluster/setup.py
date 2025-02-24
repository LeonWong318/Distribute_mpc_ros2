import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_robot_cluster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools','shapely'],
    zip_safe=True,
    maintainer='zihao',
    maintainer_email='luzihaoprivate@gmail.com',
    description='ROS2 robot cluster implementation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_cluster = obj_robot_cluster.robot_cluster:main'
        ],
    },
)