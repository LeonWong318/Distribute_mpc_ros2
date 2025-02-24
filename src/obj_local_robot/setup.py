import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_local_robot'

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
    maintainer='Yinsong Wang',
    maintainer_email='wangyinsong01@gmail.com',
    description='local robot node so that we can implement pure persuit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_robot = obj_local_robot.local_robot:main'
        ],
    },
)
