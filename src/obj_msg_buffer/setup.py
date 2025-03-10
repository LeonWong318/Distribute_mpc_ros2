import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_msg_buffer'

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
    maintainer='Yinsong',
    maintainer_email='wangyinsong01@gmail.com',
    description='msg buffer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poission_buffer = obj_msg_buffer.poission_buffer:main'
        ],
    },
)