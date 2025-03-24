import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'obj_gazebo_simulation'

def package_files(source_dir, target_dir):
    paths = []
    if os.path.exists(source_dir):
        for root, _, files in os.walk(source_dir):
            for file in files:
                full_path = os.path.join(root, file)
                relative_path = os.path.relpath(full_path, source_dir)
                paths.append((os.path.join('share', package_name, target_dir, os.path.dirname(relative_path)), [full_path]))
    return paths

models_data_files = []
models_dir = 'models'
if os.path.exists(models_dir):
    model_subdirs = [d for d in os.listdir(models_dir) 
                    if os.path.isdir(os.path.join(models_dir, d))]
    
    for subdir in model_subdirs:
        subdir_path = os.path.join(models_dir, subdir)
        target_path = os.path.join(models_dir, subdir)
        models_data_files.extend(package_files(subdir_path, target_path))

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
        # Models
    ] + models_data_files,
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