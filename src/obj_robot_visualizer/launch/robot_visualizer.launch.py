from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('obj_robot_visualizer')
    rviz_config_path = os.path.join(pkg_share, 'config', 'default.rviz')
    
    return LaunchDescription([
        Node(
            package='obj_robot_visualizer',
            executable='robot_visualizer',
            name='robot_visualizer',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])