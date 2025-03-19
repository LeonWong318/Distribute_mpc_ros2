from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Python path setup
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )
    
    # buffer node with parameters
    msg_buffer = Node(
        package='obj_msg_buffer',
        executable='poission_buffer',
        name='poission_buffer',
        parameters=[
                {'num_robots': 2},
                {'enable_gui': True},
                {'mean_delay': 2.0}
            ],
        output='screen'
    )
    
    return LaunchDescription([pythonpath_cmd]  + [msg_buffer])