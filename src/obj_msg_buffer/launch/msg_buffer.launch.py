from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Python path setup
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )
    
    # === Launch Arguments ===
    robot_start_path_arg = DeclareLaunchArgument(
        'robot_start_path',
        default_value='data/test_data/robot_start.json'
    )
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true'
    )
    
    mean_delay_arg = DeclareLaunchArgument(
        'mean_delay',
        default_value='1.0'
    )
    
    # === Node Configuration ===
    msg_buffer = Node(
        package='obj_msg_buffer',
        executable='poission_buffer',
        name='poission_buffer',
        parameters=[{
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'enable_gui': LaunchConfiguration('enable_gui'),
            'mean_delay': LaunchConfiguration('mean_delay')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pythonpath_cmd,
        robot_start_path_arg,
        enable_gui_arg,
        mean_delay_arg,
        msg_buffer
    ])