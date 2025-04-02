from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import sys

def generate_launch_description():
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )
    
    pkg_share = get_package_share_directory('obj_msg_buffer')

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    sys_config_path = os.path.join(workspace_root, 'config', 'sys_config.yaml')
    
    if not os.path.exists(sys_config_path):
        print(f"Error: Configuration file {sys_config_path} not found.", file=sys.stderr)
        sys.exit(1)
    
    try:
        with open(sys_config_path, 'r') as f:
            config = yaml.safe_load(f)
            if config is None:
                print(f"Error: Configuration file {sys_config_path} is empty or invalid.", file=sys.stderr)
                sys.exit(1)
    except Exception as e:
        print(f"Error reading configuration file {sys_config_path}: {str(e)}", file=sys.stderr)
        sys.exit(1)
    
    default_robot_start_path = config.get('robot_start_path', 'data/test_data/robot_start.json')
    default_enable_gui = str(config.get('enable_gui', 'true')).lower()
    default_mean_delay = str(config.get('mean_delay', '1.0'))
    default_stddev_delay = str(config.get('stddev_delay', '0.1'))
    
    # === Launch Arguments ===
    robot_start_path_arg = DeclareLaunchArgument(
        'robot_start_path',
        default_value=default_robot_start_path,
        description='Path to the robot start JSON file'
    )
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value=default_enable_gui,
        description='Enable GUI for visualization'
    )
    
    mean_delay_arg = DeclareLaunchArgument(
        'mean_delay',
        default_value=default_mean_delay,
        description='Mean delay for the Poisson process'
    )

    stddev_delay_arg = DeclareLaunchArgument(
        'stddev_delay',
        default_value=default_stddev_delay,
        description='stddev'

    )
    
    # === Node Configuration ===
    msg_buffer = Node(
        package='obj_msg_buffer',
        executable='poission_buffer',
        name='poission_buffer',
        parameters=[{
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'enable_gui': LaunchConfiguration('enable_gui'),
            'mean_delay': LaunchConfiguration('mean_delay'),
            'stddev_delay': LaunchConfiguration('stddev_delay')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pythonpath_cmd,
        robot_start_path_arg,
        enable_gui_arg,
        mean_delay_arg,
        stddev_delay_arg,
        msg_buffer
    ])