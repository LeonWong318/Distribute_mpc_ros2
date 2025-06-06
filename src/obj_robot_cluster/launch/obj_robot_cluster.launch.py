from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import yaml

def generate_launch_description():
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

    if config.get('controller_type', 'lqr') == 'lqr_update' and config.get('lqr_lookahead_style','dist') == 'time':
        default_lookahead_time = config.get('lqr_lookahead_time',0.5)
    elif config.get('controller_type', 'lqr') == 'pure_pursuit' and config.get('lookahead_style','dist') == 'time':
        default_lookahead_time = config.get('lookahead_time',0.5)
    else:
        default_lookahead_time = 0.0
    default_robot_id = '0' 
    default_close_to_target_rate = config.get('close_to_target_rate', 0.5)
    default_map_path = config.get('map_path', '')
    default_graph_path = config.get('graph_path', '')
    default_schedule_path = config.get('robot_spec_path', '')
    default_robot_start_path = config.get('robot_start_path', '')
    default_control_frequency = config.get('cluster_control_frequency', 1.0)
    default_mpc_config_path = config.get('mpc_config_path', 'config/mpc_default.yaml')
    default_robot_config_path = config.get('robot_config_path', 'config/spec_robot.yaml')
    default_mpc_method = config.get('mpc_method', 'state_fusion')
    

    declare_args = [
        DeclareLaunchArgument('robot_id', default_value=default_robot_id, description='Robot ID'),
        DeclareLaunchArgument('lookahead_time', default_value = str(default_lookahead_time), description='Lookahead Time'),
        DeclareLaunchArgument('close_to_target_rate', default_value = str(default_close_to_target_rate), description = 'close_to_target_rate'),
        DeclareLaunchArgument('map_path', default_value=default_map_path, description='Path to the map JSON file'),
        DeclareLaunchArgument('graph_path', default_value=default_graph_path, description='Path to the graph JSON file'),
        DeclareLaunchArgument('schedule_path', default_value=default_schedule_path, description='Path to the schedule CSV file'),
        DeclareLaunchArgument('robot_start_path', default_value=default_robot_start_path, description='Path to the robot start JSON file'),
        DeclareLaunchArgument('control_frequency', default_value=str(default_control_frequency), description='Control frequency in Hz'),
        DeclareLaunchArgument('mpc_config_path', default_value=default_mpc_config_path, description='Path to the MPC configuration file'),
        DeclareLaunchArgument('robot_config_path', default_value=default_robot_config_path, description='Path to the robot specification file'),
        DeclareLaunchArgument('mpc_method', default_value=default_mpc_method, description='mpc method')
    ]

    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX', '') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )

    # Cluster node with parameters
    robot_cluster = Node(
        package='obj_robot_cluster',
        executable='robot_cluster',
        name='robot_cluster',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'lookahead_time': LaunchConfiguration('lookahead_time'),
            'close_to_target_rate': LaunchConfiguration('close_to_target_rate'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'mpc_config_path': LaunchConfiguration('mpc_config_path'),
            'robot_config_path': LaunchConfiguration('robot_config_path'),
            'map_path': LaunchConfiguration('map_path'),
            'graph_path': LaunchConfiguration('graph_path'),
            'schedule_path': LaunchConfiguration('schedule_path'),
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'mpc_method': LaunchConfiguration('mpc_method'),
            'enable_traj_share': config.get('enable_traj_share', True)
        }],
        output='screen'
    )

    return LaunchDescription([pythonpath_cmd] + declare_args + [robot_cluster])