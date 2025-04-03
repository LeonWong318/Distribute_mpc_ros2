from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import yaml

def generate_launch_description():
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    sys_config_path = os.path.join(workspace_root, 'config', 'sys_config.yaml')
    
    print(f"Looking for config file at: {sys_config_path}")
    
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
    
    default_map_path = os.path.join(workspace_root, config['map_path'])
    default_graph_path = os.path.join(workspace_root, config['graph_path'])
    default_schedule_path = os.path.join(workspace_root, config['robot_spec_path'])
    default_robot_start_path = os.path.join(workspace_root, config['robot_start_path'])
    default_mpc_config_path = os.path.join(workspace_root, config['mpc_config_path'])
    default_robot_config_path = os.path.join(workspace_root, config['robot_config_path'])
    
    for path_name, path in {
        'map_path': default_map_path,
        'graph_path': default_graph_path,
        'schedule_path': default_schedule_path,
        'robot_start_path': default_robot_start_path,
        'mpc_config_path': default_mpc_config_path,
        'robot_config_path': default_robot_config_path
    }.items():
        if not os.path.exists(path):
            print(f"Error: {path_name} file not found at {path}", file=sys.stderr)
            sys.exit(1)
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Path to the map file'
    )
    
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        default_value=default_graph_path,
        description='Path to the graph file'
    )
    
    schedule_path_arg = DeclareLaunchArgument(
        'schedule_path',
        default_value=default_schedule_path,
        description='Path to the schedule file'
    )
    
    robot_start_path_arg = DeclareLaunchArgument(
        'robot_start_path',
        default_value=default_robot_start_path,
        description='Path to the robot start file'
    )
    
    mpc_config_path_arg = DeclareLaunchArgument(
        'mpc_config_path',
        default_value=default_mpc_config_path,
        description='Path to the MPC configuration file'
    )
    
    robot_config_path_arg = DeclareLaunchArgument(
        'robot_config_path',
        default_value=default_robot_config_path,
        description='Path to the robot configuration file'
    )
    
    robot_manager = Node(
        package='obj_robot_manager',
        executable='robot_manager',
        name='robot_manager',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'graph_path': LaunchConfiguration('graph_path'),
            'schedule_path': LaunchConfiguration('schedule_path'),
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'mpc_config_path': LaunchConfiguration('mpc_config_path'),
            'robot_config_path': LaunchConfiguration('robot_config_path'),
            'cluster_package': 'obj_robot_cluster',
            'publish_frequency': config.get('manager_publish_frequency', 10.0)
        }],
        output='screen'
    )
    
    return LaunchDescription([
        map_path_arg,
        graph_path_arg,
        schedule_path_arg,
        robot_start_path_arg,
        mpc_config_path_arg,
        robot_config_path_arg,
        robot_manager
    ])