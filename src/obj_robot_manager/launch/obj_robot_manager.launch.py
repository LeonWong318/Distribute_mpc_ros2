from launch import LaunchDescription
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

    data_paths = {
        'map_path': os.path.join(workspace_root, config['map_path']),
        'graph_path': os.path.join(workspace_root, config['graph_path']),
        'schedule_path': os.path.join(workspace_root, config['robot_spec_path']),  
        'robot_start_path': os.path.join(workspace_root, config['robot_start_path'])
    }

    config_paths = {
        'mpc_config_path': os.path.join(workspace_root, config['mpc_config_path']),
        'robot_config_path': os.path.join(workspace_root, config['robot_config_path'])
    }
    

    for path_name, path in {**data_paths, **config_paths}.items():
        if not os.path.exists(path):
            print(f"Error: {path_name} file not found at {path}", file=sys.stderr)
            sys.exit(1)

    robot_manager = Node(
        package='obj_robot_manager',
        executable='robot_manager',
        name='robot_manager',
        parameters=[{
            **data_paths,
            **config_paths,
            'cluster_package': 'obj_robot_cluster',
            'publish_frequency': config.get('manager_publish_frequency', 10.0)
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_manager
    ])