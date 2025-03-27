from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import yaml

def generate_launch_description():
    # Get workspace root
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    sys_config_path = os.path.join(workspace_root, 'config', 'sys_config.yaml')
    test_config_path = os.path.join(workspace_root, 'config', 'test_config.yaml')

    # Load system configuration
    if not os.path.exists(sys_config_path):
        print(f"Error: System configuration file {sys_config_path} not found.", file=sys.stderr)
        sys.exit(1)

    try:
        with open(sys_config_path, 'r') as f:
            sys_config = yaml.safe_load(f)
            if sys_config is None:
                print(f"Error: System configuration file {sys_config_path} is empty or invalid.", file=sys.stderr)
                sys.exit(1)
    except Exception as e:
        print(f"Error reading system configuration file {sys_config_path}: {str(e)}", file=sys.stderr)
        sys.exit(1)

    # Load test configuration with defaults
    test_config = {
        'latency_values': [0.0, 0.1, 0.3, 0.5, 1.0],
        'iterations_per_latency': 20,
        'timeout_seconds': 300.0
    }
    
    if os.path.exists(test_config_path):
        try:
            with open(test_config_path, 'r') as f:
                loaded_config = yaml.safe_load(f)
                if loaded_config:
                    test_config.update(loaded_config)
        except Exception as e:
            print(f"Warning: Error reading test configuration file {test_config_path}: {str(e)}", file=sys.stderr)
            print("Using default test configuration.", file=sys.stderr)

    # Setup Python path
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX', '') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )

    # === Launch Arguments ===
    # Test configuration file path
    test_config_arg = DeclareLaunchArgument(
        'test_config_path',
        default_value=test_config_path,
        description='Path to test configuration file'
    )

    # Robot start path
    robot_start_path_arg = DeclareLaunchArgument(
        'robot_start_path',
        default_value=os.path.join(workspace_root, sys_config.get('robot_start_path', 'data/test_data/robot_start.json')),
        description='Path to robot start configuration'
    )

    # Log directory
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='auto_test_logs',
        description='Directory to store test logs'
    )

    # === Node Configuration ===
    auto_test_node = Node(
        package='obj_auto_test',
        executable='auto_test',
        name='auto_test',
        parameters=[{
            # File paths
            'test_config_path': LaunchConfiguration('test_config_path'),
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'log_dir': LaunchConfiguration('log_dir'),
            
            # System paths 
            'map_path': os.path.join(workspace_root, sys_config.get('map_path', 'data/test_data/map.json')),
            'graph_path': os.path.join(workspace_root, sys_config.get('graph_path', 'data/test_data/graph.json')),
            'robot_config_path': os.path.join(workspace_root, sys_config.get('robot_config_path', 'config/spec_robot.yaml')),
            
            # Test parameters
            'latency_values': test_config['latency_values'],
            'iterations_per_latency': test_config['iterations_per_latency'],
            'timeout_seconds': test_config['timeout_seconds'],
            
            # Launch settings
            'show_terminals': False,
            'workspace_root': workspace_root
        }],
        output='screen'
    )

    return LaunchDescription([
        pythonpath_cmd,
        test_config_arg,
        robot_start_path_arg,
        log_dir_arg,
        auto_test_node
    ])