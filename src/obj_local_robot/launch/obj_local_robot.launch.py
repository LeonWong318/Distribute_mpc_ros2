from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
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

    # Setup Python path
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX', '') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )

    # === Launch Arguments ===
    # Robot ID argument
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )

    # Controller type argument
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value=config.get('controller_type', 'pure_pursuit'),
        description='Controller type (pure_pursuit/cbf/lqr/lqr_update)'
    )

    # === Node Configuration ===
    local_robot = Node(
        package='obj_local_robot',
        executable='local_robot',
        name='local_robot',
        parameters=[{
            # --- Robot Configuration ---
            'robot_id': LaunchConfiguration('robot_id'),

            # --- Motion Constraints ---
            'max_velocity': config.get('max_velocity', 1.0),
            'max_angular_velocity': config.get('max_angular_velocity', 1.0),
            'control_frequency': config.get('control_frequency', 30.0),

            # --- File Paths ---
            'robot_config_path': os.path.join(workspace_root, config.get('robot_config_path', 'config/spec_robot.yaml')),
            'robot_start_path': os.path.join(workspace_root, config.get('robot_start_path', 'data/test_data/robot_start.json')),
            'robot_graph_path': os.path.join(workspace_root, config.get('graph_path', 'data/test_data/graph.json')),
            'robot_schedule_path': os.path.join(workspace_root, config.get('robot_spec_path', 'data/test_data/schedule.csv')),

            # --- Communication ---
            'cluster_wait_timeout': config.get('cluster_wait_timeout', 30.0),

            # --- Controller Selection ---
            'controller_type': LaunchConfiguration('controller_type'),
            'mpc_ts': config.get('mpc_ts', 0.3),

            # --- Pure Pursuit Parameters ---
            'lookahead_distance': config.get('lookahead_distance', 0.8),
            'lookahead_time': config.get('lookahead_time', 0.5),
            'lookahead_style': config.get('lookahead_style', 'time'),
            'alpha': config.get('alpha', 0.1),

            # --- LQR Parameters ---
            'lqr_q_pos': config.get('lqr_q_pos', 1000.0),
            'lqr_q_theta': config.get('lqr_q_theta', 100.0), 
            'lqr_r_v': config.get('lqr_r_v', 0.001),
            'lqr_r_omega': config.get('lqr_r_omega', 0.01),

            # --- LQR update Parameters ---
            'lqr_update_q_pos': config.get('lqr_update_q_pos', 1000.0),
            'lqr_update_q_theta': config.get('lqr_update_q_theta', 100.0),
            'lqr_update_r_v': config.get('lqr_update_r_v', 0.001),
            'lqr_update_r_omega': config.get('lqr_update_r_omega', 0.01),
            'lqr_lookahead_dist': config.get('lqr_lookahead_dist', 0.8),
            'lqr_lookahead_time': config.get('lqr_lookahead_time', 0.5),
            'lqr_lookahead_style': config.get('lqr_lookahead_style', 'time'),
        }],
        output='screen'
    )

    return LaunchDescription([
        pythonpath_cmd,
        robot_id_arg,
        controller_type_arg,
        local_robot
    ])