from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Setup Python path
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )
    
    # === Launch Arguments ===
    # Robot ID argument
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0'
    )
    
    # Controller type argument
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='lqr' # choose pure_pursuit/cbf/lqr/lqr_update
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
            'max_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'control_frequency': 30.0,
            
            # --- File Paths ---
            'robot_config_path': "config/spec_robot.yaml",
            'robot_start_path': "data/test_data/robot_start.json",
            'robot_graph_path': 'data/test_data/graph.json',
            'robot_schedule_path': 'data/test_data/schedule.csv',
            
            # --- Communication ---
            'cluster_wait_timeout': 30.0,  # Timeout for waiting for cluster node
            
            # --- Controller Selection ---
            'controller_type': LaunchConfiguration('controller_type'),
            
            # --- Pure Pursuit Parameters ---
            'lookahead_distance': .8,
            'alpha': 0.0,  # Tuning parameter for velocity reduction at high curvature
            
            # --- LQR Parameters ---
            'lqr_q_pos': 1000.0,    # Position error weight
            'lqr_q_theta': 100.0,    # Heading error weight
            'lqr_r_v': 0.001,        # Linear velocity control weight
            'lqr_r_omega': 0.01,    # Angular velocity control weight

            # --- LQR update Parameters ---
            'lqr_update_q_pos': 1000.0,    # Position error weight
            'lqr_update_q_theta': 100.0,    # Heading error weight
            'lqr_update_r_v': 0.001,        # Linear velocity control weight
            'lqr_update_r_omega': 0.01,    # Angular velocity control weight
            'lqr_lookahead_dist':.8,     # Lookahead Distance
            # --- CBF Parameters (for future implementation) ---

        }],
        output='screen'
    )
    
    return LaunchDescription([
        pythonpath_cmd,
        robot_id_arg,
        controller_type_arg,
        local_robot
    ])