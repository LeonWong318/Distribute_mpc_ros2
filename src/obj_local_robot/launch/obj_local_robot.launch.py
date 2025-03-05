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
    
    # Robot ID argument
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )
    
    # Robot node
    local_robot = Node(
        package='obj_local_robot',
        executable='local_robot',
        name='local_robot',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'max_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'control_frequency': 10.0,
            'lookahead_distance': 0.3,  # Added for Pure Pursuit
            'robot_config_path': "config/spec_robot.yaml",
            'robot_start_path': "data/test_data/robot_start.json",
            'robot_graph_path': 'data/test_data/graph.json',
            'robot_schedule_path': 'data/test_data/schedule.csv',
            'cluster_wait_timeout': 30.0,  # Added timeout parameter
            'alpha': 0.1,  # Added tuning parameter
            'ts': 0.2  # Added sampling time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pythonpath_cmd,
        robot_id_arg,
        local_robot
    ])