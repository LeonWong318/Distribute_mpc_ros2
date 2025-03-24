from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('robot_id', default_value='0', description='Robot ID'),
        DeclareLaunchArgument('map_path', default_value='data/scene_1/map.json'),
        DeclareLaunchArgument('graph_path', default_value='data/scene_1/graph.json'),
        DeclareLaunchArgument('schedule_path', default_value='data/scene_1/schedule.csv'),
        DeclareLaunchArgument('robot_start_path', default_value='data/scene_1/robot_start.json')
    ]
    
    # Python path setup
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )
    
    # Cluster node with parameters
    robot_cluster = Node(
        package='obj_robot_cluster',
        executable='robot_cluster',
        name='robot_cluster',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'control_frequency': 1.0,
            'mpc_config_path': "config/mpc_default.yaml",
            'robot_config_path': "config/spec_robot.yaml",
            'map_path': LaunchConfiguration('map_path'),
            'graph_path': LaunchConfiguration('graph_path'),
            'schedule_path': LaunchConfiguration('schedule_path'),
            'robot_start_path': LaunchConfiguration('robot_start_path')
        }],
        output='screen'
    )
    
    return LaunchDescription([pythonpath_cmd] + declare_args + [robot_cluster])