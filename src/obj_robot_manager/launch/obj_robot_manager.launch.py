from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    
    data_paths = {
        'map_path': os.path.join(workspace_root, 'data/scene_1/map.json'),
        'graph_path': os.path.join(workspace_root, 'data/scene_1/graph.json'),
        'schedule_path': os.path.join(workspace_root, 'data/scene_1/schedule.csv'),
        'robot_start_path': os.path.join(workspace_root, 'data/scene_1/robot_start.json')
    }
    
    config_paths = {
        'mpc_config_path': os.path.join(workspace_root, 'config/mpc_default.yaml'),
        'robot_config_path': os.path.join(workspace_root, 'config/spec_robot.yaml')
    }
    
    robot_manager = Node(
        package='obj_robot_manager',
        executable='robot_manager',
        name='robot_manager',
        parameters=[{
            **data_paths,
            **config_paths,
            'cluster_package': 'obj_robot_cluster', 
            'publish_frequency': 10.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_manager
    ])