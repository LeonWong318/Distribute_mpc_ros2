from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    
    data_paths = {
        'map_path': os.path.join(workspace_root, 'data/test_data/map.json'),
        'graph_path': os.path.join(workspace_root, 'data/test_data/graph.json'),
        'schedule_path': os.path.join(workspace_root, 'data/test_data/schedule.csv'),
        'robot_start_path': os.path.join(workspace_root, 'data/test_data/robot_start.json')
    }
    
    robot_manager = Node(
        package='obj_robot_manager',
        executable='robot_manager',
        name='robot_manager',
        parameters=[{
            **data_paths,
            'publish_frequency': 10.0
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_manager
    ])