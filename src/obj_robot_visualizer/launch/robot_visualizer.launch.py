from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
import yaml
import sys

def generate_launch_description():
    pkg_share = get_package_share_directory('obj_robot_visualizer')
    rviz_config_path = os.path.join(pkg_share, 'config', 'default.rviz')
    

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    sys_config_path = os.path.join(workspace_root, 'config', 'sys_config.yaml')
        
    if not os.path.exists(sys_config_path):
        print(f"Error: Configuration file {sys_config_path} not found.", file=sys.stderr)
        sys.exit(1)
    
    try:
        with open(sys_config_path, 'r') as f:
            default_config = yaml.safe_load(f)
            if default_config is None:
                print(f"Error: Configuration file {sys_config_path} is empty or invalid.", file=sys.stderr)
                sys.exit(1)
    except Exception as e:
        print(f"Error reading configuration file {sys_config_path}: {str(e)}", file=sys.stderr)
        sys.exit(1)
    
    required_params = ['map_path', 'graph_path', 'robot_start_path', 'robot_config_path', 'path_min_distance']
    missing_params = [param for param in required_params if param not in default_config]
    
    if missing_params:
        print(f"Error: Missing required parameters in configuration file: {', '.join(missing_params)}", file=sys.stderr)
        sys.exit(1)
    
    default_map_path = default_config['map_path']
    default_graph_path = default_config['graph_path']
    default_robot_start_path = default_config['robot_start_path']
    default_robot_spec_path = default_config['robot_config_path']
    default_min_distance = str(default_config['path_min_distance'])
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Path to the map JSON file'
    )
    
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        default_value=default_graph_path,
        description='Path to the graph JSON file'
    )
    
    robot_start_path_arg = DeclareLaunchArgument(
        'robot_start_path',
        default_value=default_robot_start_path,
        description='Path to the robot start JSON file'
    )
    
    robot_spec_path_arg = DeclareLaunchArgument(
        'robot_spec_path',
        default_value=default_robot_spec_path,
        description='Path to the robot specification file'
    )
    
    path_min_distance_arg = DeclareLaunchArgument(
        'path_min_distance',
        default_value=default_min_distance,
        description='Minimum distance between path points'
    )
    
    visualizer_node = Node(
        package='obj_robot_visualizer',
        executable='robot_visualizer',
        name='robot_visualizer',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'graph_path': LaunchConfiguration('graph_path'),
            'robot_start_path': LaunchConfiguration('robot_start_path'),
            'robot_spec_path': LaunchConfiguration('robot_spec_path'),
            'path_min_distance': LaunchConfiguration('path_min_distance')
        }],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path, '--fixed-frame', 'map'],
        output='screen'
    )
    
    return LaunchDescription([
        map_path_arg,
        graph_path_arg,
        robot_start_path_arg,
        robot_spec_path_arg,
        path_min_distance_arg,
        visualizer_node,
        rviz_node
    ])