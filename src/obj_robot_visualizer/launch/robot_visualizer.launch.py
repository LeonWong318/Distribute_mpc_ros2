from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('obj_robot_visualizer')
    rviz_config_path = os.path.join(pkg_share, 'config', 'default.rviz')
    
    default_map_path = os.path.join('data', 'scene_1', 'map.json')
    default_graph_path = os.path.join('data', 'scene_1', 'graph.json')
    default_robot_start_path = os.path.join('data', 'scene_1', 'robot_start.json')
    default_robot_spec_path = os.path.join('config','spec_robot.yaml')
    
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
            'path_min_distance': 0.1
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
        visualizer_node,
        rviz_node
    ])