# Import necessary ROS2 and launch modules
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext

def generate_launch_description():
    # Ensure Python packages can be found
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX', '') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )

    # Declare argument for map selection
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='test_data.world',
        description='Name of the map file to load'
    )

    # Define the world file dynamically based on the selected map name
    
    map_name = LaunchConfiguration('map_name')

    # Get the package directory
    pkg_dir = get_package_share_directory('obj_gazebo_simulation')

    # Set Gazebo environment variables properly
    gazebo_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_dir, 'models')
    )

    # Use a proper substitution for the world file
    world_file_path = PathJoinSubstitution([
    pkg_dir, 'worlds',  map_name ])


    # Launch Gazebo with the selected world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot entity inside the simulation
    spawn_entity = Node(
        package='obj_gazebo_simulation',
        executable='demo',
        parameters=[{
            
            'robot_setup_path': "data/test_data/robot_start.json",
            
        }],
        #arguments=['WarehouseBot', 'demo', '-1.5', '-4.0', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        pythonpath_cmd,
        gazebo_model_path_cmd,
        map_name_arg,
        gazebo,
        spawn_entity
    ])
