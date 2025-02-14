from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
#    current_dir = os.path.dirname(os.path.abspath(__file__))
#    src_dir = os.path.dirname(os.path.dirname(current_dir))
#    set_pythonpath = SetEnvironmentVariable(
#        name='PYTHONPATH',
#        value=[src_dir] + (os.environ.get('PYTHONPATH', '').split(':') if 'PYTHONPATH' in os.environ else [])
#    )

   robot_id_arg = DeclareLaunchArgument(
       'robot_id',
       default_value='0',
       description='Robot ID'
   )

   robot_node = Node(
       package='robot',
       executable='robot_node',
       name='robot_node',
       parameters=[{
           'robot_id': LaunchConfiguration('robot_id'),
           'max_velocity': 1.0,
           'max_angular_velocity': 1.0,
           'control_frequency': 10.0,
           'mpc_config_path': "config/mpc_config.yaml",
           'robot_config_path': "config/robot_config.yaml",
           'planner_search_range': 10,
           'nominal_speed_ratio': 0.8
       }],
       output='screen'
   )
   
   return LaunchDescription([
        robot_id_arg,
        robot_node
    ])