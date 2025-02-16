from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 设置PYTHONPATH环境变量
    pythonpath_cmd = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.getenv('CONDA_PREFIX') + '/lib/python3.8/site-packages:' + os.getenv('PYTHONPATH', '')
    )

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
            'mpc_config_path': "config/mpc_default.yaml",
            'robot_config_path': "config/spec_robot.yaml",
            'planner_search_range': 10,
            'nominal_speed_ratio': 0.8
        }],
        output='screen'
    )
   
    return LaunchDescription([
        pythonpath_cmd,  # 添加环境变量设置
        robot_id_arg,
        robot_node
    ])