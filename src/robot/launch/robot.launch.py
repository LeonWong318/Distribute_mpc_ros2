from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
    
    # 返回启动描述
    return LaunchDescription([
        robot_id_arg,
        robot_node
    ])