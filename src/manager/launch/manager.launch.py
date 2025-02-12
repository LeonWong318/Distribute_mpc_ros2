from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    max_robots_arg = DeclareLaunchArgument(
        'max_robots',
        default_value='10',
        description='Maximum number of robots to manage'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.5',
        description='Minimum safety distance between robots'
    )
    
    check_frequency_arg = DeclareLaunchArgument(
        'check_frequency',
        default_value='10.0',
        description='Frequency of safety checks'
    )
    
    # 创建管理器节点
    manager_node = Node(
        package='manager',
        executable='robot_manager',
        name='robot_manager',
        parameters=[{
            'max_robots': LaunchConfiguration('max_robots'),
            'safety_distance': LaunchConfiguration('safety_distance'),
            'check_frequency': LaunchConfiguration('check_frequency')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        max_robots_arg,
        safety_distance_arg,
        check_frequency_arg,
        manager_node
    ])