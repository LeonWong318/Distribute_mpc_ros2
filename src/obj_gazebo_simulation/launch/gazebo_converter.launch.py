import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.05',
        description='Command timeout in seconds'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'state_publish_frequency',
        default_value='50.0',
        description='State publishing frequency in Hz'
    )
    
    interval_arg = DeclareLaunchArgument(
        'min_cmd_interval',
        default_value='0.01',
        description='Minimum interval between commands'
    )
    
    enable_noise_arg = DeclareLaunchArgument(
        'enable_noise',
        default_value='false',
        description='Enable noise simulation'
    )
    
    gaussian_stddev_arg = DeclareLaunchArgument(
        'gaussian_stddev',
        default_value='0.05',
        description='Standard deviation for Gaussian noise'
    )
    
    failure_probability_arg = DeclareLaunchArgument(
        'failure_probability',
        default_value='0.05',
        description='Probability of localization failure'
    )
    
    failure_max_deviation_arg = DeclareLaunchArgument(
        'failure_max_deviation',
        default_value='1.0',
        description='Maximum deviation on failure'
    )
    
    robot_id = LaunchConfiguration('robot_id')
    command_timeout = LaunchConfiguration('command_timeout')
    state_publish_frequency = LaunchConfiguration('state_publish_frequency')
    min_cmd_interval = LaunchConfiguration('min_cmd_interval')
    enable_noise = LaunchConfiguration('enable_noise')
    gaussian_stddev = LaunchConfiguration('gaussian_stddev')
    failure_probability = LaunchConfiguration('failure_probability')
    failure_max_deviation = LaunchConfiguration('failure_max_deviation')
    
    converter_node = Node(
        package='obj_gazebo_simulation',
        executable='gazebo_converter',
        name='gazebo_converter',
        namespace=['robot_', robot_id],
        parameters=[{
            'robot_id': robot_id,
            'command_timeout': command_timeout,
            'state_publish_frequency': state_publish_frequency,
            'min_cmd_interval': min_cmd_interval,
            'enable_noise': enable_noise,
            'gaussian_stddev': gaussian_stddev,
            'failure_probability': failure_probability,
            'failure_max_deviation': failure_max_deviation,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_id_arg,
        timeout_arg,
        frequency_arg,
        interval_arg,
        enable_noise_arg,
        gaussian_stddev_arg,
        failure_probability_arg,
        failure_max_deviation_arg,
        converter_node
    ])