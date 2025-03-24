import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    sys_config_path = os.path.join(workspace_root, 'config', 'sys_config.yaml')

    if not os.path.exists(sys_config_path):
        print(f"Error: Configuration file {sys_config_path} not found.", file=sys.stderr)
        sys.exit(1)

    try:
        with open(sys_config_path, 'r') as f:
            config = yaml.safe_load(f)
            if config is None:
                print(f"Error: Configuration file {sys_config_path} is empty or invalid.", file=sys.stderr)
                sys.exit(1)
    except Exception as e:
        print(f"Error reading configuration file {sys_config_path}: {str(e)}", file=sys.stderr)
        sys.exit(1)

    default_robot_id = '0'
    default_command_timeout = str(config.get('command_timeout', '0.05'))
    default_state_publish_frequency = str(config.get('converter_state_publish_frequency', '50.0'))
    default_min_cmd_interval = str(config.get('min_cmd_interval', '0.01'))
    default_enable_noise = str(config.get('enable_noise', 'false')).lower()
    default_gaussian_stddev = str(config.get('gaussian_stddev', '0.05'))
    default_failure_probability = str(config.get('failure_probability', '0.05'))
    default_failure_max_deviation = str(config.get('failure_max_deviation', '1.0'))

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value=default_robot_id,
        description='Robot ID'
    )

    timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value=default_command_timeout,
        description='Command timeout in seconds'
    )

    frequency_arg = DeclareLaunchArgument(
        'state_publish_frequency',
        default_value=default_state_publish_frequency,
        description='State publishing frequency in Hz'
    )

    interval_arg = DeclareLaunchArgument(
        'min_cmd_interval',
        default_value=default_min_cmd_interval,
        description='Minimum interval between commands'
    )

    enable_noise_arg = DeclareLaunchArgument(
        'enable_noise',
        default_value=default_enable_noise,
        description='Enable noise simulation'
    )

    gaussian_stddev_arg = DeclareLaunchArgument(
        'gaussian_stddev',
        default_value=default_gaussian_stddev,
        description='Standard deviation for Gaussian noise'
    )

    failure_probability_arg = DeclareLaunchArgument(
        'failure_probability',
        default_value=default_failure_probability,
        description='Probability of localization failure'
    )

    failure_max_deviation_arg = DeclareLaunchArgument(
        'failure_max_deviation',
        default_value=default_failure_max_deviation,
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