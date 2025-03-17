#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from msg_interfaces.msg import GazeboToManagerState
from msg_interfaces.srv import ExecuteCommand
from transforms3d import euler
import threading
import time
import random
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class RobotControlConverter(Node):
    """
    Node to convert between RobotNode control format and Gazebo format.
    - Converts RobotToGazeboCmd messages to Twist messages for Gazebo
    - Converts Odometry messages from Gazebo to GazeboToRobotState
    - Publishes state information when control commands are received
    - Can simulate noise in state information for testing localization errors
    """
    
    def __init__(self):
        super().__init__('robot_control_converter')
        
        # Declare parameters
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('command_timeout', 0.05)  # 50ms timeout for high frequency control
        self.declare_parameter('state_publish_frequency', 50.0)  # 50Hz state publish frequency
        self.declare_parameter('min_cmd_interval', 0.0)  # Minimum interval between commands (0 = no limit)
        
        # Noise simulation parameters
        self.declare_parameter('enable_noise', False)  # Enable/disable noise simulation
        self.declare_parameter('gaussian_stddev', 0.05)  # Standard deviation for Gaussian noise (meters/radians)
        self.declare_parameter('failure_probability', 0.05)  # Probability of complete failure (0.05 = 5%)
        self.declare_parameter('failure_max_deviation', 1.0)  # Maximum deviation on failure (meters)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.state_publish_frequency = self.get_parameter('state_publish_frequency').value
        self.min_cmd_interval = self.get_parameter('min_cmd_interval').value
        
        # Get noise parameters
        self.enable_noise = self.get_parameter('enable_noise').value
        self.gaussian_stddev = self.get_parameter('gaussian_stddev').value
        self.failure_probability = self.get_parameter('failure_probability').value
        self.failure_max_deviation = self.get_parameter('failure_max_deviation').value
        
        self.get_logger().info(f'Starting converter for robot_{self.robot_id} at {self.state_publish_frequency}Hz')
        if self.enable_noise:
            self.get_logger().info(f'Noise simulation enabled: stddev={self.gaussian_stddev}, '
                                  f'failure_prob={self.failure_probability}, '
                                  f'max_dev={self.failure_max_deviation}')
        
        # Use separate callback groups for better performance
        self.odom_callback_group = ReentrantCallbackGroup()
        self.command_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create QoS profiles for reliable communication
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry from Gazebo - with high priority
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/robot_{self.robot_id}/odom',
            self.odom_callback,
            self.reliable_qos,
            callback_group=self.odom_callback_group
        )
        
        # Publisher to Gazebo for robot control
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel',
            self.reliable_qos
        )
        
        # Publisher for robot state
        self.state_pub = self.create_publisher(
            GazeboToManagerState,
            f'/robot_{self.robot_id}/sim_state',
            self.reliable_qos
        )
        
        # Publisher for robot state without noise
        self.real_state_pub = self.create_publisher(
            GazeboToManagerState,
            f'/robot_{self.robot_id}/real_state',
            self.reliable_qos
        )
        
        # Initialize robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.true_x = 0.0  # Store true positions for reference
        self.true_y = 0.0
        self.true_theta = 0.0
        self.last_update_time = self.get_clock().now()
        self.last_update_time_ns = self.last_update_time.nanoseconds
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_time_ns = self.last_cmd_time.nanoseconds
        self.last_cmd_sent_time = self.get_clock().now()
        
        # Use locks with polling for thread synchronization (more efficient for high frequency)
        self.state_lock = threading.RLock()
        self.state_cv = threading.Condition(self.state_lock)
        
        # Create command service
        self.send_cmd_service = self.create_service(
            ExecuteCommand,
            f'/robot_{self.robot_id}/command',
            self.send_command_callback,
            callback_group=self.command_callback_group
        )
        
        self.state_timer = self.create_timer(
            1.0 / self.state_publish_frequency,
            self.publish_state,
            callback_group=self.timer_callback_group
        )
        
        self.odom_update_count = 0
        self.stats_timer = self.create_timer(
            1.0,  # Report stats every 1 seconds
            self.report_stats,
            callback_group=self.timer_callback_group
        )
        
        # Stats for monitoring
        self.command_count = 0
        self.timeout_count = 0
        self.noise_failure_count = 0
        self.stats_timer = self.create_timer(
            1.0,  # Report stats every 1 seconds
            self.report_stats,
            callback_group=self.timer_callback_group
        )
        
        # Initialize random seed for noise generation
        random.seed()
        np.random.seed()
        
        self.get_logger().info(f'Converter for robot_{self.robot_id} initialized')
    
    def report_stats(self):
        """Report statistics on command processing and noise simulation"""
        if self.command_count > 0:
            timeout_percent = (self.timeout_count / self.command_count) * 100
            message = f'Commands: {self.command_count}, Timeouts: {self.timeout_count} ({timeout_percent:.1f}%)'

            if self.enable_noise and self.odom_update_count > 0:
                failure_percent = (self.noise_failure_count / self.odom_update_count) * 100
                message += f', Odom updates: {self.odom_update_count}, Simulated failures: {self.noise_failure_count} ({failure_percent:.1f}%)'

            self.get_logger().info(message)

    
    def apply_noise_to_state(self, x, y, theta):
        """
        Apply noise to the state variables based on configured parameters.
        
        Args:
            x, y, theta: Original state values from odometry
                
        Returns:
            noisy_x, noisy_y, noisy_theta: State values with noise applied
            true_x, true_y, true_theta: Original state values without noise
        """
        # Always store the original values as true state
        true_x, true_y, true_theta = x, y, theta
        
        # Initialize noisy values with true values
        noisy_x, noisy_y, noisy_theta = x, y, theta
        
        if not self.enable_noise:
            return noisy_x, noisy_y, noisy_theta, true_x, true_y, true_theta
                
        # Determine if we should simulate a complete failure
        if random.random() < self.failure_probability:
            # Complete failure mode - significant deviation
            self.noise_failure_count += 1
            self.get_logger().debug('Simulating localization failure')
            
            # Generate random deviations within the configured range
            x_dev = random.uniform(-self.failure_max_deviation, self.failure_max_deviation)
            y_dev = random.uniform(-self.failure_max_deviation, self.failure_max_deviation)
            theta_dev = random.uniform(-self.failure_max_deviation, self.failure_max_deviation)
            
            # Apply the large deviations
            noisy_x += x_dev
            noisy_y += y_dev
            noisy_theta += theta_dev
            
            # Normalize theta to [-pi, pi]
            noisy_theta = np.arctan2(np.sin(noisy_theta), np.cos(noisy_theta))
        else:
            # Normal Gaussian noise mode
            noisy_x += np.random.normal(0, self.gaussian_stddev)
            noisy_y += np.random.normal(0, self.gaussian_stddev)
            noisy_theta += np.random.normal(0, self.gaussian_stddev)
            
            # Normalize theta to [-pi, pi]
            noisy_theta = np.arctan2(np.sin(noisy_theta), np.cos(noisy_theta))
        
        return noisy_x, noisy_y, noisy_theta, true_x, true_y, true_theta
    
    def publish_state(self):
        """
        Publish the current robot state at the specified frequency.
        """
        try:
            # Use try/finally pattern with explicit lock acquire/release
            # This is more efficient than 'with' statement for high-frequency operations
            if self.state_lock.acquire(timeout=0.005):  # Short timeout to avoid blocking
                try:
                    # Create state message
                    state_msg = GazeboToManagerState()
                    state_msg.robot_id = self.robot_id
                    state_msg.x = self.current_x
                    state_msg.y = self.current_y
                    state_msg.theta = self.current_theta
                    state_msg.stamp = self.last_update_time.to_msg()
                    
                    # Publish state
                    self.state_pub.publish(state_msg)
                    
                    # Create real state message (without noise)
                    real_state_msg = GazeboToManagerState()
                    real_state_msg.robot_id = self.robot_id
                    real_state_msg.x = self.true_x 
                    real_state_msg.y = self.true_y
                    real_state_msg.theta = self.true_theta
                    real_state_msg.stamp = self.last_update_time.to_msg()
                    
                    # Publish real state
                    self.real_state_pub.publish(real_state_msg)
                    
                finally:
                    self.state_lock.release()
        except Exception as e:
            self.get_logger().error(f'Error in publish_state: {str(e)}')
    
    def send_command_callback(self, request, response):
        """
        Service callback to send a command to Gazebo and wait for state update.
        Now uses real state values (without noise) to fill the response.

        Args:
            request: SendCommand.Request with v and omega fields
            response: SendCommand.Response with success field

        Returns:
            response: Filled response with robot state
        """
        # Apply command throttling if configured
        if self.min_cmd_interval > 0:
            current_time = self.get_clock().now()
            time_since_last_cmd = (current_time.nanoseconds - self.last_cmd_sent_time.nanoseconds) / 1e9
            if time_since_last_cmd < self.min_cmd_interval:
                self.get_logger().debug(f"Command throttled ({time_since_last_cmd:.4f}s < {self.min_cmd_interval:.4f}s)")
                time.sleep(self.min_cmd_interval - time_since_last_cmd)
            self.last_cmd_sent_time = self.get_clock().now()
            
        self.command_count += 1

        try:
            # Create Twist message for Gazebo
            twist_msg = Twist()
            twist_msg.linear.x = request.v
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = request.omega
            
            # Get command time before any locks
            cmd_time = self.get_clock().now()
            cmd_time_ns = cmd_time.nanoseconds
            
            # Use polling approach for high-frequency operation
            if not self.state_lock.acquire(timeout=0.01):
                self.get_logger().error("Failed to acquire state_lock for command - skipping")
                response.success = False
                response.stamp = self.get_clock().now().to_msg()
                return response
                
            try:
                # Record time before sending command
                self.last_cmd_time = cmd_time
                self.last_cmd_time_ns = cmd_time_ns
                
                # Publish to Gazebo - still within lock to ensure timestamp is set before publishing
                self.cmd_vel_pub.publish(twist_msg)
            finally:
                self.state_lock.release()
            
            # Very short sleep to allow scheduler to run odom callback if needed
            time.sleep(0.001)
            
            # Now poll for state update with short timeouts
            start_wait = self.get_clock().now()
            poll_interval = 0.001  # 1ms poll interval
            max_wait = self.command_timeout
            
            # Use polling instead of condition variable for more predictable behavior
            success = False
            
            while (self.get_clock().now().nanoseconds - start_wait.nanoseconds) / 1e9 < max_wait:
                if self.state_lock.acquire(timeout=0.005):
                    try:
                        if self.last_update_time_ns >= cmd_time_ns:
                            # State is updated
                            response.success = True
                            # Use real state values (without noise)
                            response.x = self.true_x
                            response.y = self.true_y
                            response.theta = self.true_theta
                            response.stamp = self.get_clock().now().to_msg()
                            success = True
                            break
                    finally:
                        self.state_lock.release()
                        
                # Short sleep before checking again
                time.sleep(poll_interval)
            
            if not success:
                self.timeout_count += 1
                self.get_logger().warn(f'Timeout waiting for state update. Last update: {self.last_update_time_ns}, cmd: {cmd_time_ns}')
                response.success = False
                response.stamp = self.get_clock().now().to_msg()
            
            return response

        except Exception as e:
            self.get_logger().error(f'Error in send_command_callback: {str(e)}')
            try:
                if self.state_lock._is_owned():
                    self.state_lock.release()
            except:
                pass
                
            response.success = False
            response.stamp = self.get_clock().now().to_msg()
            return response

    def odom_callback(self, msg):
        """
        Process odometry data from Gazebo.
        Now stores both noisy and real state values separately.

        Args:
            msg (Odometry): Odometry message with pose and twist
        """
        try:
            # Extract orientation (yaw/theta)
            quat = msg.pose.pose.orientation
            # Convert quaternion to euler angles
            euler_angles = euler.quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')
            
            # Extract position data
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            theta = euler_angles[2]
            
            # Get both noisy and real state values
            noisy_x, noisy_y, noisy_theta, true_x, true_y, true_theta = self.apply_noise_to_state(x, y, theta)
            
            # Update timestamp before attempting to acquire lock
            now = self.get_clock().now()
            now_ns = now.nanoseconds
            
            # Try to acquire lock with very short timeout - don't block if lock is busy
            if self.state_lock.acquire(timeout=0.002):
                try:
                    # Update current state with potentially noisy values
                    self.current_x = noisy_x      # Noisy state
                    self.current_y = noisy_y
                    self.current_theta = noisy_theta
                    
                    # Update real state values (without noise)
                    self.true_x = true_x          # Real state
                    self.true_y = true_y
                    self.true_theta = true_theta
                    
                    self.last_update_time = now
                    self.last_update_time_ns = now_ns
                    self.odom_update_count += 1
                    # Notify waiting threads
                    self.state_cv.notify_all()
                finally:
                    self.state_lock.release()
            else:
                # Skip this update if lock is busy - prevents odometry processing backlog
                pass
                
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')
            try:
                if self.state_lock._is_owned():
                    self.state_lock.release()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create node
        node = RobotControlConverter()
    
        # Use more threads for better parallelism
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        
        try:
            node.get_logger().info('Starting executor...')
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard interrupt, shutting down...')
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f'Error during node execution: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()