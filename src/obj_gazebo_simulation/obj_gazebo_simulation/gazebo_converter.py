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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class RobotControlConverter(Node):
    """
    Node to convert between RobotNode control format and Gazebo format.
    - Converts RobotToGazeboCmd messages to Twist messages for Gazebo
    - Converts Odometry messages from Gazebo to GazeboToRobotState
    - Publishes state information when control commands are received
    """
    
    def __init__(self):
        super().__init__('robot_control_converter')
        
        # Declare parameters
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('command_timeout', 0.05)  # 50ms timeout for high frequency control
        self.declare_parameter('state_publish_frequency', 50.0)  # 50Hz state publish frequency
        self.declare_parameter('min_cmd_interval', 0.0)  # Minimum interval between commands (0 = no limit)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.state_publish_frequency = self.get_parameter('state_publish_frequency').value
        self.min_cmd_interval = self.get_parameter('min_cmd_interval').value
        
        self.get_logger().info(f'Starting converter for robot_{self.robot_id} at {self.state_publish_frequency}Hz')
        
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
        
        # Initialize robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
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
        
        # Stats for monitoring
        self.command_count = 0
        self.timeout_count = 0
        self.stats_timer = self.create_timer(
            5.0,  # Report stats every 5 seconds
            self.report_stats,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info(f'Converter for robot_{self.robot_id} initialized')
    
    def report_stats(self):
        """Report statistics on command processing"""
        if self.command_count > 0:
            timeout_percent = (self.timeout_count / self.command_count) * 100
            self.get_logger().info(f'Commands: {self.command_count}, Timeouts: {self.timeout_count} ({timeout_percent:.1f}%)')
    
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
                finally:
                    self.state_lock.release()
        except Exception as e:
            self.get_logger().error(f'Error in publish_state: {str(e)}')
    
    
    def send_command_callback(self, request, response):
        """
        Service callback to send a command to Gazebo and wait for state update.

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
            # First, publish command and set timestamp
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
                            response.x = self.current_x
                            response.y = self.current_y
                            response.theta = self.current_theta
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

        Args:
            msg (Odometry): Odometry message with pose and twist
        """
        try:
            # Extract orientation (yaw/theta)
            quat = msg.pose.pose.orientation
            # Convert quaternion to euler angles
            euler_angles = euler.quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')
            
            # Update timestamp before attempting to acquire lock
            now = self.get_clock().now()
            now_ns = now.nanoseconds
            
            # Try to acquire lock with very short timeout - don't block if lock is busy
            # This makes the odom callback non-blocking, preventing backlog in high-frequency scenarios
            if self.state_lock.acquire(timeout=0.002):
                try:
                    # Update current state
                    self.current_x = msg.pose.pose.position.x
                    self.current_y = msg.pose.pose.position.y
                    self.current_theta = euler_angles[2]
                    self.last_update_time = now
                    self.last_update_time_ns = now_ns
                    
                    # Always notify waiting threads - they will check the condition themselves
                    self.state_cv.notify_all()
                finally:
                    self.state_lock.release()
            else:
                # Skip this update if lock is busy - this prevents odometry processing backlog
                # in high-frequency scenarios
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