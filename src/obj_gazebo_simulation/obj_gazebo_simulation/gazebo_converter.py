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
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.declare_parameter('command_timeout', 1.0)  # Default timeout in seconds
        self.declare_parameter('state_publish_frequency', 50.0)  # Default 50Hz state publish frequency
        
        self.robot_id = self.get_parameter('robot_id').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.state_publish_frequency = self.get_parameter('state_publish_frequency').value
        
        self.get_logger().info(f'Starting converter for robot_{self.robot_id}')
        
        # Use reentrant callback group for nested callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create QoS profiles for reliable communication
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry from Gazebo
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/robot_{self.robot_id}/odom',
            self.odom_callback,
            self.reliable_qos,
            callback_group=self.callback_group
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
        self.last_cmd_time = self.get_clock().now()
        
        # Use condition variable for thread synchronization
        self.state_lock = threading.RLock()
        self.state_cv = threading.Condition(self.state_lock)
        
        # Create command service
        self.send_cmd_service = self.create_service(
            ExecuteCommand,
            f'/robot_{self.robot_id}/command',
            self.send_command_callback,
            callback_group=self.callback_group
        )
        
        self.state_timer = self.create_timer(
            1.0 / self.state_publish_frequency,
            self.publish_state,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f'Converter for robot_{self.robot_id} initialized')
    
    def publish_state(self):
        """
        Publish the current robot state at the specified frequency.
        """
        try:
            with self.state_lock:
                # Create state message
                state_msg = GazeboToManagerState()
                state_msg.robot_id = self.robot_id
                state_msg.x = self.current_x
                state_msg.y = self.current_y
                state_msg.theta = self.current_theta
                state_msg.stamp = self.last_update_time.to_msg()
                
                # Publish state
                self.state_pub.publish(state_msg)
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
        self.get_logger().info(f"Starting send_command_callback with v={request.v}, omega={request.omega}")

        try:
            # Create Twist message for Gazebo
            twist_msg = Twist()
            twist_msg.linear.x = request.v
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = request.omega

            # Set command time before acquiring lock to avoid potential deadlock
            cmd_time = self.get_clock().now()
            self.get_logger().info(f"Created command at time: {cmd_time.nanoseconds}")

            # First lock acquisition - just for setting command time and publishing
            lock_acquired = self.state_lock.acquire(timeout=0.5)  # Add timeout to prevent deadlock
            if not lock_acquired:
                self.get_logger().error("Failed to acquire state_lock in 0.5s - potential deadlock")
                response.success = False
                response.stamp = self.get_clock().now().to_msg()
                return response

            try:
                # Record time before sending command
                self.last_cmd_time = cmd_time
                self.get_logger().info(f"Set last_cmd_time to {self.last_cmd_time.nanoseconds}")

                # Publish to Gazebo
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info("Published twist message to Gazebo")
            finally:
                # Always release lock
                self.state_lock.release()
                self.get_logger().info("Released state_lock after publishing command")

            # Give odometry callback a chance to process
            time.sleep(0.01)

            # Second lock acquisition - for checking state update and waiting if needed
            lock_acquired = self.state_lock.acquire(timeout=0.5)
            if not lock_acquired:
                self.get_logger().error("Failed to acquire state_lock for state checking - potential deadlock")
                response.success = False
                response.stamp = self.get_clock().now().to_msg()
                return response

            try:
                # Safety check for last_cmd_time
                if self.last_cmd_time is None:
                    self.get_logger().error("last_cmd_time is None before state check - unexpected reset")
                    self.last_cmd_time = cmd_time  # Use local backup

                # Check if state was already updated before waiting
                self.get_logger().info(f"Checking immediate update: update_time={self.last_update_time.nanoseconds}, cmd_time={self.last_cmd_time.nanoseconds}")

                if self.last_update_time.nanoseconds >= self.last_cmd_time.nanoseconds:
                    self.get_logger().info("State already updated, returning immediate success")
                    response.success = True
                    response.x = self.current_x
                    response.y = self.current_y
                    response.theta = self.current_theta
                    response.stamp = self.get_clock().now().to_msg()
                    return response

                # Define condition for state update - using >= instead of > for safer comparison
                def state_updated():
                    updated = (self.last_update_time.nanoseconds >= self.last_cmd_time.nanoseconds)
                    self.get_logger().debug(f"State updated check: {updated}, update_time={self.last_update_time.nanoseconds}, cmd_time={self.last_cmd_time.nanoseconds}")
                    return updated

                # Wait for state update with timeout
                self.get_logger().info(f"Waiting for state update with timeout={self.command_timeout}s")
                result = self.state_cv.wait_for(state_updated, self.command_timeout)

                if result:
                    self.get_logger().info("State update received within timeout")
                    response.success = True
                    response.x = self.current_x
                    response.y = self.current_y
                    response.theta = self.current_theta
                    response.stamp = self.get_clock().now().to_msg()
                else:
                    self.get_logger().warn(f'Timeout waiting for state update. Last update time: {self.last_update_time.nanoseconds}, command time: {self.last_cmd_time.nanoseconds}')
                    response.success = False
                    response.stamp = self.get_clock().now().to_msg()
            finally:
                # Always release lock
                self.state_lock.release()
                self.get_logger().info("Released state_lock after state check/wait")

            return response

        except Exception as e:
            self.get_logger().error(f'Error in send_command_callback: {str(e)}')
            # Make sure lock is released in case of exception
            try:
                if self.state_lock._is_owned():
                    self.state_lock.release()
                    self.get_logger().info("Released lock after exception")
            except Exception as unlock_e:
                self.get_logger().error(f'Error releasing lock: {str(unlock_e)}')

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

            # Add timeout to avoid deadlock
            lock_acquired = self.state_lock.acquire(timeout=0.5)
            if not lock_acquired:
                self.get_logger().error("Failed to acquire state_lock in odom_callback - potential deadlock")
                return

            try:
                # Update current state
                self.current_x = msg.pose.pose.position.x
                self.current_y = msg.pose.pose.position.y
                self.current_theta = euler_angles[2]
                self.last_update_time = self.get_clock().now()

                update_time = self.last_update_time.nanoseconds
                cmd_time = self.last_cmd_time.nanoseconds if self.last_cmd_time is not None else None

                # Debug log
                self.get_logger().debug(f'odom_callback: pos=({self.current_x:.2f},{self.current_y:.2f}), theta={self.current_theta:.2f}')

                # Notify waiting threads if a command was sent
                if self.last_cmd_time is not None:
                    condition = update_time >= cmd_time
                    self.get_logger().debug(f'odom_callback: update_time={update_time}, cmd_time={cmd_time}, notify={condition}')

                    # Always notify - let the waiting thread decide if the condition is satisfied
                    # This prevents potential deadlocks if time comparison has issues
                    self.state_cv.notify_all()
                    self.get_logger().debug('Notified all waiting threads')
            finally:
                # Always release lock
                self.state_lock.release()

        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')
            # Make sure lock is released in case of exception
            try:
                if self.state_lock._is_owned():
                    self.state_lock.release()
                    self.get_logger().info("Released lock after exception in odom_callback")
            except Exception as unlock_e:
                self.get_logger().error(f'Error releasing lock: {str(unlock_e)}')

    
    # def send_command_callback(self, request, response):
    #     """
    #     Service callback to send a command to Gazebo and wait for state update.
        
    #     Args:
    #         request: SendCommand.Request with v and omega fields
    #         response: SendCommand.Response with success field
            
    #     Returns:
    #         response: Filled response with robot state
    #     """
    #     try:
    #         # Create Twist message for Gazebo
    #         twist_msg = Twist()
    #         twist_msg.linear.x = request.v
    #         twist_msg.linear.y = 0.0
    #         twist_msg.linear.z = 0.0
    #         twist_msg.angular.x = 0.0
    #         twist_msg.angular.y = 0.0
    #         twist_msg.angular.z = request.omega
            
    #         with self.state_lock:
    #             # Record time before sending command
    #             self.last_cmd_time = self.get_clock().now()
                
    #             # Publish to Gazebo
    #             self.cmd_vel_pub.publish(twist_msg)
            
    #         # Give odometry callback a chance to process
    #         time.sleep(0.01)
            
    #         with self.state_lock:
    #             # Check if state was already updated before waiting
    #             if (self.last_update_time.nanoseconds > self.last_cmd_time.nanoseconds):
    #                 response.success = True
    #                 response.x = self.current_x
    #                 response.y = self.current_y
    #                 response.theta = self.current_theta
    #                 response.stamp = self.get_clock().now().to_msg()
    #                 return response
                
    #             # Define condition for state update
    #             def state_updated():
    #                 return self.last_update_time.nanoseconds > self.last_cmd_time.nanoseconds
                
    #             # Wait for state update with timeout
    #             result = self.state_cv.wait_for(state_updated, self.command_timeout)
                
    #             if result:
    #                 response.success = True
    #                 response.x = self.current_x
    #                 response.y = self.current_y
    #                 response.theta = self.current_theta
    #                 response.stamp = self.get_clock().now().to_msg()
    #             else:
    #                 self.get_logger().warn('Timeout waiting for state update')
    #                 response.success = False
    #                 response.stamp = self.get_clock().now().to_msg()
            
    #         return response
            
    #     except Exception as e:
    #         self.get_logger().error(f'Error in send_command_callback: {str(e)}')
    #         response.success = False
    #         response.stamp = self.get_clock().now().to_msg()
    #         return response
    
    # def odom_callback(self, msg):
    #     """
    #     Process odometry data from Gazebo.
        
    #     Args:
    #         msg (Odometry): Odometry message with pose and twist
    #     """
    #     try:
    #         # Extract orientation (yaw/theta)
    #         quat = msg.pose.pose.orientation
    #         # Convert quaternion to euler angles
    #         euler_angles = euler.quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')
            
    #         with self.state_lock:
    #             # Update current state
    #             self.current_x = msg.pose.pose.position.x
    #             self.current_y = msg.pose.pose.position.y
    #             self.current_theta = euler_angles[2]
    #             self.last_update_time = self.get_clock().now()
                
    #             # Notify waiting threads if a command was sent
    #             if self.last_cmd_time is not None:
    #                 self.state_cv.notify_all()
            
    #     except Exception as e:
    #         self.get_logger().error(f'Error in odom_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create node
        node = RobotControlConverter()
    
        executor = MultiThreadedExecutor(num_threads=4)
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