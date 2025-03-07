#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from msg_interfaces.srv import ExecuteCommand
from transforms3d import euler
import threading
import time

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
        self.robot_id = self.get_parameter('robot_id').value
        
        self.get_logger().info(f'Starting converter for robot_{self.robot_id}')
        
        # Create QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry from Gazebo
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/robot_{self.robot_id}/odom',
            self.odom_callback,
            10  # QoS depth
        )
        
        # Publisher to Gazebo for robot control
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel',  # Direct to Gazebo
            10
        )
        
        # Initialize robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_update_time = self.get_clock().now()
        self.state_updated = False
        self.state_lock = threading.Lock()
        
        self.send_cmd_service = self.create_service(
            ExecuteCommand,
            f'/robot_{self.robot_id}/command',
            self.send_command_callback
        )
        
        self.get_logger().info(f'Converter for robot_{self.robot_id} initialized')
    
    def send_command_callback(self, request, response):
        """
        Service callback to send a command to Gazebo and wait for state update.
        
        Args:
            request: SendCommand.Request with v and omega fields
            response: SendCommand.Response with success field
        """
        try:
            # Create Twist message for Gazebo
            twist_msg = Twist()
            
            # Extract v and omega from request
            twist_msg.linear.x = request.v  # Linear velocity
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = request.omega  # Angular velocity
            
            # Reset state update flag
            with self.state_lock:
                self.state_updated = False
            
            # Record time before sending command
            cmd_time = self.get_clock().now()
            
            # Publish to Gazebo
            self.cmd_vel_pub.publish(twist_msg)
            
            self.get_logger().debug(f'Sent command: v={twist_msg.linear.x:.2f}, ω={twist_msg.angular.z:.2f}')
            
            # Wait for state update with timeout
            timeout = 0.5  # 500 ms timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                with self.state_lock:
                    if self.state_updated and self.last_update_time > cmd_time:
                        response.success = True
                        response.x = self.current_x
                        response.y = self.current_y
                        response.theta = self.current_theta
                        return response
                time.sleep(0.01)  # Short sleep to prevent CPU hogging
            
            # If timeout occurred
            self.get_logger().warn(f'Timeout waiting for state update after command')
            response.success = False
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error in send_command_callback: {str(e)}')
            response.success = False
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
            # Convert quaternion to euler angles using transforms3d
            euler_angles = euler.quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')
            with self.state_lock:
                self.current_x = msg.pose.pose.position.x
                self.current_y = msg.pose.pose.position.y
                self.current_theta = euler_angles[2]
                self.last_update_time = self.get_clock().now()
                self.state_updated = True
            
            self.get_logger().debug(f'Updated state: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={self.current_theta:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()