import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from robot_interfaces.msg import RobotState, RobotCommand, Schedule, RobotStatus

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        
        # 获取参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('max_velocity', 1.0),
                ('max_angular_velocity', 1.0),
                ('control_frequency', 10.0)
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # 设置QoS
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # 创建发布者
        self.state_publisher = self.create_publisher(
            RobotState, 
            f'/robot_{self.robot_id}/state', 
            state_qos
        )
        
        self.status_publisher = self.create_publisher(
            RobotStatus,
            f'/robot_{self.robot_id}/status',
            10
        )
        
        # 创建订阅者
        self.command_subscription = self.create_subscription(
            RobotCommand,
            f'/robot_{self.robot_id}/command',
            self.command_callback,
            cmd_qos
        )
        
        self.schedule_subscription = self.create_subscription(
            Schedule,
            f'/robot_{self.robot_id}/schedule',
            self.schedule_callback,
            10
        )
        
        # 初始化机器人状态
        self.current_state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'v_x': 0.0,
            'v_y': 0.0,
            'omega': 0.0
        }
        
        self.is_idle = True
        self.current_task = "idle"
        
        # 创建定时器用于状态更新和发布
        self.create_timer(
            1.0/self.get_parameter('control_frequency').value,
            self.update_and_publish_state
        )
        
    def command_callback(self, msg: RobotCommand):
        """处理控制命令"""
        if msg.robot_id != self.robot_id:
            return
            
        # 验证和限制速度命令
        linear_vel = max(min(msg.linear_velocity, self.max_velocity), -self.max_velocity)
        angular_vel = max(min(msg.angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)
        
        # 更新机器人状态
        self.current_state['v_x'] = linear_vel * np.cos(self.current_state['theta'])
        self.current_state['v_y'] = linear_vel * np.sin(self.current_state['theta'])
        self.current_state['omega'] = angular_vel
        
    def schedule_callback(self, msg: Schedule):
        """处理调度任务"""
        if msg.robot_id != self.robot_id:
            return
            
        self.is_idle = False
        self.current_task = "executing_schedule"
        
        # 发布状态更新
        self.publish_status()
        
    def update_and_publish_state(self):
        """更新并发布机器人状态"""
        try:
            # 更新位置和姿态
            dt = 1.0/self.get_parameter('control_frequency').value
            
            self.current_state['x'] += self.current_state['v_x'] * dt
            self.current_state['y'] += self.current_state['v_y'] * dt
            self.current_state['theta'] += self.current_state['omega'] * dt
            
            # 标准化角度到[-pi, pi]
            self.current_state['theta'] = np.arctan2(np.sin(self.current_state['theta']), 
                                                   np.cos(self.current_state['theta']))
            
            # 创建并发布状态消息
            state_msg = RobotState()
            
            # 先尝试获取消息结构
            self.get_logger().debug(f"Available fields: {dir(state_msg)}")
            
            # 基本信息
            state_msg.robot_id = self.robot_id
            
            # 检查并设置位姿
            if hasattr(state_msg, 'x'):
                state_msg.x = float(self.current_state['x'])
                state_msg.y = float(self.current_state['y'])
                state_msg.theta = float(self.current_state['theta'])
            elif hasattr(state_msg, 'pose'):
                state_msg.pose = Pose2D()
                state_msg.pose.x = float(self.current_state['x'])
                state_msg.pose.y = float(self.current_state['y'])
                state_msg.pose.theta = float(self.current_state['theta'])
            
            # 检查并设置速度
            if hasattr(state_msg, 'velocity'):
                state_msg.velocity = Twist()
                state_msg.velocity.linear.x = float(self.current_state['v_x'])
                state_msg.velocity.linear.y = float(self.current_state['v_y'])
                state_msg.velocity.angular.z = float(self.current_state['omega'])
            
            # 设置其他可选字段
            if hasattr(state_msg, 'is_idle'):
                state_msg.is_idle = self.is_idle
            if hasattr(state_msg, 'current_task'):
                state_msg.current_task = self.current_task
            if hasattr(state_msg, 'stamp'):
                state_msg.stamp = self.get_clock().now().to_msg()
            
            self.state_publisher.publish(state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in update_and_publish_state: {str(e)}')
        
    def publish_status(self):
        """发布机器人状态信息"""
        try:
            status_msg = RobotStatus()
            status_msg.robot_id = self.robot_id
            status_msg.status_code = 0  # 正常状态
            status_msg.status_message = self.current_task
            status_msg.is_error = False
            status_msg.completion_percentage = 0.0  # 需要根据实际任务进度更新
            status_msg.stamp = self.get_clock().now().to_msg()
            
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error in publish_status: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()