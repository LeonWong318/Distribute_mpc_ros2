import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from typing import Any, Dict, List, Optional
from robot_interfaces.msg import (
    RobotState, RobotCommand, Schedule, RobotStatus,
    PlanningResult, CollisionWarning
)
from geometry_msgs.msg import Pose2D, Point
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # 回调组
        self.state_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        
        # 状态存储
        self.robot_states: Dict[int, RobotState] = {}
        self.robot_status: Dict[int, RobotStatus] = {}
        self.active_robots: List[int] = []
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_robots', 10),
                ('safety_distance', 0.5),
                ('check_frequency', 10.0)
            ]
        )
        
        # QoS配置
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # 创建发布者字典
        self.command_publishers: Dict[int, Any] = {}
        self.schedule_publishers: Dict[int, Any] = {}
        self.collision_publisher = self.create_publisher(
            CollisionWarning,
            '/collision_warning',
            10
        )
        
        # 创建定时器进行周期性检查
        self.create_timer(
            1.0/self.get_parameter('check_frequency').value,
            self.periodic_check,
            callback_group=self.timer_callback_group
        )
        
    def register_robot(self, robot_id: int) -> None:
        """注册新的机器人"""
        if robot_id in self.active_robots:
            self.get_logger().warn(f'Robot {robot_id} already registered')
            return
            
        # 创建该机器人的发布者
        self.command_publishers[robot_id] = self.create_publisher(
            RobotCommand,
            f'/robot_{robot_id}/command',
            10
        )
        
        self.schedule_publishers[robot_id] = self.create_publisher(
            Schedule,
            f'/robot_{robot_id}/schedule',
            10
        )
        
        # 创建该机器人的状态订阅者
        self.create_subscription(
            RobotState,
            f'/robot_{robot_id}/state',
            lambda msg: self.state_callback(msg, robot_id),
            10,
            callback_group=self.state_callback_group
        )
        
        # 创建该机器人的状态订阅者
        self.create_subscription(
            RobotStatus,
            f'/robot_{robot_id}/status',
            lambda msg: self.status_callback(msg, robot_id),
            10,
            callback_group=self.state_callback_group
        )
        
        self.active_robots.append(robot_id)
        self.get_logger().info(f'Robot {robot_id} registered successfully')
        
    def state_callback(self, msg: RobotState, robot_id: int) -> None:
        """处理机器人状态更新"""
        self.robot_states[robot_id] = msg
        
    def status_callback(self, msg: RobotStatus, robot_id: int) -> None:
        """处理机器人状态反馈"""
        self.robot_status[robot_id] = msg
        
    def send_command(self, robot_id: int, linear_vel: float, angular_vel: float, 
                    priority: int = 0) -> None:
        """发送控制命令到指定机器人"""
        if robot_id not in self.active_robots:
            self.get_logger().warn(f'Robot {robot_id} not registered')
            return
            
        msg = RobotCommand()
        msg.robot_id = robot_id
        msg.linear_velocity = linear_vel
        msg.angular_velocity = angular_vel
        msg.priority = priority
        msg.stamp = self.get_clock().now().to_msg()
        
        self.command_publishers[robot_id].publish(msg)
        
    def assign_task(self, robot_id: int, waypoints: List[tuple], 
                   scheduled_times: Optional[List[float]] = None) -> None:
        """为指定机器人分配任务"""
        if robot_id not in self.active_robots:
            self.get_logger().warn(f'Robot {robot_id} not registered')
            return
            
        if robot_id not in self.robot_states:
            self.get_logger().warn(f'No state information for robot {robot_id}')
            return
            
        msg = Schedule()
        msg.robot_id = robot_id
        
        # 设置起始位置
        current_state = self.robot_states[robot_id]
        msg.start_pose = current_state.pose
        
        # 设置目标位置
        msg.goal_pose = Pose2D()
        msg.goal_pose.x = float(waypoints[-1][0])
        msg.goal_pose.y = float(waypoints[-1][1])
        
        # 设置路径点
        for x, y in waypoints:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            msg.waypoints.append(point)
            
        # 设置时间戳（如果提供）
        if scheduled_times:
            msg.timestamps = scheduled_times
            
        msg.stamp = self.get_clock().now().to_msg()
        
        self.schedule_publishers[robot_id].publish(msg)
        
    def periodic_check(self) -> None:
        """周期性检查所有机器人的状态"""
        self._check_collisions()
        self._check_task_completion()
        self._check_robot_health()
        
    def _check_collisions(self) -> None:
        """检查机器人之间的潜在碰撞"""
        safety_distance = self.get_parameter('safety_distance').value
        
        for id1 in self.active_robots:
            if id1 not in self.robot_states:
                continue
                
            state1 = self.robot_states[id1]
            pos1 = np.array([state1.pose.x, state1.pose.y])
            
            for id2 in self.active_robots:
                if id2 <= id1 or id2 not in self.robot_states:
                    continue
                    
                state2 = self.robot_states[id2]
                pos2 = np.array([state2.pose.x, state2.pose.y])
                
                distance = np.linalg.norm(pos1 - pos2)
                
                if distance < safety_distance:
                    # 发布碰撞警告
                    warning = CollisionWarning()
                    warning.robot_id = id1
                    warning.potential_collision_ids = [id2]
                    warning.collision_point.x = (pos1[0] + pos2[0]) / 2
                    warning.collision_point.y = (pos1[1] + pos2[1]) / 2
                    warning.time_to_collision = distance / max(0.1, abs(state1.velocity.linear.x))
                    warning.severity = 1 if distance < safety_distance/2 else 0
                    warning.stamp = self.get_clock().now().to_msg()
                    
                    self.collision_publisher.publish(warning)
                    
                    # 发送停止命令
                    self.send_command(id1, 0.0, 0.0, priority=1)
                    self.send_command(id2, 0.0, 0.0, priority=1)
                    
    def _check_task_completion(self) -> None:
        """检查任务完成情况"""
        for robot_id in self.active_robots:
            if robot_id not in self.robot_status:
                continue
                
            status = self.robot_status[robot_id]
            if status.completion_percentage >= 100.0 and not status.is_error:
                self.get_logger().info(f'Robot {robot_id} completed its task')
                
    def _check_robot_health(self) -> None:
        """检查机器人健康状态"""
        current_time = self.get_clock().now()
        
        for robot_id in self.active_robots:
            if robot_id not in self.robot_states:
                continue
                
            state = self.robot_states[robot_id]
            state_age = (current_time - rclpy.time.Time.from_msg(state.stamp)).nanoseconds / 1e9
            
            if state_age > 1.0:  # 如果状态更新超过1秒
                self.get_logger().warn(f'No recent state update from robot {robot_id}')

def main(args=None):
    rclpy.init(args=args)
    
    manager = RobotManager()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()