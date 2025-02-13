import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
from geometry_msgs.msg import Pose2D, Twist
from robot_interfaces.msg import RobotState, RobotCommand, Schedule, RobotStatus, GlobalMapData

from configs import MpcConfiguration, CircularRobotSpecification

from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('max_velocity', 1.0),
                ('max_angular_velocity', 1.0),
                ('control_frequency', 10.0),
                ('mpc_config_path', ''),
                ('robot_config_path', '')
            ]
        )
        
        self.pred_states = None
        self.last_actions = None
        self.current_refs = None
        self.debug_info = None
        
        # 获取基本参数
        self.robot_id = self.get_parameter('robot_id').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # 加载配置文件
        mpc_config_path = self.get_parameter('mpc_config_path').value
        robot_config_path = self.get_parameter('robot_config_path').value
        self.config_mpc = MpcConfiguration.from_yaml(mpc_config_path)
        self.config_robot = CircularRobotSpecification.from_yaml(robot_config_path)
        
        
        # 等待并获取地图数据
        self.get_logger().info('Waiting for map data...')
        while True:
            try:
                map_client = self.create_client(GlobalMapData, '/request_map_data')
                while not map_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Map data service not available, waiting...')

                request = GlobalMapData.Request()
                request.robot_id = self.robot_id

                future = map_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)

                if future.result() is None:
                    raise RuntimeError('Service call failed')

                # 保存地图数据为成员变量
                response = future.result()
                self.map_json = response.map_json
                self.graph_json = response.graph_json
                self.schedule_json = response.schedule_json
                self.robot_start = response.robot_start

                self.get_logger().info('Successfully received map data')
                break

            except Exception as e:
                self.get_logger().error(f'Failed to get map data: {str(e)}')
                self.get_logger().info('Retrying in 1 second...')
                time.sleep(1.0)
        
        # 初始化控制组件
        self.setup_control_components()
        
        #初始化全局路径协调器GPC
        self.gpc = GlobalPathCoordinator.from_csv_string(self.schedule_json)
        self.gpc.load_graph_from_json_string(self.graph_json)
        self.gpc.load_map_from_json_string(self.graph_json,inflation_margin=self.config_robot.vehicle_width+0.2)
        robot_ids = self.gpc.robot_ids if robot_ids is None else robot_ids
        self.static_obstacles = self.gpc.inflated_map.obstacle_coords_list
        
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
        
        # 创建订阅者获取其他机器人状态
        self.other_robot_states = {}
        for i in range(10):  # 假设最多10个机器人
            if i != self.robot_id:
                self.create_subscription(
                    RobotState,
                    f'/robot_{i}/state',
                    lambda msg, rid=i: self.other_robot_state_callback(msg, rid),
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
        
        # 创建控制定时器
        self.create_timer(
            1.0/self.control_frequency,
            self.control_loop
        )

    def setup_control_components(self):
        """初始化控制相关组件"""
        # 运动模型
        self.motion_model = UnicycleModel(sampling_time=self.config_mpc.ts)

        # 轨迹规划器
        self.planner = LocalTrajPlanner(
            self.config_mpc.ts,
            self.config_mpc.N_hor,
            self.config_robot.lin_vel_max
        )
        
        self.planner.load_map(self.gpc.inflated_map.boundary_coords, self.gpc.inflated_map.obstacle_coords_list)

        # 轨迹跟踪控制器
        self.controller = TrajectoryTracker(
            self.config_mpc,
            self.config_robot,
            robot_id=self.robot_id
        )
        self.controller.load_motion_model(self.motion_model)
    
    def other_robot_state_callback(self, msg: RobotState, robot_id: int):
        """处理其他机器人的状态信息"""
        # 更新其他机器人状态字典
        self.other_robot_states[robot_id] = msg
    
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
        
        # 根据物理约束计算标称速度
        nominal_speed = self.config_robot.lin_vel_max * 0.8  # 使用最大速度的80%作为标称速度
        
        # 使用LocalTrajPlanner的load_path加载路径
        self.planner.load_path(
            path_coords=msg.path_coords,
            path_times=msg.path_times,
            nomial_speed=nominal_speed,
            method='linear'  # 或者使用'time'，取决于你的需求
        )
        
        # 发布状态更新
        self.publish_status()
    
    def control_loop(self):
        """主控制循环"""
        try:
            if self.planner.idle:
                return
                
            # 获取当前时间
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time = current_time[0] + current_time[1] * 1e-9
            
            # 获取局部参考轨迹
            current_pos = (self.current_state['x'], self.current_state['y'])
            ref_states, ref_speed, done = self.planner.get_local_ref(
                current_time=current_time,
                current_pos=current_pos,
                idx_check_range=10  # 可以通过参数配置
            )
            
            if done:
                self.is_idle = True
                self.current_task = "idle"
                self.publish_status()
                return
                
            # 设置参考轨迹
            self.controller.set_ref_states(ref_states, ref_speed=ref_speed)
            
            # 获取其他机器人状态列表
            other_robot_states = [state for rid, state in self.other_robot_states.items()]
            
            # 运行控制器
            self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
                static_obstacles=self.static_obstacles,
                other_robot_states=other_robot_states
            )
            
            # 执行控制动作
            self.execute_control(self.last_actions[-1])
            
            # 更新并发布状态
            self.update_and_publish_state()
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    
    def execute_control(self, action):
        """执行控制动作"""
        dt = 1.0/self.control_frequency
        
        # 更新状态
        self.current_state['x'] += self.current_state['v_x'] * dt
        self.current_state['y'] += self.current_state['v_y'] * dt
        self.current_state['theta'] += self.current_state['omega'] * dt
        
        # 标准化角度
        self.current_state['theta'] = np.arctan2(
            np.sin(self.current_state['theta']),
            np.cos(self.current_state['theta'])
        )
    
    def update_and_publish_state(self):
        """更新并发布机器人状态"""
        try:
            state_msg = RobotState()
            state_msg.robot_id = self.robot_id
            
            # 设置位姿
            if hasattr(state_msg, 'pose'):
                state_msg.pose = Pose2D()
                state_msg.pose.x = float(self.current_state['x'])
                state_msg.pose.y = float(self.current_state['y'])
                state_msg.pose.theta = float(self.current_state['theta'])
            
            # 设置速度
            if hasattr(state_msg, 'velocity'):
                state_msg.velocity = Twist()
                state_msg.velocity.linear.x = float(self.current_state['v_x'])
                state_msg.velocity.linear.y = float(self.current_state['v_y'])
                state_msg.velocity.angular.z = float(self.current_state['omega'])
            
            # 设置预测状态序列
            if hasattr(state_msg, 'predicted_states') and self.pred_states is not None:
                state_msg.predicted_states = self.pred_states.flatten().tolist()
            
            # 设置其他字段
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
            status_msg.status_code = 0
            status_msg.status_message = self.current_task
            status_msg.is_error = False
            status_msg.completion_percentage = self.planner.get_completion_percentage() if hasattr(self.planner, 'get_completion_percentage') else 0.0
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