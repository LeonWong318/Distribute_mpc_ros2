import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import json
from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification

from pkg_moving_object.moving_object import RobotObject
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator

from robot_interfaces.msg import RobotState, RobotStatesQuery
from robot_interfaces.srv import GetMapData

class RobotNode(Node):
    async def initialize(self):
        """异步初始化方法"""
        self.get_logger().info('Starting initialization...')
    
        # 等待服务可用
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, waiting...')
    
        # 获取地图数据
        self.get_logger().info('Requesting map data...')
        await self.get_map_data()
        self.get_logger().info('Requesting map data finished')
    
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
        
        # 设置初始状态
        self.start = None
        self.goal = None
        self.idle = True
        self.pred_states = None
        
        # 设置QoS
        DEFAULT_QOS = 10
        
        # 创建发布者发布机器人状态
        self.state_publisher = self.create_publisher(
            RobotState, 
            f'/robot_{self.robot_id}/state', 
            DEFAULT_QOS
        )
        
        # 创建订阅者获取其他机器人状态
        self.other_robot_states = {}
        self.last_states_update_time = self.get_clock().now()
        
        # 创建订阅者接收manager发布的机器人状态信息
        self.states_query_subscription = self.create_subscription(
            RobotStatesQuery,
            '/manager/robot_states',
            self.robot_states_callback,
            DEFAULT_QOS
        )
        
        # 创建一个回调组用于服务客户端
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建客户端
        self.map_client = self.create_client(
            GetMapData,
            '/get_map_data',
            callback_group=self.callback_group
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
    
    def add_schedule(self):
        self.planner.load_path(self.path_coords,self.path_times,nomial_speed=self.config_robot.lin_vel_max, method="linear")

        goal_coord = self.path_coords[-1]
        goal_coord_prev = self.path_coords[-2]
        goal_heading = np.arctan2(goal_coord[1]-goal_coord_prev[1], goal_coord[0]-goal_coord_prev[0])
        self.goal_state = np.array([*goal_coord, goal_heading])

        self.idle = False
        self.controller.load_init_states(self._state, self.goal_state)
    
    def robot_states_callback(self, msg: RobotStatesQuery):
        """处理来自manager的机器人状态信息"""
        try:
            # 检查消息是否已过期
            current_time = self.get_clock().now()
            msg_time = Time.from_msg(msg.stamp)
            time_diff = (current_time - msg_time).nanoseconds / 1e9  # 转换为秒
            
            if time_diff > 1.0:  # 如果数据超过1秒则认为过期
                self.get_logger().warn('Received outdated robot states')
                return
                
            # 更新其他机器人状态字典
            self.other_robot_states.clear()
            for state in msg.robot_states:
                if state.robot_id != self.robot_id:
                    self.other_robot_states[state.robot_id] = state
            
            self.last_states_update_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error processing robot states: {str(e)}')
            rclpy.shutdown()
    
    async def get_map_data(self):
        """请求地图数据"""
        try:
            # 创建请求
            request = GetMapData.Request()
            request.robot_id = self.robot_id
            
            # 发送请求
            future = self.map_client.call_async(request)
            
            # 等待响应
            response = await future
                        
            # 处理响应数据
            self.map_json = response.map_json
            self.graph_json = response.graph_json
            self.schedule_json = response.schedule_json
            self.robot_start = json.loads(response.robot_start)
            
            self.get_logger().info('Successfully received map data')
            
            self.initialize_after_map_data()
            
        except Exception as e:
            self.get_logger().error(f'Error getting map data: {str(e)}')
    
    def initialize_after_map_data(self):
        """初始化地图数据后的设置"""
        try:
            # 初始化全局路径协调器GPC
            self.gpc = GlobalPathCoordinator.from_csv_string(self.schedule_json)
            self.gpc.load_graph_from_json_string(self.graph_json)
            self.gpc.load_map_from_json_string(
                self.map_json,
                inflation_margin=self.config_robot.vehicle_width+0.2
            )
            self.static_obstacles = self.gpc.inflated_map.obstacle_coords_list
            self.path_coords, self.path_times = self.gpc.get_robot_schedule(self.robot_id)

            # 初始化控制组件
            self.setup_control_components()
            
            # 初始化机器人状态
            self.set_state(np.asarray(self.robot_start[str(self.robot_id)]))

            # 添加调度
            self.add_schedule()

            # 创建控制定时器
            self.create_timer(
                1.0/self.control_frequency,
                self.control_loop
            )

            self.get_logger().info('Initialization completed successfully')
            return True

        except Exception as e:
            self.get_logger().error(f'Error in initialize_after_map_data: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return False

    
    def control_loop(self):
        """主控制循环"""
        try:
            if self.planner.idle:
                return
                
            # 获取当前时间
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time = current_time[0] + current_time[1] * 1e-9
            
            # 获取局部参考轨迹
            current_pos = (self._state[0], self._state[1])
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
            
            # TODO： 解决parameters长度不匹配的问题
            # 运行控制器
            self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
                static_obstacles=self.static_obstacles,
                other_robot_states=other_robot_states
            )
            
            # 执行控制动作
            self.step(self.last_actions[-1])
            
            self.publish_state()
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    

    def set_state(self, state: np.ndarray) -> None:
        self._state = state
        self.robot_object = RobotObject(state=state, ts=self.config_robot.ts, radius=self.config_robot.vehicle_width/2)
        self.robot_object.motion_model = self.motion_model
        
    def step(self, action: np.ndarray) -> None:
        self.robot_object.one_step(action)
        self._state = self.robot_object.state
    
    def publish_state(self):
        """发布机器人状态"""
        try:
            state_msg = RobotState()
            state_msg.robot_id = self.robot_id
            state_msg.x = self._state[0]
            state_msg.y = self._state[1]
            state_msg.theta = self._state[2]
            state_msg.is_idle = self.is_idle
            state_msg.stamp = self.get_clock().now().to_msg()
            
            if self.pred_states is not None:
                state_msg.pred_states = self.pred_states.flatten().tolist()
                
            self.state_publisher.publish(state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in update_and_publish_state: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    
    node = RobotNode()
    executor.add_node(node)
    
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    import asyncio
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.initialize())
    
    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

