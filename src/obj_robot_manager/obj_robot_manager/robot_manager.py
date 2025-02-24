import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from threading import Lock
from rclpy.callback_groups import ReentrantCallbackGroup
from concurrent.futures import ThreadPoolExecutor
from threading import Lock, Event



from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock, Event

from msg_interfaces.msg import RobotState, RobotStatesQuery
from msg_interfaces.srv import GetMapData


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_path', ''),
                ('graph_path', ''),
                ('schedule_path', ''),
                ('robot_start_path', ''),
                ('publish_frequency', 10.0)
            ]
        )
        
        # 获取参数
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.schedule_path = self.get_parameter('schedule_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # 初始化数据结构
        self.active_robots = []
        self.robot_states = {}  # 存储每个机器人的最新状态
        self.robot_subscribers = {}
        self._lock = Lock()
        
        self.expected_robots = set()  # 预期的机器人ID集合
        self.registered_robots = set()  # 已注册的机器人ID集合
        self.registration_complete = False  # 是否所有预期机器人都已注册
        self._registration_lock = Lock()  # 用于注册状态的锁
        
        self.service_group = ReentrantCallbackGroup() # 允许并发处理
        self._registration_complete_event = Event() # 创建事件用于通知注册完成
        self._executor = ThreadPoolExecutor(max_workers=10) # 使用线程池执行器
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 读取配置文件并且发布数据
        self.load_config_files()
        self.parse_robot_start()
        
        # 创建全局地图数据发布者
        self.map_service = self.create_service(
            GetMapData,
            '/get_map_data',
            self.handle_get_map_data,
            callback_group=self.service_group
        )
        
        # 创建机器人状态查询响应发布者
        self.states_publisher = self.create_publisher(
            RobotStatesQuery,
            '/manager/robot_states',
            self.qos_profile
        )
        
        # 创建定时发布定时器
        self.create_timer(
            1.0/self.publish_frequency,
            self.publish_states,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.get_logger().info('Robot manager initialized successfully')
    
    def load_config_files(self):
        """加载配置文件"""
        try:
            print(self.map_path)
            # 从配置文件读取相关数据
            with open(self.map_path, 'r') as f:
                self.map_json = f.read()  # 直接读取json字符串
            
            with open(self.graph_path, 'r') as f:
                self.graph_json = f.read()
                
            with open(self.schedule_path, 'r') as f:
                self.schedule_json = f.read()
                
            with open(self.robot_start_path, 'r') as f:
                self.robot_start = f.read()
        
        except Exception as e:
            self.get_logger().error(f'Error loading config files: {str(e)}')
            raise
        
    def parse_robot_start(self):
        """解析robot_start文件并设置预期的机器人ID"""
        try:
            import json
            robot_start_data = json.loads(self.robot_start)
            self.expected_robots = set(int(robot_id) for robot_id in robot_start_data.keys())
            self.get_logger().info(f'Expected robots: {self.expected_robots}')
        except Exception as e:
            self.get_logger().error(f'Error parsing robot_start: {str(e)}')
            raise
    
    def wait_for_registration(self, timeout=60):
        """等待所有机器人注册完成"""
        return self._registration_complete_event.wait(timeout)
    
    def handle_get_map_data(self, request, response):
        """处理地图数据请求服务"""
        try:
            self.get_logger().info(f'Received map data request from robot {request.robot_id}')
            
            # 尝试注册机器人
            if not self.register_robot(request.robot_id):
                self.get_logger().error(f'Registration failed for robot {request.robot_id}')
                return response
            
            # 等待所有机器人注册完成
            if not self.wait_for_registration():
                self.get_logger().error('Registration timeout')
                return response
            
            # 所有机器人都已注册，返回地图数据
            response.map_json = self.map_json
            response.graph_json = self.graph_json
            response.schedule_json = self.schedule_json
            response.robot_start = self.robot_start
            
            self.get_logger().info(f'Map data sent to robot {request.robot_id}')
            return response
                
        except Exception as e:
            self.get_logger().error(f'Error handling map data request: {str(e)}')
            raise
        
    def register_robot(self, robot_id: int) -> bool:
        """注册新的机器人，返回是否注册成功"""
        if robot_id not in self.expected_robots:
            self.get_logger().warn(f'Robot {robot_id} not in expected robots list, registration rejected')
            return False
            
        if robot_id in self.registered_robots:
            self.get_logger().warn(f'Robot {robot_id} already registered')
            return True
                
        # 创建该机器人的状态订阅者
        state_sub = self.create_subscription(
            RobotState,
            f'/robot_{robot_id}/state',
            lambda msg: self.state_callback(msg, robot_id),
            self.qos_profile
        )
        
        self.robot_subscribers[robot_id] = state_sub
        
        with self._registration_lock:
            self.registered_robots.add(robot_id)
            self.active_robots.append(robot_id)
            self.robot_states[robot_id] = None
            
            # 检查是否所有预期的机器人都已注册
            if self.registered_robots == self.expected_robots:
                self.registration_complete = True
                self._registration_complete_event.set()
                self.get_logger().info('All expected robots have been registered')
                
        self.get_logger().info(f'Robot {robot_id} registered successfully')
        return True
        
    def unregister_robot(self, robot_id: int) -> None:
        """注销机器人"""
        if robot_id not in self.registered_robots:
            return

        # 删除订阅者
        if robot_id in self.robot_subscribers:
            self.destroy_subscription(self.robot_subscribers[robot_id])
            del self.robot_subscribers[robot_id]

        with self._registration_lock:
            self.registered_robots.remove(robot_id)
            self.active_robots.remove(robot_id)
            if robot_id in self.robot_states:
                del self.robot_states[robot_id]

            # 重置注册完成状态
            self.registration_complete = False
            self._registration_complete_event.clear()

        self.get_logger().info(f'Robot {robot_id} unregistered')
        
    def state_callback(self, msg: RobotState, robot_id: int) -> None:
        """处理机器人状态更新"""
        with self._lock:
            self.robot_states[robot_id] = msg
            
    def publish_states(self):
        """广播所有机器人状态"""
        try:
            with self._lock:
                msg = RobotStatesQuery()
                msg.stamp = self.get_clock().now().to_msg()

                # 添加所有机器人的状态
                for robot_id, state in self.robot_states.items():
                    if state is not None:
                        msg.robot_states.append(state)

                # 发布消息
                self.states_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing states: {str(e)}')

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