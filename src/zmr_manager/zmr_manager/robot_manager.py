import rclpy
from rclpy.node import Node
from zmr_interface import RobotAction, RobotState

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        self.robot_states = {}
        self.robot_state_subscriptions = {}
        self.robot_action_publishers = {}
        
    def add_robot(self, robot_id):
        self.robot_states[robot_id] = None
        self.robot_state_subscriptions[robot_id] = self.create_subscription(
            RobotState,
            f'robot_{robot_id}/state',
            lambda msg: self.robot_state_callback(robot_id, msg),
            10
        )
        self.robot_action_publishers[robot_id] = self.create_publisher(
            RobotAction,
            f'robot_{robot_id}/action',
            10
        )
    
    def remove_robot(self, robot_id):
        del self.robot_states[robot_id]
        self.destroy_subscription(self.robot_state_subscriptions[robot_id])
        del self.robot_state_subscriptions[robot_id]
        self.destroy_publisher(self.robot_action_publishers[robot_id])
        del self.robot_action_publishers[robot_id]
    
    def robot_state_callback(self, robot_id, msg):
        self.robot_states[robot_id] = msg
    
    def send_action(self, robot_id, action):
        msg = RobotAction()
        msg.robot_id = robot_id
        msg.action_type = action['type']
        msg.action_params = action['params']
        self.robot_action_publishers[robot_id].publish(msg)
    
    def get_robot_state(self, robot_id):
        return self.robot_states[robot_id]
    
def main(args=None):
    rclpy.init(args=args)

    robot_manager = RobotManager()

    # 添加一些示例机器人
    robot_manager.add_robot(1)
    robot_manager.add_robot(2)

    # 创建一个定时器,每秒钟打印一次所有机器人的状态
    def timer_callback():
        for robot_id in robot_manager.robot_states:
            state = robot_manager.get_robot_state(robot_id)
            if state is not None:
                robot_manager.get_logger().info(f'Robot {robot_id} state: {state}')
    
    timer_period = 1.0  # 定时器周期,单位为秒
    timer = robot_manager.create_timer(timer_period, timer_callback)

    rclpy.spin(robot_manager)

    # 程序结束,销毁节点并关闭rclpy
    robot_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()