from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from zmr_interface import RobotConfig, RobotState, MotionModel, RobotAction


class Robot(Node):
    def __init__(self, config: RobotConfig, motion_model: MotionModel, id_: Optional[int] = None, name: Optional[str] = None):
        super().__init__(name or f'robot_{id_}')
        self.config = config
        self.motion_model = motion_model
        self._check_identifier(id_, name)
        self._priority = 0
        self._state = None

        self.action_subscription_ = self.create_subscription(RobotAction, f'robot_{self.id_}/action', self.action_callback, 10)
        self.state_publisher_ = self.create_publisher(RobotState, f'robot_{self.id_}/state', 10)
        self.config_subscription_ = self.create_subscription(RobotConfig, f'robot_{self.id_}/config', self.config_callback, 10)
        self.motion_model_subscription_ = self.create_subscription(MotionModel, f'robot_{self.id_}/motion_model', self.motion_model_callback, 10)

    def _check_identifier(self, id_: Optional[int], name: Optional[str]) -> None:
        if id_ is None:
            if max(self._id_list) >= self.MAX_NUMBER_OF_ROBOTS:
                raise ValueError('Maximum number of robots reached.')
            id_ = max(self._id_list) + 1 if self._id_list else 0
        elif id_ in self._id_list:
            raise ValueError(f'A robot with id {id_} already exists.')
        
        self._id = id_
        self._id_list.append(id_)
        
        if name is None:
            name = f'{self.__class__.__name__}_{id_}'
        
        self._name = name

    @classmethod
    def reset_id_list(cls) -> None:
        cls._id_list = [-1]

    @property
    def id_(self) -> int:
        return self._id

    @property
    def name(self) -> str:
        return self._name

    @property
    def priority(self) -> int:
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        self._priority = value

    @property
    def state(self) -> np.ndarray:
        return self._state

    def config_callback(self, msg: RobotConfig):
        self.config = msg

    def motion_model_callback(self, msg: MotionModel):
        self.motion_model = msg

    def action_callback(self, msg: RobotAction):
        action = np.array(msg.action)
        self.step(action)

    def publish_state(self):
        msg = RobotState()
        msg.id = self.id_
        msg.state = self.state.tolist()
        self.state_publisher_.publish(msg)

    def set_state(self, state: np.ndarray) -> None:
        self._state = state

    def step(self, action: np.ndarray) -> None:
        # 根据运动模型更新机器人状态
        self._state = self.motion_model.predict(self.state, action)
        # 发布最新的机器人状态
        self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    config = RobotConfig()
    motion_model = MotionModel()
    robot = Robot(config, motion_model, id_=1)
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()