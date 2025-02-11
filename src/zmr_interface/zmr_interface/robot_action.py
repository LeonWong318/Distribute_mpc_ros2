from dataclasses import dataclass
from typing import List

@dataclass
class RobotAction:
    robot_id: int
    action_type: str
    action_params: List[float]