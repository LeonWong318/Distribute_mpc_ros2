# robot_state.py

from dataclasses import dataclass
from typing import List

@dataclass
class RobotState:
    id: int
    state: List[float]