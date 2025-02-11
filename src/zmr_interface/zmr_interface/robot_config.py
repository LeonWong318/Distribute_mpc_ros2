# robot_config.py

from dataclasses import dataclass

@dataclass
class RobotConfig:
    max_velocity: float
    max_acceleration: float
    max_jerk: float
    robot_radius: float