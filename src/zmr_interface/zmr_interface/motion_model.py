# motion_model.py

from dataclasses import dataclass
from typing import List

@dataclass
class MotionModel:
    model_type: str
    model_params: List[float]