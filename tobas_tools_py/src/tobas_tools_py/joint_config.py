from enum import Enum
from dataclasses import dataclass
from typing import List


class JointCommandType(Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


@dataclass
class JointConfig:
    name: str = ""
    home_pos: float = 0.0
    min_pos: float = 0.0
    max_pos: float = 0.0
    cmd_type: JointCommandType = JointCommandType.POSITION


JointConfigs = List[JointConfig]
