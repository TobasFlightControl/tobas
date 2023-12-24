from enum import Enum


class JointType:
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    PLANER = "planar"
    FLOATING = "floating"


class HardwareInterface(Enum):
    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"
