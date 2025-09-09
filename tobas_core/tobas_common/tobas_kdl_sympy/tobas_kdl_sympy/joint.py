from enum import Enum


class JointType:
    """urdf_parser_py.urdf.Joint.TYPESに対応．"""

    UNKNOWN = "unknown"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FLOATING = "floating"
    PLANER = "planar"
    FIXED = "fixed"


class HardwareInterface(Enum):
    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"
