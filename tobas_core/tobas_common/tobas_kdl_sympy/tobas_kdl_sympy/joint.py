# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from enum import Enum


class JointType:
    """Corresponds to `urdf_parser_py.urdf.Joint.TYPES`."""

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
