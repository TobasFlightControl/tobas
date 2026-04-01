# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Euler angle  # Target euler angles expressed in the global frame [rad]
float64 throttle            # Target throttle [0, 1]
float64 thrust_angle        # Target thrust angle [rad]
