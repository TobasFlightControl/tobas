# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Vector rate  # Target angular velocity expressed in the body frame [rad/s]
float64 throttle            # Target throttle [0, 1]
float64 thrust_angle        # Target thrust angle [rad]
