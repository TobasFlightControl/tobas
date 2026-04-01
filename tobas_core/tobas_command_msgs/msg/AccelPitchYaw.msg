# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Vector accel  # Target linear acceleration expressed in the global frame [m/s^2]
float64 pitch                # Target pitch angle [rad]
float64 yaw                  # Target yaw angle [rad]
