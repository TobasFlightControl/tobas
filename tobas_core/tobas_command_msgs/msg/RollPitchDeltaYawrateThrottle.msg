# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

float64 roll            # Target roll angle [rad]
float64 pitch           # Target pitch angle [rad]
float64 delta_yawrate   # Target delta yawrate [rad/s]
float64 throttle        # Target throttle [0, 1]
