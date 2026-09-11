# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

float64 elevator     # [rad]
float64 aileron      # [rad]
float64 rudder       # [rad]
float64 throttle     # Target throttle [0, 1]
