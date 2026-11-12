# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

float64 vel_x           # Target linear velocity expressed in the yaw-rotated global frame [m/s]
float64 vel_z           # Target linear velocity expressed in the yaw-rotated global frame [m/s]
float64 roll            # Target roll angle [rad]
float64 side_slip       # Target side slip angle [rad]
uint8 mode              # mode

uint8 FORWARD_FLIGHT = 0
uint8 BACKWARD_TRANSITION = 1
