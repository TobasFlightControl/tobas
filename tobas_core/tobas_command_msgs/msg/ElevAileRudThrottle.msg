# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

float64 elevator     # Target elevator deflection [-1, 1]
float64 aileron      # Target aileron deflection [-1, 1]
float64 rudder       # Target rudder deflection [-1, 1]
float64 throttle     # Target throttle [0, 1]

float64 MAX_DEFLECTION = 1.0
float64 MIN_DEFLECTION = -1.0
