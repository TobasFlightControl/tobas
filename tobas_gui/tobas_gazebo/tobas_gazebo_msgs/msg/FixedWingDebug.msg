# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
geometry_msgs/Vector3 relative_body_velocity  # uvw (FRD body frame) [m/s]
float64 alpha                                 # Angle of attack [rad]
float64 beta                                  # Angle of sideslip [rad]
geometry_msgs/Vector3 air_force               # Air force (FLU body frame) [N]
geometry_msgs/Vector3 air_moment              # Air moment (FLU body frame) [Nm]
