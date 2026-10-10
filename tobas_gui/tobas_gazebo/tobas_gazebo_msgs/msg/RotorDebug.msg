# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
float64 position                       # [rad]
float64 velocity                       # [rad/s]
float64 acceleration                   # [rad/s^2]
float64 esc_thermal_load               # [-] Normalized ESC thermal load; failure above 1.0.
geometry_msgs/Vector3 inertia_moment   # [Nm]
geometry_msgs/Vector3 coriolis_moment  # [Nm]
geometry_msgs/Vector3 thrust_force     # [N]
geometry_msgs/Vector3 h_force          # [N]
geometry_msgs/Vector3 drag_momeent     # [Nm]
