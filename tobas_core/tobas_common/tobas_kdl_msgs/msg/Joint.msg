# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

# tobas::kdl::Joint
string name
uint8 type
tobas_kdl_msgs/Vector origin
tobas_kdl_msgs/Vector axis
float64 damping
float64 friction
float64 lower_limit
float64 upper_limit
float64 max_effort
float64 max_velocity
