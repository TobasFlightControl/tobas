# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Vector pos  # Target position expressed in the global frame [m]
tobas_kdl_msgs/Vector vel  # Target linear velocity expressed in the global frame [m/s]
tobas_kdl_msgs/Vector acc  # Target linear acceleration expressed in the global frame [m/s]
