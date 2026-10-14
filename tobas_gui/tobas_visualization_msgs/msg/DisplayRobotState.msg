# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.
# Source: https://github.com/moveit/moveit_msgs/blob/HEAD/msg/DisplayRobotState.msg
# Modified by Tobas, Inc.

# The robot state to display.
RobotState state

# Optionally, various links can be highlighted.
ObjectColor[] highlight_links

# If true, suppress the display in visualizations (like rviz).
bool hide
