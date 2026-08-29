// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_visualization_msgs/msg/robot_state.hpp>

#include "./robot_state.hpp"

namespace tobas
{
namespace rviz
{
bool robotStateMsgToRobotState(const tobas_visualization_msgs::msg::RobotState& robot_state, RobotState& state);
}  // namespace rviz
}  // namespace tobas
