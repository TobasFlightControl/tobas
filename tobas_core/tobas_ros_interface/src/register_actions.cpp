// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_real_common/ros_interface.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>

#include "./ros_interface.hpp"

namespace tobas
{
void RosInterfaceNode::registerActions()
{
  addAction<tobas_mission_msgs::action::ExecuteMission>(action::kExecuteMission);
}
}  // namespace tobas
