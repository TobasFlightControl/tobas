// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <std_msgs/msg/string.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerModelSubscribers()
{
  addTypeAdaptedMsgSub<Drone>(drone_, topic::kDrone, true, true);
  addTypeAdaptedMsgSub<kdl::Tree>(tree_, topic::kKdlTree, true, true);
  addStandardMsgSub<std_msgs::msg::String>(topic::kRobotDescription, true, true);
}
}  // namespace tobas
