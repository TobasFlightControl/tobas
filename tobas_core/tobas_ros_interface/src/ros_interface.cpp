// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./ros_interface.hpp"

namespace tobas
{
RosInterfaceNode::RosInterfaceNode(const rclcpp::NodeOptions& options)
  : super("ros_interface", nodeOptions_Default(options))
{
  // Avoid deadlock when service callbacks are called recursively.
  // cf. https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
  group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Register ROS interfaces.
  // These methods are split for separate compilation to reduce memory usage.
  registerTopicsLogicToIface();
  registerTopicsIfaceToLogic();
  registerServices();
  registerActions();
}
}  // namespace tobas
