// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./ros_interface.hpp"

namespace tobas
{
RosInterfaceNode::RosInterfaceNode(const rclcpp::NodeOptions& _options)
  : super("ros_interface", nodeOptions_Default(_options))
{
  // Serialize topic forwarding on the single CPU assigned to this node.
  // Keep it separate from blocking server callbacks.
  topic_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Serialize blocking server callbacks and leave a worker free for client responses in the default group.
  server_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Register ROS interfaces.
  // These methods are split for separate compilation to reduce memory usage.
  registerTopicsLogicToIface();
  registerTopicsIfaceToLogic();
  registerServices();
  registerActions();
}
}  // namespace tobas
