// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

#include "./ros_interface.hpp"

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::RosInterfaceNode>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
}
