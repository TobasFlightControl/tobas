// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>

namespace tobas
{
namespace ros2
{
/**
 * @brief Create and manage a ROS node that runs on a thread separate from the main thread.
 */
class AsyncNodeManager
{
public:
  explicit AsyncNodeManager(int argc, char** argv, const std::string& node_name);
  explicit AsyncNodeManager(rclcpp::Context::SharedPtr context, const std::string& node_name);
  ~AsyncNodeManager();

  void clear();

  rclcpp::Node::SharedPtr node();
  rclcpp::Node::ConstSharedPtr node() const;

private:
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::unique_ptr<std::thread> executor_thread_;

  void initialize(const std::string& node_name);
};
}  // namespace ros2
}  // namespace tobas
