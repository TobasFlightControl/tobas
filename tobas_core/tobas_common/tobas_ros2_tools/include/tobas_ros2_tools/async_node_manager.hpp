// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>
#include <thread>

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

  void shutdown();

  inline rclcpp::Node::SharedPtr node();
  inline rclcpp::Node::ConstSharedPtr node() const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Context::SharedPtr context_;
  std::optional<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::optional<std::thread> executor_thread_;

  void initialize(rclcpp::Context::SharedPtr context, const std::string& node_name);
};

inline rclcpp::Node::SharedPtr AsyncNodeManager::node()
{
  return node_;
}

inline rclcpp::Node::ConstSharedPtr AsyncNodeManager::node() const
{
  return node_;
}
}  // namespace ros2
}  // namespace tobas
