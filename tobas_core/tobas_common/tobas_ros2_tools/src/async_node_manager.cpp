// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/async_node_manager.hpp"

#include <stdexcept>
#include <utility>

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

namespace tobas
{
namespace ros2
{
AsyncNodeManager::AsyncNodeManager(int argc, char** argv, const std::string& node_name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  context_ = rclcpp::contexts::get_global_default_context();
  initialize(node_name);
}

AsyncNodeManager::AsyncNodeManager(rclcpp::Context::SharedPtr context, const std::string& node_name)
  : context_(std::move(context))
{
  if (!context_ || !context_->is_valid()) {
    throw std::invalid_argument("The ROS context must be initialized before creating an AsyncNodeManager.");
  }

  initialize(node_name);
}

void AsyncNodeManager::initialize(const std::string& node_name)
{
  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = rclcpp::Node::make_shared(node_name, node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  executor_->add_node(node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

AsyncNodeManager::~AsyncNodeManager()
{
  clear();
}

void AsyncNodeManager::clear()
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_ && executor_thread_->joinable()) {
    executor_thread_->join();
  }

  executor_thread_.reset();
  executor_.reset();
  node_.reset();
  context_.reset();
}

rclcpp::Node::SharedPtr AsyncNodeManager::node()
{
  return node_;
}

rclcpp::Node::ConstSharedPtr AsyncNodeManager::node() const
{
  return node_;
}
}  // namespace ros2
}  // namespace tobas
