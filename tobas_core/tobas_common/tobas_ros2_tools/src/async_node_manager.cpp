// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/async_node_manager.hpp"

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

  initialize(rclcpp::contexts::get_global_default_context(), node_name);
}

AsyncNodeManager::AsyncNodeManager(rclcpp::Context::SharedPtr context, const std::string& node_name)
{
  initialize(context, node_name);
}

AsyncNodeManager::~AsyncNodeManager()
{
  shutdown();
}

void AsyncNodeManager::shutdown()
{
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
  }

  if (executor_thread_ && executor_thread_->joinable()) {
    executor_thread_->join();
  }

  if (context_ && context_->is_valid()) {
    context_->shutdown("");
  }

  node_.reset();
  context_.reset();
  executor_.reset();
  executor_thread_.reset();
}

void AsyncNodeManager::initialize(rclcpp::Context::SharedPtr context, const std::string& node_name)
{
  context_ = std::move(context);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = rclcpp::Node::make_shared(node_name, node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_.emplace(executor_options);
  executor_->add_node(node_);
  executor_thread_.emplace([this]() { executor_->spin(); });
}
}  // namespace ros2
}  // namespace tobas
