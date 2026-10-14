// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/async_node_manager.hpp"

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

namespace tobas::ros2
{
AsyncNodeManager::AsyncNodeManager(int _argc, char** _argv, const std::string& _node_name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(_argc, _argv);
  }

  initialize(rclcpp::contexts::get_global_default_context(), _node_name);
}

AsyncNodeManager::AsyncNodeManager(rclcpp::Context::SharedPtr _context, const std::string& _node_name)
{
  initialize(_context, _node_name);
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

void AsyncNodeManager::initialize(rclcpp::Context::SharedPtr _context, const std::string& _node_name)
{
  context_ = std::move(_context);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = rclcpp::Node::make_shared(_node_name, node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_.emplace(executor_options);
  executor_->add_node(node_);
  executor_thread_.emplace([this]() { executor_->spin(); });
}
}  // namespace tobas::ros2
