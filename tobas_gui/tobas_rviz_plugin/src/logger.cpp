// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/logger.hpp"

#include <rclcpp/node.hpp>
#include <rsl/random.hpp>

namespace tobas
{
namespace
{
/* Stores the global logger used by Tobas. */
rclcpp::Logger& getGlobalRootLogger()
{
  static rclcpp::Logger logger = [&]
  {
    // A random number is appended to the name used for the node to make it unique.
    const auto name = std::format("tobas_{}", rsl::rng()());
    try {
      static auto* tobas_node = new rclcpp::Node(name);
      return tobas_node->get_logger();
    }
    catch (const std::exception& e) {
      // rclcpp::init was not called so rcl context is null, return non-node logger.
      const auto logger2 = rclcpp::get_logger(name);
      RCLCPP_WARN_STREAM(logger2, "exception thrown while creating node for logging: " << e.what());
      RCLCPP_WARN(logger2, "if rclcpp::init was not called, messages from this logger may be missing from /rosout");
      return logger2;
    }
  }();
  return logger;
}
}  // namespace

rclcpp::Logger getLogger(const std::string& name)
{
  return getGlobalRootLogger().get_child(name);
}
}  // namespace tobas
