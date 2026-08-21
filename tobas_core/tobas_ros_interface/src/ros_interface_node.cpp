// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "./ros_interface.hpp"

int main(int argc, char* argv[])
{
  constexpr long kDefaultNumThreads = 4;
  constexpr long kMinNumThreads = 2;

  rclcpp::init(argc, argv);

  const auto node = std::make_shared<tobas::RosInterfaceNode>();

  long num_threads = kDefaultNumThreads;
  if (node->has_parameter("num_threads")) {
    num_threads = node->get_parameter("num_threads").as_int();
  }

  if (num_threads < kMinNumThreads) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "To avoid deadlock with recursive service calls, at least " << kMinNumThreads << " threads are required.");
    num_threads = kMinNumThreads;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "The number of threads has been set to " << num_threads << ".");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_threads);
  exec.add_node(node);

  exec.spin();
}
