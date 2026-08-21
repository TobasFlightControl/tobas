// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/multi_component_managers.hpp"

#include <sched.h>

#include <csignal>
#include <cstddef>

#include <rclcpp/utilities.hpp>

int main(int argc, char* argv[])
{
  constexpr size_t kNumManagers = 3;

  // Initialize ROS nodes.
  rclcpp::init(argc, argv);

  // Exit immediately on Ctrl+C.
  signal(SIGINT, [](int) { rclcpp::shutdown(); });

  // Run multiple component managers in a single process.
  tobas::MultiComponentManagers managers(kNumManagers);

  for (size_t i = 0; i < kNumManagers; ++i) {
    // Use a policy that strictly follows priorities.
    managers.setPolicy(i, SCHED_FIFO);

    // Keep this below the default IO IRQ priority of 50 because setting it to 50 or higher can deadlock
    // when CPU affinity is fixed.
    // https://docs.redhat.com/ja/documentation/red_hat_enterprise_linux/9/html/monitoring_and_managing_system_status_and_performance/priority-map_tuning-scheduling-policy
    managers.setPriority(i, 49 - i);

    // Dedicate one CPU to each component manager.
    managers.setCpuAffinity(i, 1 << (i + 1));

    // `MultiThreadedExecutor` is CPU intensive, so assign only one thread to each CPU for better performance.
    managers.setNumThreads(i, 1);
  }

  managers.spin();

  rclcpp::shutdown();
}
