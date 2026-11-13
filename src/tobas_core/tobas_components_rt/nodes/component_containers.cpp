// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/multi_component_managers.hpp"

#include <sched.h>

#include <algorithm>
#include <charconv>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <system_error>
#include <thread>

#include <rclcpp/utilities.hpp>

int main(int _argc, char* _argv[])
{
  // Initialize ROS nodes.
  const auto args = rclcpp::init_and_remove_ros_arguments(_argc, _argv);

  // CPU 0 is reserved, and each manager needs one bit in the CPU affinity mask.
  constexpr size_t kMaxManagers = std::numeric_limits<uint32_t>::digits - 1;

  // Read the manager count from the first positional argument, defaulting to zero.
  size_t num_managers = 0;
  if (args.size() >= 2) {
    const auto& value = args[1];
    const auto [ptr, error] = std::from_chars(value.data(), value.data() + value.size(), num_managers);
    if (error != std::errc{} || ptr != value.data() + value.size() || num_managers > kMaxManagers) {
      std::cerr << "The manager count must be an integer between 0 and " << kMaxManagers << ".\n";
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  }

  // Select the manager count automatically when the argument is omitted or zero.
  if (num_managers == 0) {
    // Leave CPU 0 free; use one manager if there is only one CPU or the count is unknown.
    const auto num_cpus = std::thread::hardware_concurrency();
    num_managers = num_cpus > 1 ? std::min<size_t>(num_cpus - 1, kMaxManagers) : 1;
  }

  // Run multiple component managers in a single process.
  tobas::MultiComponentManagers managers(num_managers);

  // Configure each manager's scheduling policy, priority, CPU affinity, and worker count.
  for (size_t i = 0; i < num_managers; ++i) {
    // Use a policy that strictly follows priorities.
    managers.setPolicy(i, SCHED_FIFO);

    // Keep this below the default IO IRQ priority of 50 because setting it to 50 or higher can deadlock.
    // when CPU affinity is fixed.
    // https://docs.redhat.com/ja/documentation/red_hat_enterprise_linux/9/html/monitoring_and_managing_system_status_and_performance/priority-map_tuning-scheduling-policy
    managers.setPriority(i, 49 - i);

    // Dedicate one CPU to each component manager.
    managers.setCpuAffinity(i, 1U << (i + 1));

    // Use one EventsCBGExecutor worker for each dedicated CPU.
    managers.setNumThreads(i, 1);
  }

  // Exit immediately on Ctrl+C.
  signal(SIGINT, [](int) { rclcpp::shutdown(); });

  // Spin all component managers until ROS shuts down.
  managers.spin();
  rclcpp::shutdown();
}
