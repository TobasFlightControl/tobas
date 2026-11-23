// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/multi_component_managers.hpp"

#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <system_error>
#include <thread>

#include <rclcpp/init_options.hpp>
#include <rclcpp/utilities.hpp>

int main(int _argc, char* _argv[])
{
  // All ROS and executor threads inherit this mask. Consume stop signals outside a signal handler.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  if (const auto error = pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr); error != 0) {
    std::cerr << "Failed to block stop signals: " << std::strerror(error) << '\n';
    return EXIT_FAILURE;
  }

  // Keep the ROS context alive until callbacks finish and component destructors release their hardware.
  rclcpp::init(_argc, _argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const auto args = rclcpp::remove_ros_arguments(_argc, _argv);

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

  // Stop the executors before shutting down ROS, so active timers are not freed during their callbacks.
  managers.spin(
    [&stop_signals]()
    {
      constexpr timespec timeout{ 0, 100'000'000 };  // 100ms
      while (rclcpp::ok()) {
        if (sigtimedwait(&stop_signals, nullptr, &timeout) >= 0) {
          return;
        }
        if (errno != EAGAIN && errno != EINTR) {
          std::cerr << "Failed to wait for stop signals: " << std::strerror(errno) << '\n';
          return;
        }
      }
    });
  rclcpp::shutdown();
}
