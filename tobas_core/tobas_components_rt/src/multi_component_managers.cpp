// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/multi_component_managers.hpp"

#include <ranges>
#include <thread>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include <tobas_linux/realtime.hpp>

#include "tobas_components_rt/component_manager.hpp"
#include "tobas_components_rt/multi_threaded_executor.hpp"
#include "tobas_components_rt/timer_coalescing_events_queue.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace
{
struct ComponentManager
{
  ThreadSafeComponentManager::SharedPtr node;
  rclcpp::Executor::SharedPtr exec;
  std::thread thread;
};

std::string nodeName(size_t idx)
{
  return "component_manager_" + std::to_string(idx + 1);
}
}  // namespace

MultiComponentManagers::MultiComponentManagers(size_t num_managers)
  : num_managers_(num_managers), configs_(num_managers)
{
}

void MultiComponentManagers::setPolicy(size_t idx, linux::sched_t policy)
{
  configs_.at(idx).policy = policy;
}

void MultiComponentManagers::setPriority(size_t idx, size_t priority)
{
  configs_.at(idx).priority = priority;
}

void MultiComponentManagers::setCpuAffinity(size_t idx, uint32_t affinity)
{
  configs_.at(idx).affinity = affinity;
}

void MultiComponentManagers::setNumThreads(size_t idx, size_t num_threads)
{
  configs_.at(idx).num_threads = num_threads;
}

void MultiComponentManagers::spin()
{
  const auto logger = rclcpp::get_logger("multi_component_managers");

  std::vector<ComponentManager> managers(num_managers_);

  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(true);  // Components cannot be loaded when this is false.
  node_options.enable_rosout(false);
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);
  node_options.append_parameter_override("start_type_description_service", false);

  for (auto&& [i, manager, cfg] : std::views::zip(std::views::iota(0), managers, configs_)) {
    if (cfg.num_threads == 1) {
      manager.exec = std::make_shared<rclcpp::experimental::executors::EventsExecutor>(
        std::make_unique<TimerCoalescingEventsQueue>());
    }
    else {
      manager.exec = std::make_shared<MultiThreadedExecutorRT>(cfg.policy, cfg.priority, cfg.affinity, cfg.num_threads);
    }

    manager.node = std::make_shared<ThreadSafeComponentManager>(manager.exec, nodeName(i), node_options);
    manager.exec->add_node(manager.node);

    const auto policy = cfg.policy;
    const auto priority = cfg.priority;
    const auto affinity = cfg.affinity;

    // Configure the worker before spin() creates any executor-internal threads.
    manager.thread = std::thread(
      [&manager, policy, priority, affinity, logger]()
      {
        const auto thread = pthread_self();
        if (priority > 0) {
          if (!linux::setThreadPriority(thread, priority, policy)) {
            RCLCPP_WARN(logger, "Failed to set thread realtime priority.");
          }
        }
        if (affinity > 0) {
          if (!linux::setThreadCPUAffinity(thread, affinity)) {
            RCLCPP_WARN(logger, "Failed to set thread CPU affinity.");
          }
        }
        manager.exec->spin();
      });

    // Wait briefly to avoid calling spin while already spinning.
    std::this_thread::sleep_for(100ms);
  }

  for (auto& manager : managers) {
    manager.thread.join();
  }
}
}  // namespace tobas
