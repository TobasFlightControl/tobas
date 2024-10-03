#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "./multi_threaded_executor.hpp"

namespace ros2
{
struct ComponentManager
{
  rclcpp_components::ComponentManager::SharedPtr node;
  MultiThreadedExecutorRT::SharedPtr exec;
  std::thread thread;
};

class MultiComponentManagers
{
public:
  explicit MultiComponentManagers(int policy, size_t num_managers, size_t num_threads = 0);

  void spin();

private:
  const int policy_;
  const size_t num_managers_;
  const size_t num_threads_;

  static size_t priority(size_t tier);
  static std::string nodeName(size_t tier);
};
}  // namespace ros2
