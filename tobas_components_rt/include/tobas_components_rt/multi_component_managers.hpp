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
  explicit MultiComponentManagers(size_t num_managers);

  void setPolicy(size_t idx, int policy);
  void setPriority(size_t idx, size_t priority);
  void setCPUAffinity(size_t idx, uint32_t affinity);
  void setNumThreads(size_t idx, size_t num_threads);

  void spin();

private:
  const size_t num_managers_;

  std::vector<int> policy_;
  std::vector<size_t> priority_;
  std::vector<uint32_t> affinity_;
  std::vector<size_t> num_threads_;

  static std::string nodeName(size_t idx);
};
}  // namespace ros2
