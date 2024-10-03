#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "./multi_threaded_executor.hpp"

namespace ros2
{
struct ComponentManager
{
  std::shared_ptr<MultiThreadedExecutorRT> exec;
  std::shared_ptr<rclcpp_components::ComponentManager> node;
  std::thread thread;
};

class MultiComponentManagers
{
public:
  explicit MultiComponentManagers(int policy, size_t num_managers, size_t num_threads = 0);

  void spin();

private:
  std::vector<ComponentManager> managers_;

  static size_t threadPriority(size_t tier);
  static std::string nodeName(size_t tier);
};
}  // namespace ros2
