#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_component_managers.hpp"

using namespace std;

namespace ros2
{
MultiComponentManagers::MultiComponentManagers(int policy, size_t num_managers, size_t num_threads)
  : managers_(num_managers)
{
  if (num_managers < 1 || 9 < num_managers)
    throw runtime_error("The number of component managers must be in range of [1, 9].");

  const auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  for (size_t i = 0; i < num_managers; ++i)
  {
    managers_[i].exec = make_shared<MultiThreadedExecutorRT>(threadPriority(i), policy, num_threads);
    managers_[i].node = make_shared<rclcpp_components::ComponentManager>(managers_[i].exec, nodeName(i), options);
    managers_[i].exec->add_node(managers_[i].node);
    managers_[i].thread = thread([&]() { managers_[i].exec->spin(); });

    // スレッドのリアルタイム優先度を設定
    if (!linux::setThreadPriority(managers_[i].thread.native_handle(), threadPriority(i), policy))
      RCLCPP_WARN(managers_[i].node->get_logger(), "Failed to set realtime thread priority.");

    // TODO: Set CPU affinity?
  }
}

void MultiComponentManagers::spin()
{
  for (auto& manager : managers_)
    manager.thread.join();
}

size_t MultiComponentManagers::threadPriority(size_t tier)
{
  return 90 - 10 * tier;
}

string MultiComponentManagers::nodeName(size_t tier)
{
  return "component_manager_" + to_string(tier + 1);
}
}  // namespace ros2
