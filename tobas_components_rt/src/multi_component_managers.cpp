#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_component_managers.hpp"

using namespace std;

namespace ros2
{
MultiComponentManagers::MultiComponentManagers(int policy, size_t num_managers, size_t num_threads)
  : policy_(policy), num_managers_(num_managers), num_threads_(num_threads)
{
  if (num_managers < 1 || 9 < num_managers)
    throw runtime_error("The number of component managers must be in range of [1, 9].");
}

void MultiComponentManagers::spin()
{
  std::vector<ComponentManager> managers(num_managers_);

  const auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);

  for (size_t i = 0; i < num_managers_; ++i)
  {
    managers[i].exec = make_shared<MultiThreadedExecutorRT>(priority(i), policy_, num_threads_);
    managers[i].node = make_shared<rclcpp_components::ComponentManager>(managers[i].exec, nodeName(i), node_options);
    managers[i].exec->add_node(managers[i].node);

    // ComponentManagerを別スレッドで起動
    // TODO: スレッドの起動を後でまとめて行おうとするとスピン中にスピンを呼ぶことになってしまうらしいが，それはなぜ？
    managers[i].thread = thread([&]() { managers[i].exec->spin(); });

    // スレッドのリアルタイム優先度を設定
    if (!linux::setThreadPriority(managers[i].thread.native_handle(), priority(i), policy_))
      RCLCPP_WARN(managers[i].node->get_logger(), "Failed to set realtime thread priority.");

    // TODO: Set CPU affinity?
  }

  for (auto& manager : managers)
    manager.thread.join();
}

size_t MultiComponentManagers::priority(size_t tier)
{
  return 90 - 10 * tier;
}

string MultiComponentManagers::nodeName(size_t tier)
{
  return "component_manager_" + to_string(tier + 1);
}
}  // namespace ros2
