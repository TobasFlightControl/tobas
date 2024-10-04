#include <thread>

#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_component_managers.hpp"

using namespace std;

namespace ros2
{
MultiComponentManagers::MultiComponentManagers(size_t num_managers)
  : num_managers_(num_managers),
    policy_(num_managers, SCHED_FIFO),
    priority_(num_managers, 0),
    affinity_(num_managers, 0),
    num_threads_(num_managers, 0)
{
}

void MultiComponentManagers::setPolicy(size_t idx, int policy)
{
  policy_.at(idx) = policy;
}

void MultiComponentManagers::setPriority(size_t idx, size_t priority)
{
  priority_.at(idx) = priority;
}

void MultiComponentManagers::setCPUAffinity(size_t idx, uint32_t affinity)
{
  affinity_.at(idx) = affinity;
}

void MultiComponentManagers::setNumThreads(size_t idx, size_t num_threads)
{
  num_threads_.at(idx) = num_threads;
}

void MultiComponentManagers::spin()
{
  vector<ComponentManager> managers(num_managers_);

  const auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);

  for (size_t i = 0; i < num_managers_; ++i)
  {
    managers[i].exec = make_shared<MultiThreadedExecutorRT>(policy_[i], priority_[i], affinity_[i], num_threads_[i]);
    managers[i].node = make_shared<rclcpp_components::ComponentManager>(managers[i].exec, nodeName(i), node_options);
    managers[i].exec->add_node(managers[i].node);

    // ComponentManagerを別スレッドで起動
    // このスレッドにリアルタイムスケジューリングを適用すると，DDS接続作成時のコールバックスレッドの遅延が大きくなる．
    managers[i].thread = thread([&]() { managers[i].exec->spin(); });
    this_thread::sleep_for(100ms);  // 一定時間待機．さもないとスピン中にスピンを呼ぶことになってしまう．
  }

  for (auto& manager : managers)
    manager.thread.join();
}

string MultiComponentManagers::nodeName(size_t idx)
{
  return "component_manager_" + to_string(idx + 1);
}
}  // namespace ros2
