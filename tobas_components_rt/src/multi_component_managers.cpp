#include <thread>

#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_component_managers.hpp"
#include "../include/tobas_components_rt/multi_threaded_executor.hpp"

using namespace std;

namespace ros2
{
MultiComponentManagers::MultiComponentManagers(size_t num_managers)
  : num_managers_(num_managers),
    policy_(num_managers, SCHED_FIFO),
    priority_(num_managers, 0),
    affinity_(num_managers, 0),
    num_threads_(num_managers, 1)
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

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  for (size_t i = 0; i < num_managers_; ++i) {
    if (num_threads_[i] == 1) {
      managers[i].exec = make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    else {
      managers[i].exec = make_shared<MultiThreadedExecutorRT>(policy_[i], priority_[i], affinity_[i], num_threads_[i]);
    }

    managers[i].node = make_shared<ros2::ThreadSafeComponentManager>(managers[i].exec, nodeName(i), node_options);
    managers[i].exec->add_node(managers[i].node);

    // ComponentManagerを別スレッドで起動
    managers[i].thread = thread([&]() { managers[i].exec->spin(); });

    // スレッドのリアルタイム優先度を設定
    if (priority_[i] > 0) {
      if (!linux::setThreadPriority(managers[i].thread.native_handle(), priority_[i], policy_[i])) {
        RCLCPP_WARN(rclcpp::get_logger(kName), "Failed to set thread realtime priority.");
      }
    }

    // スレッドのCPU割当を設定
    if (affinity_[i] > 0) {
      if (!linux::setThreadCPUAffinity(managers[i].thread.native_handle(), affinity_[i])) {
        RCLCPP_WARN(rclcpp::get_logger(kName), "Failed to set thread CPU affinity.");
      }
    }

    this_thread::sleep_for(100ms);  // 一定時間待機．さもないとスピン中にスピンを呼ぶことになってしまう．
  }

  for (auto& manager : managers) {
    manager.thread.join();
  }
}

string MultiComponentManagers::nodeName(size_t idx)
{
  return "component_manager_" + to_string(idx + 1);
}
}  // namespace ros2
