#include "tobas_components_rt/multi_component_managers.hpp"

#include <ranges>
#include <thread>

#include <tobas_linux/realtime.hpp>

#include "tobas_components_rt/component_manager.hpp"
#include "tobas_components_rt/multi_threaded_executor.hpp"

using namespace std::chrono_literals;

namespace ros2
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
  constexpr char kLoggerName[] = "multi_component_managers";

  std::vector<ComponentManager> managers(num_managers_);

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  for (auto&& [i, manager, cfg] : std::views::zip(std::views::iota(num_managers_), managers, configs_)) {
    if (cfg.num_threads == 1) {
      manager.exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    else {
      manager.exec = std::make_shared<MultiThreadedExecutorRT>(cfg.policy, cfg.priority, cfg.affinity, cfg.num_threads);
    }

    manager.node = std::make_shared<ThreadSafeComponentManager>(manager.exec, nodeName(i), node_options);
    manager.exec->add_node(manager.node);

    // ComponentManagerを別スレッドで起動
    manager.thread = std::thread([&manager]() { manager.exec->spin(); });

    // スレッドのリアルタイム優先度を設定
    if (cfg.priority > 0) {
      if (!linux::setThreadPriority(manager.thread.native_handle(), cfg.priority, cfg.policy)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "Failed to set thread realtime priority.");
      }
    }

    // スレッドのCPU割当を設定
    if (cfg.affinity > 0) {
      if (!linux::setThreadCPUAffinity(manager.thread.native_handle(), cfg.affinity)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "Failed to set thread CPU affinity.");
      }
    }

    std::this_thread::sleep_for(100ms);  // 一定時間待機．さもないとスピン中にスピンを呼ぶことになってしまう．
  }

  for (auto& manager : managers) {
    manager.thread.join();
  }
}
}  // namespace ros2
