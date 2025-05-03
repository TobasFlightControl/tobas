#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_threaded_executor.hpp"

using namespace std;

namespace ros2
{
MultiThreadedExecutorRT::MultiThreadedExecutorRT(int policy, size_t priority, uint32_t cpu_affinity, size_t num_threads)
  : rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), num_threads)
  , policy_(policy)
  , priority_(priority)
  , cpu_affinity_(cpu_affinity)
{
}

void MultiThreadedExecutorRT::spin()
{
  if (spinning.exchange(true)) {
    throw runtime_error("spin() called while already spinning.");
  }

  RCPPUTILS_SCOPE_EXIT(wait_result_.reset(); spinning.store(false););

  vector<thread> threads;
  for (size_t thread_id = 0; thread_id < get_number_of_threads(); ++thread_id) {
    auto func = bind(&MultiThreadedExecutorRT::run, this, thread_id);
    threads.emplace_back(func);

    // スレッドのリアルタイム優先度を設定
    if (priority_ > 0) {
      if (!linux::setThreadPriority(threads.back().native_handle(), priority_, policy_)) {
        RCLCPP_WARN(rclcpp::get_logger(kName), "Failed to set thread realtime priority.");
      }
    }

    // スレッドのCPU割当を設定
    if (cpu_affinity_ > 0) {
      if (!linux::setThreadCPUAffinity(threads.back().native_handle(), cpu_affinity_)) {
        RCLCPP_WARN(rclcpp::get_logger(kName), "Failed to set thread CPU affinity.");
      }
    }
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

size_t MultiThreadedExecutorRT::priority() const
{
  return priority_;
}

int MultiThreadedExecutorRT::policy() const
{
  return policy_;
}
}  // namespace ros2
