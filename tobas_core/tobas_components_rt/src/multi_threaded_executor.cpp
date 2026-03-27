#include "tobas_components_rt/multi_threaded_executor.hpp"

#include <tobas_linux/realtime.hpp>

namespace tobas
{
MultiThreadedExecutorRT::MultiThreadedExecutorRT(
  linux::sched_t policy,
  size_t priority,
  uint32_t cpu_affinity,
  size_t num_threads)
  : rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), num_threads)
  , policy_(policy)
  , priority_(priority)
  , cpu_affinity_(cpu_affinity)
{
}

void MultiThreadedExecutorRT::spin()
{
  constexpr char kLoggerName[] = "multi_threaded_executor_rt";

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning.");
  }

  RCPPUTILS_SCOPE_EXIT(wait_result_.reset(); spinning.store(false););

  std::vector<std::thread> threads;
  for (size_t thread_id = 0; thread_id < get_number_of_threads(); ++thread_id) {
    auto func = std::bind(&MultiThreadedExecutorRT::run, this, thread_id);
    threads.emplace_back(func);

    // スレッドのリアルタイム優先度を設定
    if (priority_ > 0) {
      if (!linux::setThreadPriority(threads.back().native_handle(), priority_, policy_)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "Failed to set thread realtime priority.");
      }
    }

    // スレッドのCPU割当を設定
    if (cpu_affinity_ > 0) {
      if (!linux::setThreadCPUAffinity(threads.back().native_handle(), cpu_affinity_)) {
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "Failed to set thread CPU affinity.");
      }
    }
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

int MultiThreadedExecutorRT::policy() const
{
  return policy_;
}

size_t MultiThreadedExecutorRT::priority() const
{
  return priority_;
}
}  // namespace tobas
