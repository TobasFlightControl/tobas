#include <tobas_linux/realtime.hpp>

#include "../include/tobas_components_rt/multi_threaded_executor.hpp"

using namespace std;

namespace ros2
{
MultiThreadedExecutorRT::MultiThreadedExecutorRT(size_t priority, int policy, size_t num_threads)
  : rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), num_threads),
    priority_(priority),
    policy_(policy)
{
}

void MultiThreadedExecutorRT::spin()
{
  if (spinning.exchange(true))
    throw runtime_error("spin() called while already spinning.");

  RCPPUTILS_SCOPE_EXIT(wait_result_.reset(); spinning.store(false););

  vector<thread> threads;
  for (size_t thread_id = 0; thread_id < get_number_of_threads(); ++thread_id)
  {
    auto func = bind(&MultiThreadedExecutorRT::run, this, thread_id);
    threads.emplace_back(func);

    // スレッドのリアルタイム優先度を設定
    if (!linux::setThreadPriority(threads.back().native_handle(), priority_, policy_))
      RCLCPP_WARN(rclcpp::get_logger("multi_threaded_executor_rt"), "Failed to set realtime thread priority.");
  }

  for (auto& thread : threads)
    thread.join();
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
