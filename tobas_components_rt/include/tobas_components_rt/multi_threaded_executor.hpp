#include <sched.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace ros2
{
/* MultiThreadedExecutorのスレッドプールにリアルタイム優先度を付与． */
class MultiThreadedExecutorRT : public rclcpp::executors::MultiThreadedExecutor
{
  static constexpr char kName[] = "multi_threaded_executor_rt";

public:
  using SharedPtr = std::shared_ptr<MultiThreadedExecutorRT>;

  explicit MultiThreadedExecutorRT(
    int policy = SCHED_FIFO,
    size_t priority = 0,
    uint32_t cpu_affinity = 0,
    size_t num_threads = 0);

  void spin() override;

  size_t priority() const;
  int policy() const;

private:
  const int policy_;
  const size_t priority_;
  const uint32_t cpu_affinity_;
};
}  // namespace ros2
