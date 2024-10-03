#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sched.h>

namespace ros2
{
/* MultiThreadedExecutorのスレッドプールにリアルタイム優先度を付与． */
class MultiThreadedExecutorRT : public rclcpp::executors::MultiThreadedExecutor
{
public:
  using SharedPtr = std::shared_ptr<MultiThreadedExecutorRT>;

  explicit MultiThreadedExecutorRT(size_t priority, int policy, size_t num_threads = 0);

  void spin() override;

  size_t priority() const;
  int policy() const;

private:
  const size_t priority_;
  const int policy_;
};
}  // namespace ros2
