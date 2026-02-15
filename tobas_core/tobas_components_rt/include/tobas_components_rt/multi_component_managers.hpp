#pragma once

#include <sched.h>

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
class MultiComponentManagers
{
public:
  explicit MultiComponentManagers(size_t num_managers);

  void setPolicy(size_t idx, int policy);
  void setPriority(size_t idx, size_t priority);
  void setCpuAffinity(size_t idx, uint32_t affinity);
  void setNumThreads(size_t idx, size_t num_threads);

  void spin();

private:
  struct ManagerConfig
  {
    int policy = SCHED_FIFO;
    size_t priority = 0;
    uint32_t affinity = 0;
    size_t num_threads = 0;
  };

  const size_t num_managers_;
  std::vector<ManagerConfig> configs_;
};
}  // namespace ros2
