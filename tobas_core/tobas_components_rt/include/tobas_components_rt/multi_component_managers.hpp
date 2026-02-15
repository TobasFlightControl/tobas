#pragma once

#include <sched.h>

#include <rclcpp/rclcpp.hpp>

#include <tobas_linux/types.hpp>

namespace ros2
{
class MultiComponentManagers
{
public:
  explicit MultiComponentManagers(size_t num_managers);

  void setPolicy(size_t idx, linux::sched_t policy);
  void setPriority(size_t idx, size_t priority);
  void setCpuAffinity(size_t idx, uint32_t affinity);
  void setNumThreads(size_t idx, size_t num_threads);

  void spin();

private:
  struct ManagerConfig
  {
    linux::sched_t policy = SCHED_FIFO;
    size_t priority = 0;
    uint32_t affinity = 0;
    size_t num_threads = 1;
  };

  const size_t num_managers_;
  std::vector<ManagerConfig> configs_;
};
}  // namespace ros2
