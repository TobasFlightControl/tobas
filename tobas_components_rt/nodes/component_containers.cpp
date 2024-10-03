#include <tobas_linux/memory_lock.hpp>

#include "tobas_components_rt/multi_component_managers.hpp"

#define LOCK_MEMORY_MB 1000

int main(int argc, char* argv[])
{
  if (!linux::lockAndPrefaultDynamic(LOCK_MEMORY_MB * (1 << 20)))
    throw std::runtime_error("Failed to lock memory.");

  rclcpp::init(argc, argv);

  ros2::MultiComponentManagers managers(SCHED_FIFO, 4, 0);
  managers.spin();

  rclcpp::shutdown();
}
