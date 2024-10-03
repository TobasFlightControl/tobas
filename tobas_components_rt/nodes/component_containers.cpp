#include "tobas_components_rt/multi_component_managers.hpp"

#define SCHED_POLICY SCHED_FIFO
#define NUM_MANAGERS 4
#define NUM_THREADS 0

static void sigIntHandler(int)
{
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  ros2::MultiComponentManagers managers(SCHED_FIFO, NUM_MANAGERS, NUM_THREADS);
  managers.spin();

  rclcpp::shutdown();
}
