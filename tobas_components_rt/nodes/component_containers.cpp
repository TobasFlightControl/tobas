#include "tobas_components_rt/multi_component_managers.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  ros2::MultiComponentManagers managers(SCHED_FIFO, 4, 0);
  managers.spin();

  rclcpp::shutdown();
}
