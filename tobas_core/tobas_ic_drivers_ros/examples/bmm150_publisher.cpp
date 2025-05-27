#include "tobas_ic_drivers_ros/bmm150_publisher.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bmm150Publisher>());
  rclcpp::shutdown();
  return 0;
}
