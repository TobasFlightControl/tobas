#include "../include/tobas_a1_ros/baro_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_baro_driver");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  a1::BaroDriver node(node, pnh);
  rclcpp::spin();
}
