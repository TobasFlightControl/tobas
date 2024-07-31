#include "../include/tobas_a1_ros/gnss_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_gnss_driver");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  a1::GNSSDriver node(node, pnh);
  rclcpp::spin();
}
