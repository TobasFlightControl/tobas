#include "../include/tobas_a1_ros/mag_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_mag_driver");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  a1::MagDriver node(node, pnh);
  rclcpp::spin();
}
