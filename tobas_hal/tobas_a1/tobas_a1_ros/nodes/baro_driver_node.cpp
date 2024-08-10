#include "../include/tobas_a1_ros/baro_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_baro_driver");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  a1::BaroDriver node(node, pnh);
  rclcpp::spin();
}
