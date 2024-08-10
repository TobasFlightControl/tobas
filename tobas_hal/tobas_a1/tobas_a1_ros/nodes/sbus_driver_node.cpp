#include "../include/tobas_a1_ros/sbus_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_sbus_driver");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  a1::SBUSDriver node(node, pnh);
  rclcpp::spin();
}
