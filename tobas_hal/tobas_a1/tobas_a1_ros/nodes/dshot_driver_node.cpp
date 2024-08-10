#include "../include/tobas_a1_ros/dshot_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_dshot_driver");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  a1::DShotDriver node(node, pnh);
  rclcpp::spin();
}
