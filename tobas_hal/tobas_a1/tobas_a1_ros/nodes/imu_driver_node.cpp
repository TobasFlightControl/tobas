#include "../include/tobas_a1_ros/imu_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_imu_driver");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  a1::IMUDriver node(node, pnh);
  rclcpp::spin();
}
