#include "../include/tobas_a1_ros/dshot_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_dshot_driver");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  a1::DShotDriver node(node, pnh);
  rclcpp::spin();
}
