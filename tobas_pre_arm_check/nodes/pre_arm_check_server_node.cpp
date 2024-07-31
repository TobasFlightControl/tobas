#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "pre_arm_check_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_pre_arm_check::PreArmCheckServer node(node, pnh);
  rclcpp::spin();
}
