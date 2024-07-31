#include "../include/tobas_mr_pid/controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_mr_pid");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_pid::ControllerRos node(node, pnh);
  rclcpp::spin();
}
