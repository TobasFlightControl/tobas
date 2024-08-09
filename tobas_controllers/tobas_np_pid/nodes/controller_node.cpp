#include "../include/tobas_np_pid/controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_np_pid");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_np_pid::ControllerNode node(node, pnh);
  rclcpp::spin();
}
