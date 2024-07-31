#include "../include/tobas_mr_arducopter/controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "arducopter_controller");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_arducopter::ControllerRos node(node, pnh);
  rclcpp::spin();
}
