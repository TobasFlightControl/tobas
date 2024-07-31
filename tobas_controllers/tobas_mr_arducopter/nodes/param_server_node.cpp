#include "../include/tobas_mr_arducopter/param_server_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "arducopter_param_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_arducopter::ParamServerRos node(node, pnh);
  rclcpp::spin();
}
