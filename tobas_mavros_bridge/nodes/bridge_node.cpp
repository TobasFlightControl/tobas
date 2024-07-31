#include "../include/tobas_mavros_bridge/bridge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_mr_arducopter");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mavros_bridge::TobasMavrosBridge node(node, pnh);
  rclcpp::spin();
}
