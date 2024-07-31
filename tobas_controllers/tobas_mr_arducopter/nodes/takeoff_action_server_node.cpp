#include "../include/tobas_mr_arducopter/takeoff_action_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "takeoff_action_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_arducopter::TakeoffActionServer node(node, pnh);
  rclcpp::spin();
}
