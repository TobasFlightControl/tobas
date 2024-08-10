#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "takeoff_action_server");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_multirotor_takeoff::TakeoffActionServer node(node, pnh);
  rclcpp::spin();
}
