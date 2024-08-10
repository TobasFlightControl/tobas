#include "../include/tobas_state_checker/state_checker.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "state_checker");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_state_checker::StateChecker node(node, pnh);
  rclcpp::spin();
}
