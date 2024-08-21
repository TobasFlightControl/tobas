#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_bridge");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr node("~");
  TobasBridge node(node, node);
  rclcpp::spin();
}
