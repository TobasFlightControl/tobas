#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_bridge");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  TobasBridge node(node, pnh);
  rclcpp::spin();
}
