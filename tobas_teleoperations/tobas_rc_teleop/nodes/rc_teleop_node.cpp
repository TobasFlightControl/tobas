#include "../include/tobas_rc_teleop/rc_teleop.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rc_teleop");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_rc_teleop::RCTeleop node(node, pnh);
  rclcpp::spin();
}
