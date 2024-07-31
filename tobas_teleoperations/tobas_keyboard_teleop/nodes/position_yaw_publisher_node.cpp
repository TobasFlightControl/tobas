#include "../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "keyboard_teleop");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_keyboard_teleop::PositionYawPublisher node(node, pnh);
  node.run();
}
