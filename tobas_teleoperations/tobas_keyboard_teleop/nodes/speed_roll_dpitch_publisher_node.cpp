#include "../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "keyboard_teleop");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_keyboard_teleop::SpeedRollDeltaPitchPublisher node(node, pnh);
  node.run();
}
