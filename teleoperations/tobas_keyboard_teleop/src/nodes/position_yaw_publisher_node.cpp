#include "../../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  tobas_keyboard_teleop::PositionYawPublisher node;
  node.run();
}
