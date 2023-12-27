#include "../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_keyboard_teleop::SpeedRollDeltaPitchPublisher node(nh, pnh);
  node.run();
}
