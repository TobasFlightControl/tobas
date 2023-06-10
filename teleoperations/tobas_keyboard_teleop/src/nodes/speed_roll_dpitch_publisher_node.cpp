#include "../../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speed_roll_delta_pitch_publisher");
  tobas_keyboard_teleop::SpeedRollDeltaPitchPublisher node;
  node.run();
}
