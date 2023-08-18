#include "../../include/tobas_rc_teleop/roll_pitch_yawrate_thrust.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_teleop");
  tobas_rc_teleop::RcinToRollPitchYawrateThrust node;
  ros::spin();
}
