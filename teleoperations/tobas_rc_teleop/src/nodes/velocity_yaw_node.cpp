#include "../../include/tobas_rc_teleop/velocity_yaw.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_to_velocity_yaw");
  tobas_rc_teleop::RcinToVelocityYaw node;
  ros::spin();
}
