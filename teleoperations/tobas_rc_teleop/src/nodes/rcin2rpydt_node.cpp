#include "../../include/tobas_rc_teleop/rcin2rpydt.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_rpydt_converter");
  tobas_rc_teleop::RcinToRollPitchYawrateThrust node;
  ros::spin();
}
