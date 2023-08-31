#include "../include/tobas_rc_teleop/velocity_yaw.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_rc_teleop::RcinToVelocityYaw node(nh, pnh);
  ros::spin();
}
