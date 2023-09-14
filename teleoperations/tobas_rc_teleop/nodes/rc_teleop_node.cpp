#include "../include/tobas_rc_teleop/rc_teleop.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_rc_teleop::RCTeleop node(nh, pnh);
  ros::spin();
}
