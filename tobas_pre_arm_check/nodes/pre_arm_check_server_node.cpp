#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pre_arm_check_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_pre_arm_check::PreArmCheckServer node(nh, pnh);
  ros::spin();
}
