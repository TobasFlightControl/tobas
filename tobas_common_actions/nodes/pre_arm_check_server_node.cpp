#include "../include/tobas_common_actions/pre_arm_check_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pre_arm_check_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_common_actions::PreArmCheckServer node(nh, pnh);
  ros::spin();
}
