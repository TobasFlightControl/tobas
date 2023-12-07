#include "../include/tobas_joint_control/joint_control_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_joint_control::JointControlRos node(nh, pnh);
  ros::spin();
}
