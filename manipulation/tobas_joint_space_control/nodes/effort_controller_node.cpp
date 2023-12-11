#include "../include/tobas_joint_space_control/effort_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_controller_effort");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_joint_space_control::EffortControllerRos node(nh, pnh);
  ros::spin();
}
