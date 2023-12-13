#include "../include/tobas_manipulation/effort_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_space_controller_effort");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_manipulation::EffortControllerRos node(nh, pnh);
  ros::spin();
}
