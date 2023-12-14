#include "../include/tobas_gazebo/joint_command_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_command_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_gazebo::JointCommandHandler node(nh, pnh);
  ros::spin();
}
