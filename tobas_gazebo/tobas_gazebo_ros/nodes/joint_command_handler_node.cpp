#include "../include/tobas_gazebo_ros/joint_command_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_joint_command_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_gazebo_ros::JointCommandHandler node(nh, pnh);
  ros::spin();
}
