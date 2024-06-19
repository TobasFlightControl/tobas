#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_rotor_command_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_gazebo_ros::RotorCommandHandler node(nh, pnh);
  ros::spin();
}
