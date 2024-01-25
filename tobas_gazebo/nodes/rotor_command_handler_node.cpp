#include "../include/tobas_gazebo/rotor_command_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotor_command_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_gazebo::RotorCommandHandler node(nh, pnh);
  ros::spin();
}
