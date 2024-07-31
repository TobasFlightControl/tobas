#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "gazebo_rotor_command_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_gazebo_ros::RotorCommandHandler node(node, pnh);
  rclcpp::spin();
}
