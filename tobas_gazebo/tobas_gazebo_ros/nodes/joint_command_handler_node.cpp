#include "../include/tobas_gazebo_ros/joint_command_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "gazebo_joint_command_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_gazebo_ros::JointCommandHandler node(node, pnh);
  rclcpp::spin();
}
