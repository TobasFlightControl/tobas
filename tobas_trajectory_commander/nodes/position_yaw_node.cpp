#include "../include/tobas_trajectory_commander/position_yaw.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "follow_position_yaw_trajectory_server");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_trajectory_commander::FollowPositionYawTrajectoryServer node(node, pnh);
  rclcpp::spin();
}
