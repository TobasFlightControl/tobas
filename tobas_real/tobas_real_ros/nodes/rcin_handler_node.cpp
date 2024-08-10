#include "../include/tobas_real_ros/rcin_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rcin_handler");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_real_ros::RCInputHandler node(node, pnh);
  rclcpp::spin();
}
