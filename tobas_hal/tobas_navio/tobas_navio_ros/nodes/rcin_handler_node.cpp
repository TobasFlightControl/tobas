#include "../include/tobas_navio_ros/rcin_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_rcin_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::RCInputHandler node(node, pnh);
  rclcpp::spin();
}
