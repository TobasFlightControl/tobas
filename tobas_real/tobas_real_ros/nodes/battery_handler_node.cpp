#include "../include/tobas_real_ros/battery_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "battery_handler");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_real_ros::BatteryHandler node(node, pnh);
  rclcpp::spin();
}
