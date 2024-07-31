#include "../include/tobas_real_ros/battery_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "battery_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::BatteryHandler node(node, pnh);
  rclcpp::spin();
}
