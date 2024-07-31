#include "../include/tobas_navio_ros/battery_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_battery_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::BatteryHandler node(node, pnh);
  rclcpp::spin();
}
