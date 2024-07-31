#include "../include/tobas_navio_ros/gps_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_gps_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::GpsHandler node(node, pnh);
  rclcpp::spin();
}
