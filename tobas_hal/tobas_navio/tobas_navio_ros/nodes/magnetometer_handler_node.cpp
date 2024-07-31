#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_magnetometer_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::MagnetometerHandler node(node, pnh);
  rclcpp::spin();
}
