#include "../include/tobas_navio_ros/barometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_barometer_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::BarometerHandler node(node, pnh);
  rclcpp::spin();
}
