#include "../include/tobas_real_ros/magnetometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "magnetometer_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::MagnetometerHandler node(node, pnh);
  rclcpp::spin();
}
