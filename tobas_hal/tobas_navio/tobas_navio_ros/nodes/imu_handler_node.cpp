#include "../include/tobas_navio_ros/imu_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_imu_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::ImuHandler node(node, pnh);
  rclcpp::spin();
}
