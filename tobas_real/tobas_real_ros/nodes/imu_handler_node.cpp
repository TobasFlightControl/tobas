#include "../include/tobas_real_ros/imu_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "imu_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::ImuHandler node(node, pnh);
  rclcpp::spin();
}
