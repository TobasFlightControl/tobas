#include "../include/tobas_real_ros/cpu_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "cpu_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::CpuHandler node(node, pnh);
  rclcpp::spin();
}
