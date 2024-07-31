#include "../include/tobas_real_ros/time_reference_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "time_reference_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::TimeReferenceServer node(node, pnh);
  rclcpp::spin();
}
