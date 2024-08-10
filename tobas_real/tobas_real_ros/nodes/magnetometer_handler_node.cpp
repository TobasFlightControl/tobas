#include "../include/tobas_real_ros/magnetometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "magnetometer_handler");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_real_ros::MagnetometerHandler node(node, pnh);
  rclcpp::spin();
}
