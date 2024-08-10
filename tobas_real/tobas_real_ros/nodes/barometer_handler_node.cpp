#include "../include/tobas_real_ros/barometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "barometer_handler");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_real_ros::BarometerHandler node(node, pnh);
  rclcpp::spin();
}
