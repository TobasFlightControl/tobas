#include "../include/tobas_real_ros/barometer_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "barometer_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_real_ros::BarometerHandler node(node, pnh);
  rclcpp::spin();
}
