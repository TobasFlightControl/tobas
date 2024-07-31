#include "../include/tobas_navio_ros/pwm_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "navio_pwm_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_navio_ros::PwmHandler node(node, pnh);
  rclcpp::spin();
}
