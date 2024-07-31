#include "../include/tobas_a1_ros/pwm_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_pwm_driver");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  a1::PWMDriver node(node, pnh);
  rclcpp::spin();
}
