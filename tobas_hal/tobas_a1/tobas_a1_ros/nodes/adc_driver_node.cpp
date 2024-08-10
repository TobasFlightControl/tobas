#include "../include/tobas_a1_ros/adc_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "a1_adc_driver");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  a1::ADCDriver node(node, pnh);
  rclcpp::spin();
}
