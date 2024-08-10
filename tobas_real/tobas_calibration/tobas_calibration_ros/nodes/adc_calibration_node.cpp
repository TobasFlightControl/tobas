#include "../include/tobas_calibration_ros/adc_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "adc_calibration");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_calibration::AdcCalibrationRos node(node, pnh);
  rclcpp::spin();
}
