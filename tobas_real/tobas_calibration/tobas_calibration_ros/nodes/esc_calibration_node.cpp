#include "../include/tobas_calibration_ros/esc_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "esc_calibration");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_calibration::EscCalibrationRos node(node, pnh);
  rclcpp::spin();
}
