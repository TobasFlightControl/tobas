#include "../include/tobas_calibration_ros/esc_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "esc_calibration");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_calibration::EscCalibrationRos node(node, pnh);
  rclcpp::spin();
}
