#include "../include/tobas_calibration_ros/mag_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "mag_calibration");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_calibration::MagCalibrationRos node(node, pnh);
  rclcpp::spin();
}
