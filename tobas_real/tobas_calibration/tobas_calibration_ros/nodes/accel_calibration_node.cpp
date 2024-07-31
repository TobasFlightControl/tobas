#include "../include/tobas_calibration_ros/accel_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "accel_calibration");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_calibration::AccelCalibrationRos node(node, pnh);
  rclcpp::spin();
}
