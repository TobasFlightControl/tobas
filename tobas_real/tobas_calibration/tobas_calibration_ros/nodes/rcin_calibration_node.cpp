#include "../include/tobas_calibration_ros/rcin_calibration.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rcin_calibration");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_calibration::RCInputCalibrationRos node(node, pnh);
  rclcpp::spin();
}
