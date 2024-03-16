#include "../include/tobas_calibration_ros/esc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esc_calibration");
  ros::NodeHandle nh;
  tobas_calibration::EscCalibrationRos node(nh);
  ros::spin();
}
