#include "../include/tobas_calibration_ros/adc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adc_calibration");
  ros::NodeHandle nh;
  tobas_calibration::AdcCalibrationRos node(nh);
  ros::spin();
}
