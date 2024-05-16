#include "../include/tobas_calibration_ros/adc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adc_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_calibration::AdcCalibrationRos node(nh, pnh);
  ros::spin();
}
