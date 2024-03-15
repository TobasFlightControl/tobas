#include "../include/tobas_calibration/rcin_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_calibration");
  ros::NodeHandle nh;
  tobas_calibration::RCInputCalibrationRos node(nh);
  ros::spin();
}
