#include "../include/tobas_calibration/mag_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mag_calibration");
  ros::NodeHandle nh;
  tobas_calibration::MagCalibrationRos node(nh);
  ros::spin();
}
