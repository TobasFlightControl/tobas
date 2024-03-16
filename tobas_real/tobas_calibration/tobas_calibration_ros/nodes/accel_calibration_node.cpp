#include "../include/tobas_calibration_ros/accel_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "accel_calibration");
  ros::NodeHandle nh;
  tobas_calibration::AccelCalibrationRos node(nh);
  ros::spin();
}
