#include "../include/tobas_calibration_ros/mag_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mag_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_calibration::MagCalibrationRos node(nh, pnh);
  ros::spin();
}
