#include "../include/tobas_calibration_ros/rcin_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_calibration::RCInputCalibrationRos node(nh, pnh);
  ros::spin();
}
