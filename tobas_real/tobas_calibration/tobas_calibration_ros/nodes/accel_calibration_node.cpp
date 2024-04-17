#include "../include/tobas_calibration_ros/accel_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "accel_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_calibration::AccelCalibrationRos node(nh, pnh);
  ros::spin();
}
