#include "../../../include/tobas_real/calibration/accel_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "accel_calibration");
  tobas_real::AccelCalibrator node;
  node.run();
}
