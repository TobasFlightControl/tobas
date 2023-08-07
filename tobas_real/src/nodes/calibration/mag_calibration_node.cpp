#include "../../../include/tobas_real/calibration/mag_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mag_calibration");
  tobas_real::MagnetometerCalibrator node;
  node.run();
}
