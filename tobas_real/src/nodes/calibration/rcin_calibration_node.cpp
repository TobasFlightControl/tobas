#include "../../../include/tobas_real/calibration/rcin_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_input_calibration");
  tobas_real::RCInputCalibrator node;
  node.run();
}
