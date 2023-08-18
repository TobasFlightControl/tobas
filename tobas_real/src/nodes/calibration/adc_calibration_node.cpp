#include "../../../include/tobas_real/calibration/adc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adc_calibration");
  tobas_real::AdcCalibrator node;
  node.run();
}
