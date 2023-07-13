#include "../../include/tobas_real/esc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esc_calibration");
  tobas_real::EscCalibrator node;
  node.run();
}
