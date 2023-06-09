#include <unistd.h>

#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/esc_calibration.hpp"

using namespace std;

int main(int argc, char** argv)
{
  if (getuid())
  {
    throw runtime_error("Not root.");
  }

  ros::init(argc, argv, "esc_calibration");
  tobas_real::EscCalibrator esc_calibrator;
  esc_calibrator.run();
}
